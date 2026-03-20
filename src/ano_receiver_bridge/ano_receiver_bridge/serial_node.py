import math
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from ano_receiver_bridge.decoder import decode_frame
from ano_receiver_bridge.parser import AnoFrameParser
from ano_receiver_bridge.protocol_types import (
    ATTITUDE_FRAME_ID,
    GENERAL_DISTANCE_FRAME_ID,
    GENERAL_VELOCITY_FRAME_ID,
    QUATERNION_FRAME_ID,
    VELOCITY_FRAME_ID,
    AttitudeData,
    GeneralDistanceData,
    QuaternionData,
    VelocityData,
)


class AnoReceiverNode(Node):
    """Serial receive-only bridge for ANO flight controller telemetry frames."""

    def __init__(self) -> None:
        super().__init__('ano_receiver_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('frame_info_enable', True)
        self.declare_parameter('frame_id', 'ano_link')

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._timeout = self.get_parameter('timeout').value
        self._frame_info_enable = self.get_parameter('frame_info_enable').value
        self._frame_id = self.get_parameter('frame_id').value

        self._parser = AnoFrameParser()
        self._serial: Optional[serial.Serial] = None
        self._last_open_attempt_ns = 0

        self._attitude_pub = self.create_publisher(Vector3Stamped, '/ano/attitude', 10)
        self._quaternion_pub = self.create_publisher(QuaternionStamped, '/ano/quaternion', 10)
        self._velocity_pub = self.create_publisher(Vector3Stamped, '/ano/velocity', 10)
        self._general_velocity_pub = self.create_publisher(Vector3Stamped, '/ano/general_velocity', 10)
        self._general_distance_pub = self.create_publisher(Float32MultiArray, '/ano/general_distance', 10)
        self._frame_info_pub = self.create_publisher(String, '/ano/frame_info', 10)

        self.create_timer(0.005, self._poll_serial)
        self._open_serial()

    def _open_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self.get_logger().info(
                f'Opened {self._port} at {self._baudrate} baud with timeout {self._timeout:.3f}s'
            )
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(f'Failed to open serial port {self._port}: {exc}')

    def _poll_serial(self) -> None:
        if self._serial is None:
            self._retry_open_if_needed()
            return

        try:
            chunk = self._serial.read(256)
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            self._close_serial()
            return

        if not chunk:
            return

        for byte in chunk:
            frame = self._parser.feed_byte(byte)
            if frame is None:
                continue

            decoded = decode_frame(frame)
            if decoded is None:
                self._publish_frame_info(f'id=0x{frame.frame_id:02X} unsupported')
                continue

            self._publish_decoded(frame.frame_id, decoded)

    def _publish_decoded(self, frame_id: int, decoded) -> None:
        stamp = self.get_clock().now().to_msg()

        if frame_id == ATTITUDE_FRAME_ID and isinstance(decoded, AttitudeData):
            msg = Vector3Stamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.vector.x = decoded.roll_deg
            msg.vector.y = decoded.pitch_deg
            msg.vector.z = decoded.yaw_deg
            self._attitude_pub.publish(msg)
            self._publish_frame_info(f'id=0x{frame_id:02X} ok')
            return

        if frame_id == QUATERNION_FRAME_ID and isinstance(decoded, QuaternionData):
            msg = QuaternionStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.quaternion.w = decoded.w
            msg.quaternion.x = decoded.x
            msg.quaternion.y = decoded.y
            msg.quaternion.z = decoded.z
            self._quaternion_pub.publish(msg)
            self._publish_frame_info(f'id=0x{frame_id:02X} ok')
            return

        if frame_id == VELOCITY_FRAME_ID and isinstance(decoded, VelocityData):
            self._velocity_pub.publish(self._build_vector3(stamp, decoded))
            self._publish_frame_info(f'id=0x{frame_id:02X} ok')
            return

        if frame_id == GENERAL_VELOCITY_FRAME_ID and isinstance(decoded, VelocityData):
            self._general_velocity_pub.publish(self._build_vector3(stamp, decoded))
            if any(math.isnan(value) for value in (decoded.x_mps, decoded.y_mps, decoded.z_mps)):
                self._publish_frame_info(f'id=0x{frame_id:02X} invalid component')
            else:
                self._publish_frame_info(f'id=0x{frame_id:02X} ok')
            return

        if frame_id == GENERAL_DISTANCE_FRAME_ID and isinstance(decoded, GeneralDistanceData):
            msg = Float32MultiArray()
            msg.data = [decoded.direction, decoded.angle_deg, decoded.distance_m]
            self._general_distance_pub.publish(msg)
            if math.isnan(decoded.distance_m):
                self._publish_frame_info(f'id=0x{frame_id:02X} invalid distance')
            else:
                self._publish_frame_info(f'id=0x{frame_id:02X} ok')

    def _build_vector3(self, stamp, decoded: VelocityData) -> Vector3Stamped:
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.vector.x = decoded.x_mps
        msg.vector.y = decoded.y_mps
        msg.vector.z = decoded.z_mps
        return msg

    def _publish_frame_info(self, text: str) -> None:
        if not self._frame_info_enable:
            return
        msg = String()
        msg.data = text
        self._frame_info_pub.publish(msg)

    def _retry_open_if_needed(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_open_attempt_ns < 1_000_000_000:
            return
        self._last_open_attempt_ns = now_ns
        self._open_serial()

    def _close_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self.get_logger().info(f'Closed serial port {self._port}')
        self._serial = None

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
