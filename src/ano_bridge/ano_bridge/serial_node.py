import math
import select
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ano_bridge.encoder import encode_realtime_control, saturate_int16
from ano_bridge.frame_dispatcher import (
    build_attitude_stamped,
    build_float_array,
    build_quaternion_stamped,
    build_vector3_stamped,
)
from ano_bridge.messages import AttitudeMessage, FloatArrayMessage, QuaternionMessage, Vector3Message
from ano_bridge.parser import AnoStreamParser, parse_payload
from ano_bridge.protocol import (
    ATTITUDE_FRAME_ID,
    DEFAULT_ADDRESS,
    GENERAL_DISTANCE_FRAME_ID,
    GENERAL_VELOCITY_FRAME_ID,
    QUATERNION_FRAME_ID,
    REALTIME_CONTROL_FRAME_ID,
    RealtimeControlCommand,
    VELOCITY_FRAME_ID,
)


class SerialNode(Node):
    def __init__(self) -> None:
        super().__init__('ano_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('address', DEFAULT_ADDRESS)
        self.declare_parameter('frame_id', 'ano_link')
        self.declare_parameter('read_chunk_size', 256)
        self.declare_parameter('read_period_s', 0.005)
        self.declare_parameter('command_resend_hz', 30.0)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._address = self.get_parameter('address').value
        self._frame_id = self.get_parameter('frame_id').value
        self._read_chunk_size = self.get_parameter('read_chunk_size').value
        resend_hz = self.get_parameter('command_resend_hz').value
        read_period = self.get_parameter('read_period_s').value

        self._serial: Optional[serial.Serial] = None
        self._parser = AnoStreamParser()
        self._last_command = RealtimeControlCommand()
        self._has_command = False
        self._last_open_attempt_ns = 0

        self._attitude_pub = self.create_publisher(Vector3Stamped, '/ano/attitude', 10)
        self._quaternion_pub = self.create_publisher(QuaternionStamped, '/ano/quaternion', 10)
        self._velocity_pub = self.create_publisher(Vector3Stamped, '/ano/velocity', 10)
        self._general_velocity_pub = self.create_publisher(Vector3Stamped, '/ano/general_velocity', 10)
        self._general_distance_pub = self.create_publisher(Float32MultiArray, '/ano/general_distance', 10)

        self.create_subscription(Twist, '/ano/cmd_vel_body', self._handle_cmd_vel, 10)
        self.create_timer(read_period, self._poll_serial)
        self.create_timer(1.0 / resend_hz, self._resend_last_command)

        self._open_serial()

    def _open_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.0,
                write_timeout=0.0,
            )
            self.get_logger().info(
                f'Opened serial port {self._port} at {self._baudrate} baud, addr=0x{self._address:02X}'
            )
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(f'Failed to open serial port {self._port}: {exc}')

    def _poll_serial(self) -> None:
        if self._serial is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_open_attempt_ns > 1_000_000_000:
                self._last_open_attempt_ns = now_ns
                self._open_serial()
            return

        try:
            ready, _, _ = select.select([self._serial.fileno()], [], [], 0.0)
            if not ready and self._serial.in_waiting == 0:
                return

            bytes_to_read = min(max(1, self._serial.in_waiting), self._read_chunk_size)
            data = self._serial.read(bytes_to_read)
            if not data:
                return

            for frame in self._parser.append(data):
                decoded = parse_payload(frame)
                if decoded is None:
                    continue
                self._publish_frame(frame.frame_id, decoded)
        except (serial.SerialException, OSError, ValueError) as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            self._close_serial()

    def _publish_frame(self, frame_id: int, decoded) -> None:
        stamp = self.get_clock().now().to_msg()

        if frame_id == ATTITUDE_FRAME_ID and isinstance(decoded, AttitudeMessage):
            self._attitude_pub.publish(build_attitude_stamped(stamp, self._frame_id, decoded))
            return

        if frame_id == QUATERNION_FRAME_ID and isinstance(decoded, QuaternionMessage):
            self._quaternion_pub.publish(build_quaternion_stamped(stamp, self._frame_id, decoded))
            return

        if frame_id == VELOCITY_FRAME_ID and isinstance(decoded, Vector3Message):
            self._velocity_pub.publish(build_vector3_stamped(stamp, self._frame_id, decoded))
            return

        if frame_id == GENERAL_VELOCITY_FRAME_ID and isinstance(decoded, Vector3Message):
            self._general_velocity_pub.publish(build_vector3_stamped(stamp, self._frame_id, decoded))
            return

        if frame_id == GENERAL_DISTANCE_FRAME_ID and isinstance(decoded, FloatArrayMessage):
            self._general_distance_pub.publish(build_float_array(decoded))

    def _handle_cmd_vel(self, msg: Twist) -> None:
        command = RealtimeControlCommand(
            ctrl_spd_x=saturate_int16(msg.linear.x * 100.0),
            ctrl_spd_y=saturate_int16(msg.linear.y * 100.0),
            ctrl_spd_z=saturate_int16(msg.linear.z * 100.0),
            ctrl_yaw_dps=saturate_int16(math.degrees(msg.angular.z)),
        )
        self._last_command = command
        self._has_command = True
        self._write_command(command)

    def _resend_last_command(self) -> None:
        if not self._has_command:
            return
        self._write_command(self._last_command)

    def _write_command(self, command: RealtimeControlCommand) -> None:
        if self._serial is None:
            return

        frame = encode_realtime_control(command, self._address)
        try:
            self._serial.write(frame)
            self.get_logger().debug(
                f'Sent 0x{REALTIME_CONTROL_FRAME_ID:02X} control frame ({len(frame)} bytes)'
            )
        except serial.SerialTimeoutException as exc:
            self.get_logger().warning(f'Serial write timeout: {exc}')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self._close_serial()

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()

    def _close_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self.get_logger().info(f'Closed serial port {self._port}')
        self._serial = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
