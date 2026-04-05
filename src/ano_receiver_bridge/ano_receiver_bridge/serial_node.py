import json
import math
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import QuaternionStamped, Twist, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from ano_receiver_bridge.candidate_state import (
    FlowCandidateState,
    FlowHealthState,
    ImuCandidateState,
    ImuHealthState,
)
from ano_receiver_bridge.decoder import decode_frame
from ano_receiver_bridge.encoder import clamp_int16, encode_realtime_control_frame
from ano_receiver_bridge.parser import AnoFrameParser
from ano_receiver_bridge.protocol_types import (
    ATTITUDE_FRAME_ID,
    FLOW_OBS_FRAME_ID,
    GENERAL_DISTANCE_FRAME_ID,
    GENERAL_VELOCITY_FRAME_ID,
    IMU_RAW_FRAME_ID,
    QUATERNION_FRAME_ID,
    REALTIME_CONTROL_FRAME_ID,
    VELOCITY_FRAME_ID,
    AttitudeData,
    FlowObsData,
    GeneralDistanceData,
    ImuRawData,
    QuaternionData,
    RealtimeControlCommand,
    VelocityData,
)


class AnoReceiverNode(Node):
    """Serial bridge for ANO telemetry receive and 0x41 realtime control transmit."""

    def __init__(self) -> None:
        super().__init__('ano_receiver_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('frame_info_enable', True)
        self.declare_parameter('frame_id', 'ano_link')
        self.declare_parameter('cmd_resend_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('send_debug_enable', False)
        self.declare_parameter('candidate_source_prefix', 'stm32_uart3')
        self.declare_parameter('health_publish_hz', 1.0)
        self.declare_parameter('stream_timeout_sec', 1.0)
        self.declare_parameter('flow_jump_threshold_mps', 1.5)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._timeout = self.get_parameter('timeout').value
        self._frame_info_enable = self.get_parameter('frame_info_enable').value
        self._frame_id = self.get_parameter('frame_id').value
        self._cmd_resend_hz = self.get_parameter('cmd_resend_hz').value
        self._cmd_timeout_sec = self.get_parameter('cmd_timeout_sec').value
        self._send_debug_enable = self.get_parameter('send_debug_enable').value
        self._candidate_source_prefix = str(
            self.get_parameter('candidate_source_prefix').value
        )
        self._health_publish_hz = max(0.2, float(self.get_parameter('health_publish_hz').value))
        self._stream_timeout_sec = max(0.1, float(self.get_parameter('stream_timeout_sec').value))
        self._flow_jump_threshold_mps = float(
            self.get_parameter('flow_jump_threshold_mps').value
        )

        self._parser = AnoFrameParser()
        self._serial: Optional[serial.Serial] = None
        self._last_open_attempt_ns = 0
        self._last_command = RealtimeControlCommand()
        self._last_command_time = None

        self._attitude_pub = self.create_publisher(Vector3Stamped, '/ano/attitude', 10)
        self._quaternion_pub = self.create_publisher(QuaternionStamped, '/ano/quaternion', 10)
        self._velocity_pub = self.create_publisher(Vector3Stamped, '/ano/velocity', 10)
        self._general_velocity_pub = self.create_publisher(Vector3Stamped, '/ano/general_velocity', 10)
        self._general_distance_pub = self.create_publisher(Float32MultiArray, '/ano/general_distance', 10)
        self._frame_info_pub = self.create_publisher(String, '/ano/frame_info', 10)
        self._imu_candidate_pub = self.create_publisher(
            String, '/observation/imu_candidate_state', 10
        )
        self._flow_candidate_pub = self.create_publisher(
            String, '/observation/flow_candidate_state', 10
        )
        self._imu_health_pub = self.create_publisher(String, '/observation/imu_health_status', 10)
        self._flow_health_pub = self.create_publisher(
            String, '/observation/flow_health_status', 10
        )

        self._last_imu_log_ns = 0
        self._last_flow_log_ns = 0
        self._decode_error_count = 0
        self._imu_frame_count = 0
        self._flow_frame_count = 0
        self._imu_first_rx_ns = 0
        self._flow_first_rx_ns = 0
        self._last_imu_rx_ns = 0
        self._last_flow_rx_ns = 0
        self._last_imu_sample = None
        self._last_flow_sample = None
        self._imu_same_sample_count = 0
        self._flow_large_jump = False

        self.create_subscription(Twist, '/ano/cmd_vel_body', self._handle_cmd_vel_body, 10)
        self.create_timer(0.005, self._poll_serial)
        self.create_timer(1.0 / self._cmd_resend_hz, self._resend_command)
        self.create_timer(1.0 / self._health_publish_hz, self._publish_health)
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
                self._decode_error_count += 1
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

        if frame_id == IMU_RAW_FRAME_ID and isinstance(decoded, ImuRawData):
            stamp_ms = self._stamp_to_ms(stamp)
            state = ImuCandidateState(
                stamp_ms=stamp_ms,
                acc_x=decoded.acc_x,
                acc_y=decoded.acc_y,
                acc_z=decoded.acc_z,
                gyr_x=decoded.gyr_x,
                gyr_y=decoded.gyr_y,
                gyr_z=decoded.gyr_z,
                valid=True,
                source=f'{self._candidate_source_prefix}:imu_raw',
            )
            self._update_imu_health(decoded, stamp_ms)
            self._publish_json(self._imu_candidate_pub, state.to_dict())
            self._publish_frame_info(
                '0x08 imu_raw '
                f'acc=({decoded.acc_x},{decoded.acc_y},{decoded.acc_z}) '
                f'gyr=({decoded.gyr_x},{decoded.gyr_y},{decoded.gyr_z})'
            )
            self._maybe_log_imu(state)
            return

        if frame_id == FLOW_OBS_FRAME_ID and isinstance(decoded, FlowObsData):
            stamp_ms = self._stamp_to_ms(stamp)
            state = FlowCandidateState(
                stamp_ms=stamp_ms,
                flow_vx=decoded.flow_vx,
                flow_vy=decoded.flow_vy,
                flow_state=decoded.flow_state,
                flow_quality=decoded.flow_quality,
                alt_cm=decoded.alt_cm,
                valid=decoded.flow_state != 0,
                source=f'{self._candidate_source_prefix}:flow_obs',
            )
            self._update_flow_health(decoded, stamp_ms)
            self._publish_json(self._flow_candidate_pub, state.to_dict())
            self._publish_frame_info(
                '0x09 flow_obs '
                f'vx={decoded.flow_vx:.2f} vy={decoded.flow_vy:.2f} '
                f'state={decoded.flow_state} quality={decoded.flow_quality} alt_cm={decoded.alt_cm}'
            )
            self._maybe_log_flow(state)
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
                self._publish_frame_info('0x34 invalid distance')
            else:
                self._publish_frame_info(
                    f'0x34 dir={int(decoded.direction)} angle={int(decoded.angle_deg)} '
                    f'dist={decoded.distance_m:.2f}m'
                )

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

    def _publish_json(self, publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        publisher.publish(msg)

    def _stamp_to_ms(self, stamp) -> int:
        return int(stamp.sec) * 1000 + int(stamp.nanosec) // 1_000_000

    def _update_imu_health(self, decoded: ImuRawData, stamp_ms: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._imu_first_rx_ns == 0:
            self._imu_first_rx_ns = now_ns
        self._last_imu_rx_ns = now_ns
        self._imu_frame_count += 1

        sample = (
            decoded.acc_x,
            decoded.acc_y,
            decoded.acc_z,
            decoded.gyr_x,
            decoded.gyr_y,
            decoded.gyr_z,
        )
        if sample == self._last_imu_sample:
            self._imu_same_sample_count += 1
        else:
            self._imu_same_sample_count = 0
        self._last_imu_sample = sample

    def _update_flow_health(self, decoded: FlowObsData, stamp_ms: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._flow_first_rx_ns == 0:
            self._flow_first_rx_ns = now_ns
        self._last_flow_rx_ns = now_ns
        self._flow_frame_count += 1

        sample = (
            decoded.flow_vx,
            decoded.flow_vy,
            decoded.flow_state,
            decoded.flow_quality,
            decoded.alt_cm,
        )
        self._flow_large_jump = False
        if self._last_flow_sample is not None:
            prev_vx, prev_vy, _, _, _ = self._last_flow_sample
            delta_v = math.hypot(decoded.flow_vx - prev_vx, decoded.flow_vy - prev_vy)
            self._flow_large_jump = delta_v > self._flow_jump_threshold_mps
        self._last_flow_sample = sample

    def _publish_health(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        now_ms = now_ns // 1_000_000
        self._publish_imu_health(now_ns, now_ms)
        self._publish_flow_health(now_ns, now_ms)

    def _publish_imu_health(self, now_ns: int, now_ms: int) -> None:
        age_sec = math.inf if self._last_imu_rx_ns == 0 else (now_ns - self._last_imu_rx_ns) / 1e9
        rate_hz = self._estimate_rate_hz(self._imu_frame_count, self._imu_first_rx_ns, now_ns)
        sample = self._last_imu_sample
        imu_all_zero = bool(sample is not None and all(value == 0 for value in sample))
        imu_stale = self._imu_same_sample_count >= 20
        state = ImuHealthState(
            stamp_ms=int(now_ms),
            imu_stream_ok=age_sec <= self._stream_timeout_sec,
            imu_rate_hz=rate_hz,
            imu_all_zero=imu_all_zero,
            imu_stale=imu_stale,
            imu_same_sample_count=self._imu_same_sample_count,
            acc_unit_hint='raw_s16_from_ano_0x01',
            gyr_unit_hint='raw_s16_from_ano_0x01',
            decode_error_count=self._decode_error_count,
        )
        self._publish_json(self._imu_health_pub, state.to_dict())

    def _publish_flow_health(self, now_ns: int, now_ms: int) -> None:
        age_sec = math.inf if self._last_flow_rx_ns == 0 else (now_ns - self._last_flow_rx_ns) / 1e9
        rate_hz = self._estimate_rate_hz(self._flow_frame_count, self._flow_first_rx_ns, now_ns)
        sample = self._last_flow_sample
        state = FlowHealthState(
            stamp_ms=int(now_ms),
            flow_stream_ok=age_sec <= self._stream_timeout_sec,
            flow_rate_hz=rate_hz,
            flow_state_recent=int(sample[2]) if sample is not None else 0,
            flow_quality_recent=int(sample[3]) if sample is not None else 0,
            flow_alt_recent=int(sample[4]) if sample is not None else 0,
            flow_large_jump=self._flow_large_jump,
            flow_stale=sample is None or age_sec > self._stream_timeout_sec,
            decode_error_count=self._decode_error_count,
        )
        self._publish_json(self._flow_health_pub, state.to_dict())

    def _estimate_rate_hz(self, frame_count: int, first_rx_ns: int, now_ns: int) -> float:
        if frame_count < 2 or first_rx_ns == 0:
            return 0.0
        elapsed_sec = max((now_ns - first_rx_ns) / 1e9, 1e-6)
        return frame_count / elapsed_sec

    def _maybe_log_imu(self, state: ImuCandidateState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_imu_log_ns < 2_000_000_000:
            return
        self._last_imu_log_ns = now_ns
        self.get_logger().info(
            'IMU_RAW refreshing '
            f'stamp_ms={state.stamp_ms} '
            f'acc=({state.acc_x},{state.acc_y},{state.acc_z}) '
            f'gyr=({state.gyr_x},{state.gyr_y},{state.gyr_z}) valid={state.valid}'
        )

    def _maybe_log_flow(self, state: FlowCandidateState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_flow_log_ns < 2_000_000_000:
            return
        self._last_flow_log_ns = now_ns
        self.get_logger().info(
            'FLOW_OBS refreshing '
            f'stamp_ms={state.stamp_ms} '
            f'vx={state.flow_vx:.2f}mps vy={state.flow_vy:.2f}mps '
            f'state={state.flow_state} quality={state.flow_quality} alt_cm={state.alt_cm} '
            f'valid={state.valid}'
        )

    def _handle_cmd_vel_body(self, msg: Twist) -> None:
        command = RealtimeControlCommand(
            ctrl_yawdps=clamp_int16(math.degrees(msg.angular.z)),
            ctrl_spd_x=clamp_int16(msg.linear.x * 100.0),
            ctrl_spd_y=clamp_int16(msg.linear.y * 100.0),
            ctrl_spd_z=clamp_int16(msg.linear.z * 100.0),
        )
        self._last_command = command
        self._last_command_time = self.get_clock().now()
        self.get_logger().info(
            'Received cmd_vel_body '
            f'vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} vz={msg.linear.z:.2f} yaw={msg.angular.z:.2f}'
        )
        self._send_command(command)

    def _resend_command(self) -> None:
        command_to_send = self._last_command

        if self._last_command_time is None:
            command_to_send = RealtimeControlCommand()
        else:
            elapsed_sec = (
                self.get_clock().now().nanoseconds - self._last_command_time.nanoseconds
            ) / 1e9
            if elapsed_sec > self._cmd_timeout_sec:
                command_to_send = RealtimeControlCommand()

        self._send_command(command_to_send)

    def _send_command(self, command: RealtimeControlCommand) -> None:
        if self._serial is None:
            return

        frame = encode_realtime_control_frame(command)
        try:
            self._serial.write(frame)
            if self._send_debug_enable:
                self.get_logger().debug(
                    f'Sent 0x{REALTIME_CONTROL_FRAME_ID:02X} '
                    f'rol={command.ctrl_rol} pit={command.ctrl_pit} thr={command.ctrl_thr} '
                    f'yaw_dps={command.ctrl_yawdps} '
                    f'spd=({command.ctrl_spd_x},{command.ctrl_spd_y},{command.ctrl_spd_z})'
                )
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self._close_serial()

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
