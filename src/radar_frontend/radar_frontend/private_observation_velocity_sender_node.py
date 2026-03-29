import json
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

from ano_receiver_bridge.observation_state import ExternalObservationState
from ano_receiver_bridge.private_protocol import encode_state_packet, packet_to_hex


class PrivateObservationVelocitySenderNode(Node):
    def __init__(self) -> None:
        super().__init__('private_observation_velocity_sender_node')

        self.declare_parameter('candidate_topic', '/observation/radar_candidate')
        self.declare_parameter('status_topic', '/observation/radar_status')
        self.declare_parameter('send_hz', 10.0)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('live_send', False)
        self.declare_parameter('enable_send', False)
        self.declare_parameter('send_invalid_frames', True)
        self.declare_parameter('min_confidence_for_live_send', 0.85)
        self.declare_parameter('max_speed_cms', 80)
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('output_mode', 'serial')
        self.declare_parameter('output_file', '/tmp/private_observation_velocity_frames.bin')
        self.declare_parameter('log_hex_enable', True)

        self._candidate_topic = str(self.get_parameter('candidate_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._send_hz = max(1.0, float(self.get_parameter('send_hz').value))
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._live_send = bool(self.get_parameter('live_send').value)
        self._enable_send = bool(self.get_parameter('enable_send').value)
        self._send_invalid_frames = bool(self.get_parameter('send_invalid_frames').value)
        self._min_confidence_for_live_send = float(
            self.get_parameter('min_confidence_for_live_send').value
        )
        self._max_speed_cms = int(self.get_parameter('max_speed_cms').value)
        self._swap_xy = bool(self.get_parameter('swap_xy').value)
        self._invert_x = bool(self.get_parameter('invert_x').value)
        self._invert_y = bool(self.get_parameter('invert_y').value)
        self._port = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._timeout = float(self.get_parameter('timeout').value)
        self._output_mode = str(self.get_parameter('output_mode').value).strip().lower()
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._log_hex_enable = bool(self.get_parameter('log_hex_enable').value)

        self._latest_candidate: Optional[Dict[str, object]] = None
        self._latest_status: Optional[Dict[str, object]] = None
        self._sequence = 0
        self._last_log_ns = 0
        self._serial: Optional[serial.Serial] = None
        self._file_handle = None
        self._last_open_attempt_ns = 0

        self._debug_pub = self.create_publisher(
            String, '/private_observation/velocity_sender_debug', 10
        )
        self._status_pub = self.create_publisher(
            String, '/private_observation/velocity_sender_status', 10
        )

        self.create_subscription(String, self._candidate_topic, self._handle_candidate, 10)
        self.create_subscription(String, self._status_topic, self._handle_status, 10)

        if self._output_mode == 'file' and not self._dry_run and self._enable_live_send():
            self._output_file.parent.mkdir(parents=True, exist_ok=True)
            self._file_handle = self._output_file.open('ab')
        elif self._output_mode == 'serial' and not self._dry_run and self._enable_live_send():
            self._open_serial()

        self.create_timer(1.0 / self._send_hz, self._send_once)
        self.get_logger().info(
            'Private observation velocity sender started '
            f'dry_run={self._dry_run} live_send={self._live_send} enable_send={self._enable_send} '
            f'output_mode={self._output_mode}'
        )

    def _handle_candidate(self, msg: String) -> None:
        try:
            self._latest_candidate = json.loads(msg.data)
        except json.JSONDecodeError:
            self._latest_candidate = None

    def _handle_status(self, msg: String) -> None:
        try:
            self._latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._latest_status = None

    def _enable_live_send(self) -> bool:
        return self._live_send and self._enable_send and not self._dry_run

    def _apply_axis_corrections(self, vx_cms: int, vy_cms: int) -> Tuple[int, int]:
        corrected_x = int(vx_cms)
        corrected_y = int(vy_cms)
        if self._swap_xy:
            corrected_x, corrected_y = corrected_y, corrected_x
        if self._invert_x:
            corrected_x = -corrected_x
        if self._invert_y:
            corrected_y = -corrected_y
        return corrected_x, corrected_y

    def _build_state_and_debug(self) -> Tuple[Optional[ExternalObservationState], Dict[str, object]]:
        now_ms = self.get_clock().now().nanoseconds // 1_000_000
        debug_payload: Dict[str, object] = {
            'timestamp_ms': int(now_ms),
            'dry_run': self._dry_run,
            'live_send': self._live_send,
            'enable_send': self._enable_send,
            'send_invalid_frames': self._send_invalid_frames,
            'swap_xy': self._swap_xy,
            'invert_x': self._invert_x,
            'invert_y': self._invert_y,
        }

        if self._latest_candidate is None or self._latest_status is None:
            debug_payload.update(
                {
                    'allow_send': False,
                    'reason': 'missing_candidate_or_status',
                    'downstream_status': 'NO_INPUT',
                }
            )
            if not self._send_invalid_frames:
                return None, debug_payload
            return self._build_invalid_state(now_ms), debug_payload

        vel_valid = bool(self._latest_candidate.get('vel_valid', False))
        confidence = float(self._latest_candidate.get('confidence', 0.0))
        status = str(self._latest_candidate.get('status', 'INVALID'))
        reject_reason = str(self._latest_candidate.get('reject_reason', ''))
        downstream_recommendation = str(
            self._latest_status.get('downstream_recommendation', 'BLOCK')
        )
        raw_vx_cms = int(self._latest_candidate.get('vel_x_cms', 0))
        raw_vy_cms = int(self._latest_candidate.get('vel_y_cms', 0))
        corrected_vx_cms, corrected_vy_cms = self._apply_axis_corrections(raw_vx_cms, raw_vy_cms)
        speed_norm_cms = int(round((corrected_vx_cms ** 2 + corrected_vy_cms ** 2) ** 0.5))

        allow_send = (
            vel_valid
            and confidence >= self._min_confidence_for_live_send
            and status == 'VALID'
            and downstream_recommendation == 'ALLOW_FOR_NEXT_STAGE_REVIEW'
            and speed_norm_cms <= self._max_speed_cms
            and reject_reason == ''
        )

        debug_payload.update(
            {
                'vel_valid': vel_valid,
                'confidence': round(confidence, 4),
                'candidate_status': status,
                'reject_reason': reject_reason,
                'downstream_recommendation': downstream_recommendation,
                'raw_vel_x_cms': raw_vx_cms,
                'raw_vel_y_cms': raw_vy_cms,
                'corrected_vel_x_cms': corrected_vx_cms,
                'corrected_vel_y_cms': corrected_vy_cms,
                'speed_norm_cms': speed_norm_cms,
                'allow_send': allow_send,
            }
        )

        if allow_send:
            return (
                ExternalObservationState(
                    timestamp_ms=int(self._latest_candidate.get('timestamp_ms', now_ms)),
                    pos_x_cm=0,
                    pos_y_cm=0,
                    pos_z_cm=0,
                    vel_x_cms=corrected_vx_cms,
                    vel_y_cms=corrected_vy_cms,
                    vel_z_cms=0,
                    dist_direction=0,
                    dist_angle_deg=0,
                    dist_cm=0,
                    pos_valid=False,
                    vel_valid=True,
                    dist_valid=False,
                    source_name='radar_velocity_sender',
                    frame_id=str(self._latest_candidate.get('frame_id', 'radar_frame_unverified')),
                    debug_info='velocity_only_live_candidate',
                ),
                debug_payload,
            )

        debug_payload['reason'] = self._derive_block_reason(
            vel_valid=vel_valid,
            confidence=confidence,
            status=status,
            reject_reason=reject_reason,
            downstream_recommendation=downstream_recommendation,
            speed_norm_cms=speed_norm_cms,
        )
        if not self._send_invalid_frames:
            return None, debug_payload
        return self._build_invalid_state(now_ms), debug_payload

    def _derive_block_reason(
        self,
        *,
        vel_valid: bool,
        confidence: float,
        status: str,
        reject_reason: str,
        downstream_recommendation: str,
        speed_norm_cms: int,
    ) -> str:
        if not vel_valid:
            return 'vel_valid_false'
        if confidence < self._min_confidence_for_live_send:
            return 'confidence_below_live_threshold'
        if status != 'VALID':
            return f'status_{status.lower()}'
        if downstream_recommendation != 'ALLOW_FOR_NEXT_STAGE_REVIEW':
            return f'downstream_{downstream_recommendation.lower()}'
        if speed_norm_cms > self._max_speed_cms:
            return 'speed_over_limit'
        if reject_reason:
            return reject_reason
        return 'blocked_unknown'

    def _build_invalid_state(self, timestamp_ms: int) -> ExternalObservationState:
        return ExternalObservationState(
            timestamp_ms=timestamp_ms,
            pos_x_cm=0,
            pos_y_cm=0,
            pos_z_cm=0,
            vel_x_cms=0,
            vel_y_cms=0,
            vel_z_cms=0,
            dist_direction=0,
            dist_angle_deg=0,
            dist_cm=0,
            pos_valid=False,
            vel_valid=False,
            dist_valid=False,
            source_name='radar_velocity_sender',
            frame_id='radar_frame_unverified',
            debug_info='all_invalid_velocity_sender',
        )

    def _send_once(self) -> None:
        state, debug_payload = self._build_state_and_debug()
        if state is None:
            self._publish_debug(debug_payload)
            self._publish_status('SKIP', debug_payload)
            return

        packet = encode_state_packet(state, sequence=self._sequence)
        debug_payload.update(
            {
                'sequence': self._sequence,
                'hex': packet_to_hex(packet) if self._log_hex_enable else '',
                'vel_valid_out': state.vel_valid,
            }
        )

        if self._enable_live_send():
            if self._output_mode == 'serial':
                if self._serial is None:
                    self._retry_open_if_needed()
                    self._publish_status('WAIT_SERIAL', debug_payload)
                    self._publish_debug(debug_payload)
                    return
                try:
                    self._serial.write(packet)
                except serial.SerialException as exc:
                    debug_payload['serial_error'] = str(exc)
                    self._close_serial()
                    self._publish_status('SERIAL_ERROR', debug_payload)
                    self._publish_debug(debug_payload)
                    return
            elif self._output_mode == 'file':
                if self._file_handle is None:
                    self._output_file.parent.mkdir(parents=True, exist_ok=True)
                    self._file_handle = self._output_file.open('ab')
                self._file_handle.write(packet)
                self._file_handle.flush()
            elif self._output_mode != 'stdout':
                debug_payload['reason'] = f'unsupported_output_mode_{self._output_mode}'
                self._publish_status('ERROR', debug_payload)
                self._publish_debug(debug_payload)
                return

            self._publish_status('LIVE_SENT', debug_payload)
        else:
            self._publish_status('DRY_RUN', debug_payload)

        self._publish_debug(debug_payload)
        self._sequence = (self._sequence + 1) & 0xFF

    def _publish_debug(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._debug_pub.publish(msg)
        self._maybe_log(payload)

    def _publish_status(self, status: str, payload: Dict[str, object]) -> None:
        status_payload = {
            'status': status,
            'dry_run': self._dry_run,
            'live_send': self._live_send,
            'enable_send': self._enable_send,
            'allow_send': payload.get('allow_send', False),
            'reason': payload.get('reason', ''),
            'sequence': payload.get('sequence', self._sequence),
        }
        msg = String()
        msg.data = json.dumps(status_payload, sort_keys=True, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _maybe_log(self, payload: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 1_000_000_000:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            'velocity_sender '
            f"allow_send={payload.get('allow_send')} "
            f"dry_run={self._dry_run} live_send={self._live_send} enable_send={self._enable_send} "
            f"vx={payload.get('corrected_vel_x_cms', 0)} vy={payload.get('corrected_vel_y_cms', 0)} "
            f"confidence={payload.get('confidence', 0.0)} reason={payload.get('reason', '')}"
        )

    def _open_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self.get_logger().info(
                f'Opened {self._port} at {self._baudrate} baud for private observation velocity send'
            )
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(f'Failed to open serial port {self._port}: {exc}')

    def _retry_open_if_needed(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_open_attempt_ns < 1_000_000_000:
            return
        self._last_open_attempt_ns = now_ns
        self._open_serial()

    def _close_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def destroy_node(self) -> bool:
        if self._file_handle is not None:
            self._file_handle.close()
            self._file_handle = None
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PrivateObservationVelocitySenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
