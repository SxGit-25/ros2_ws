from pathlib import Path
from typing import Optional

import rclpy
import serial
from rclpy.node import Node

from ano_receiver_bridge.observation_state import ExternalObservationState
from ano_receiver_bridge.private_protocol import (
    encode_state_packet,
    packet_to_hex,
    state_valid_flags,
)


class ObservationSenderNode(Node):
    """Periodic UART3 private-frame sender for fixed-value link bring-up."""

    def __init__(self) -> None:
        super().__init__('rpi_observation_sender')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('send_hz', 10.0)
        self.declare_parameter('profile', 'all_valid')
        self.declare_parameter('log_hex_enable', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('output_mode', 'serial')
        self.declare_parameter('output_file', '/tmp/private_observation_frames.bin')
        self.declare_parameter('pos_x_cm', 100)
        self.declare_parameter('pos_y_cm', 0)
        self.declare_parameter('pos_z_cm', 120)
        self.declare_parameter('vel_x_cms', 0)
        self.declare_parameter('vel_y_cms', 0)
        self.declare_parameter('vel_z_cms', 0)
        self.declare_parameter('dist_direction', 1)
        self.declare_parameter('dist_angle_deg', 270)
        self.declare_parameter('dist_cm', 120)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._timeout = self.get_parameter('timeout').value
        self._send_hz = float(self.get_parameter('send_hz').value)
        self._profile = str(self.get_parameter('profile').value)
        self._log_hex_enable = bool(self.get_parameter('log_hex_enable').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._output_mode = str(self.get_parameter('output_mode').value).strip().lower()
        self._output_file = Path(str(self.get_parameter('output_file').value))

        self._pos_x_cm = int(self.get_parameter('pos_x_cm').value)
        self._pos_y_cm = int(self.get_parameter('pos_y_cm').value)
        self._pos_z_cm = int(self.get_parameter('pos_z_cm').value)
        self._vel_x_cms = int(self.get_parameter('vel_x_cms').value)
        self._vel_y_cms = int(self.get_parameter('vel_y_cms').value)
        self._vel_z_cms = int(self.get_parameter('vel_z_cms').value)
        self._dist_direction = int(self.get_parameter('dist_direction').value)
        self._dist_angle_deg = int(self.get_parameter('dist_angle_deg').value)
        self._dist_cm = int(self.get_parameter('dist_cm').value)

        self._serial: Optional[serial.Serial] = None
        self._file_handle = None
        self._last_open_attempt_ns = 0
        self._sequence = 0
        self._last_log_ns = 0

        if self._output_mode == 'file':
            self._output_file.parent.mkdir(parents=True, exist_ok=True)
            self._file_handle = self._output_file.open('ab')
            self.get_logger().info(f'Writing private observation frames to {self._output_file}')
        elif self._output_mode == 'serial' and not self._dry_run:
            self._open_serial()
        else:
            self.get_logger().info(
                f'Observation sender running without serial: dry_run={self._dry_run} output_mode={self._output_mode}'
            )
        self.create_timer(max(0.02, 1.0 / max(self._send_hz, 0.1)), self._send_once)

    def _open_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self.get_logger().info(
                f'Opened {self._port} at {self._baudrate} baud for private observation frames'
            )
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(f'Failed to open serial port {self._port}: {exc}')

    def _build_state(self) -> ExternalObservationState:
        timestamp_ms = self.get_clock().now().nanoseconds // 1_000_000
        return ExternalObservationState.build_profile(
            profile=self._profile,
            timestamp_ms=timestamp_ms,
            pos_x_cm=self._pos_x_cm,
            pos_y_cm=self._pos_y_cm,
            pos_z_cm=self._pos_z_cm,
            vel_x_cms=self._vel_x_cms,
            vel_y_cms=self._vel_y_cms,
            vel_z_cms=self._vel_z_cms,
            dist_direction=self._dist_direction,
            dist_angle_deg=self._dist_angle_deg,
            dist_cm=self._dist_cm,
        )

    def _send_once(self) -> None:
        state = self._build_state()
        packet = encode_state_packet(state, sequence=self._sequence)

        if self._dry_run:
            if self._output_mode == 'file':
                if self._file_handle is None:
                    self.get_logger().error(f'Output file is not open: {self._output_file}')
                    return
                self._file_handle.write(packet)
                self._file_handle.flush()
        elif self._output_mode == 'serial':
            if self._serial is None:
                self._retry_open_if_needed()
                return
            try:
                self._serial.write(packet)
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial write failed: {exc}')
                self._close_serial()
                return
        elif self._output_mode == 'file':
            if self._file_handle is None:
                self.get_logger().error(f'Output file is not open: {self._output_file}')
                return
            self._file_handle.write(packet)
            self._file_handle.flush()
        elif self._output_mode != 'stdout':
            self.get_logger().error(
                f"Unsupported output_mode '{self._output_mode}', expected serial/stdout/file"
            )
            return

        self._maybe_log(packet, state)
        self._sequence = (self._sequence + 1) & 0xFF

    def _maybe_log(self, packet: bytes, state: ExternalObservationState) -> None:
        if not self._log_hex_enable:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 1_000_000_000:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            'TX private obs '
            f'profile={self._profile} seq={self._sequence} valid=0x{state_valid_flags(state):02X} '
            f'pos=({state.pos_x_cm},{state.pos_y_cm},{state.pos_z_cm})cm pos_valid={state.pos_valid} '
            f'vel=({state.vel_x_cms},{state.vel_y_cms},{state.vel_z_cms})cm/s vel_valid={state.vel_valid} '
            f'dist=(dir={state.dist_direction},angle={state.dist_angle_deg},cm={state.dist_cm}) '
            f'dist_valid={state.dist_valid} mode={self._output_mode} dry_run={self._dry_run} '
            f'hex={packet_to_hex(packet)}'
        )

    def _retry_open_if_needed(self) -> None:
        if self._output_mode != 'serial' or self._dry_run:
            return
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
        if self._file_handle is not None:
            self._file_handle.close()
            self._file_handle = None
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObservationSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
