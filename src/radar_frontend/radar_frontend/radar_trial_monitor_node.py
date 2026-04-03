import json
import math
import sys
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


COLOR_RED = '\033[31m'
COLOR_YELLOW = '\033[33m'
COLOR_GREEN = '\033[32m'
COLOR_CYAN = '\033[36m'
COLOR_RESET = '\033[0m'


def _round_or_none(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _json_loads_or_none(payload: str) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    return data


def _is_truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _fmt_float(value: Optional[float], digits: int = 3, suffix: str = '') -> str:
    if value is None:
        return 'n/a'
    return f'{float(value):.{digits}f}{suffix}'


def _fmt_signed(value: Optional[float], digits: int = 3, suffix: str = '') -> str:
    if value is None:
        return 'n/a'
    return f'{float(value):+.{digits}f}{suffix}'


def _fmt_bool(value: Any) -> str:
    return 'Y' if _is_truthy(value) else 'N'


def _fmt_text_list(value: Any) -> str:
    if isinstance(value, list):
        items = [str(item) for item in value if str(item)]
        return ', '.join(items) if items else '-'
    if value in (None, ''):
        return '-'
    return str(value)


class RadarTrialMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('radar_trial_monitor_node')

        self.declare_parameter('radar_status_topic', '/radar/odom_status')
        self.declare_parameter('observation_status_topic', '/observation/radar_status')
        self.declare_parameter(
            'sender_status_topic', '/private_observation/velocity_sender_status'
        )
        self.declare_parameter(
            'sender_debug_topic', '/private_observation/velocity_sender_debug'
        )
        self.declare_parameter('radar_match_debug_topic', '/radar/match_debug')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('summary_topic', '/radar/trial_monitor_summary')
        self.declare_parameter('publish_summary', True)
        self.declare_parameter('print_console', True)
        self.declare_parameter('clear_console', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('stale_timeout_sec', 1.5)
        self.declare_parameter('false_speed_alert_cms', 12)
        self.declare_parameter('live_block_abort_streak', 20)

        self._radar_status_topic = str(self.get_parameter('radar_status_topic').value)
        self._observation_status_topic = str(
            self.get_parameter('observation_status_topic').value
        )
        self._sender_status_topic = str(self.get_parameter('sender_status_topic').value)
        self._sender_debug_topic = str(self.get_parameter('sender_debug_topic').value)
        self._radar_match_debug_topic = str(
            self.get_parameter('radar_match_debug_topic').value
        )
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._summary_topic = str(self.get_parameter('summary_topic').value)
        self._publish_summary_enabled = bool(self.get_parameter('publish_summary').value)
        self._print_console = bool(self.get_parameter('print_console').value)
        self._clear_console = bool(self.get_parameter('clear_console').value)
        self._enable_color = bool(self.get_parameter('enable_color').value)
        self._refresh_hz = max(0.5, float(self.get_parameter('refresh_hz').value))
        self._stale_timeout_sec = max(0.5, float(self.get_parameter('stale_timeout_sec').value))
        self._false_speed_alert_cms = int(self.get_parameter('false_speed_alert_cms').value)
        self._live_block_abort_streak = max(
            1, int(self.get_parameter('live_block_abort_streak').value)
        )

        self._radar_status: Optional[Dict[str, Any]] = None
        self._observation_status: Optional[Dict[str, Any]] = None
        self._sender_status: Optional[Dict[str, Any]] = None
        self._sender_debug: Optional[Dict[str, Any]] = None
        self._match_debug: Optional[Dict[str, Any]] = None

        self._last_scan_stamp_ns: Optional[int] = None
        self._scan_freq_samples_hz: Deque[float] = deque(maxlen=12)
        self._last_scan_valid_ratio: Optional[float] = None
        self._topic_update_ns: Dict[str, int] = {}
        self._sender_block_streak = 0
        self._last_rendered_text = ''

        if self._publish_summary_enabled:
            self._summary_pub = self.create_publisher(String, self._summary_topic, 10)
        else:
            self._summary_pub = None

        self.create_subscription(
            String, self._radar_status_topic, self._handle_radar_status, 10
        )
        self.create_subscription(
            String,
            self._observation_status_topic,
            self._handle_observation_status,
            10,
        )
        self.create_subscription(
            String, self._sender_status_topic, self._handle_sender_status, 10
        )
        self.create_subscription(
            String, self._sender_debug_topic, self._handle_sender_debug, 10
        )
        self.create_subscription(
            String, self._radar_match_debug_topic, self._handle_match_debug, 10
        )
        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, 10)
        self.create_timer(1.0 / self._refresh_hz, self._refresh)

        self.get_logger().info(
            'Radar trial monitor started '
            f'radar_status_topic={self._radar_status_topic} '
            f'observation_status_topic={self._observation_status_topic} '
            f'sender_status_topic={self._sender_status_topic}'
        )

    def _mark_topic_seen(self, key: str) -> None:
        self._topic_update_ns[key] = self.get_clock().now().nanoseconds

    def _handle_radar_status(self, msg: String) -> None:
        self._radar_status = _json_loads_or_none(msg.data)
        self._mark_topic_seen('radar_status')

    def _handle_observation_status(self, msg: String) -> None:
        self._observation_status = _json_loads_or_none(msg.data)
        self._mark_topic_seen('observation_status')

    def _handle_sender_status(self, msg: String) -> None:
        self._sender_status = _json_loads_or_none(msg.data)
        self._mark_topic_seen('sender_status')
        allow_send = _is_truthy(
            self._sender_status.get('allow_send', False) if self._sender_status else False
        )
        if allow_send:
            self._sender_block_streak = 0
        else:
            self._sender_block_streak += 1

    def _handle_sender_debug(self, msg: String) -> None:
        self._sender_debug = _json_loads_or_none(msg.data)
        self._mark_topic_seen('sender_debug')

    def _handle_match_debug(self, msg: String) -> None:
        self._match_debug = _json_loads_or_none(msg.data)
        self._mark_topic_seen('match_debug')

    def _handle_scan(self, scan: LaserScan) -> None:
        stamp_ns = (
            int(scan.header.stamp.sec) * 1_000_000_000
            + int(scan.header.stamp.nanosec)
        )
        if self._last_scan_stamp_ns is not None:
            delta_ns = stamp_ns - self._last_scan_stamp_ns
            if delta_ns > 0:
                self._scan_freq_samples_hz.append(1e9 / float(delta_ns))
        self._last_scan_stamp_ns = stamp_ns
        total_points = len(scan.ranges)
        valid_points = 0
        for raw_range in scan.ranges:
            if not math.isfinite(raw_range):
                continue
            if raw_range < scan.range_min or raw_range > scan.range_max:
                continue
            valid_points += 1
        self._last_scan_valid_ratio = (
            float(valid_points) / float(total_points) if total_points > 0 else None
        )
        self._mark_topic_seen('scan')

    def _topic_age_sec(self, key: str) -> Optional[float]:
        last_ns = self._topic_update_ns.get(key)
        if last_ns is None:
            return None
        return (self.get_clock().now().nanoseconds - last_ns) / 1e9

    def _is_stale(self, key: str) -> bool:
        age_sec = self._topic_age_sec(key)
        return age_sec is None or age_sec > self._stale_timeout_sec

    def _refresh(self) -> None:
        summary = self._build_summary()
        self._publish_summary(summary)
        self._render_console(summary)

    def _build_summary(self) -> Dict[str, Any]:
        radar = self._radar_status or {}
        observation = self._observation_status or {}
        sender_status = self._sender_status or {}
        sender_debug = self._sender_debug or {}
        match_debug = self._match_debug or {}

        sender_live_mode = (
            _is_truthy(sender_status.get('live_send', False))
            and _is_truthy(sender_status.get('enable_send', False))
            and not _is_truthy(sender_status.get('dry_run', True))
        )
        sender_mode = 'LIVE_SEND' if sender_live_mode else 'SAFE_DRY_RUN'
        sender_phase = str(sender_status.get('status', 'NO_INPUT'))
        sender_allow_send = _is_truthy(sender_status.get('allow_send', False))
        sender_live_active = sender_live_mode and sender_phase == 'LIVE_SENT'
        vel_valid_out = _is_truthy(sender_debug.get('vel_valid_out', False))
        accept_motion = _is_truthy(radar.get('accept_motion', False))
        radar_status = str(radar.get('status', 'NO_INPUT'))
        observation_vel_valid = _is_truthy(observation.get('vel_valid', False))
        downstream_recommendation = str(
            observation.get('downstream_recommendation', 'NO_INPUT')
        )
        speed_norm_cms = sender_debug.get('speed_norm_cms')
        speed_norm_cms = int(speed_norm_cms) if speed_norm_cms is not None else None

        alerts: List[str] = []
        severe_abort = False

        stale_topics = [
            key
            for key in ['radar_status', 'observation_status', 'sender_status', 'sender_debug']
            if self._is_stale(key)
        ]
        if stale_topics:
            alerts.append(f'stale_topics={",".join(stale_topics)}')
            if sender_live_mode:
                severe_abort = True

        if sender_phase in {'SERIAL_ERROR', 'ERROR'}:
            alerts.append(f'sender_fault={sender_phase}')
            severe_abort = True

        if sender_live_mode and not sender_live_active:
            alerts.append(f'live_send_not_active={sender_phase}')

        if sender_live_mode and not sender_allow_send and self._sender_block_streak >= self._live_block_abort_streak:
            alerts.append(f'continuous_sender_block={self._sender_block_streak}')
            severe_abort = True

        if speed_norm_cms is not None and not accept_motion and speed_norm_cms > self._false_speed_alert_cms:
            alerts.append(f'false_velocity_suspected={speed_norm_cms}cms')
            if sender_allow_send or sender_live_active:
                severe_abort = True

        if sender_allow_send and not accept_motion:
            alerts.append('sender_allow_send_while_frontend_rejects_motion')
            severe_abort = True

        if sender_live_active and (not sender_allow_send or not vel_valid_out):
            alerts.append('live_send_state_inconsistent')
            severe_abort = True

        if radar_status != 'VALID':
            alerts.append(f'frontend_status={radar_status}')
        if not observation_vel_valid:
            reject_reason = str(observation.get('reject_reason', ''))
            if reject_reason:
                alerts.append(f'adapter_reject={reject_reason}')
        if downstream_recommendation not in {'ALLOW_FOR_NEXT_STAGE_REVIEW', 'NO_INPUT'}:
            alerts.append(f'downstream={downstream_recommendation}')

        recommendation = 'CONTINUE'
        if severe_abort:
            recommendation = 'ABORT'
        elif (
            stale_topics
            or radar_status != 'VALID'
            or not observation_vel_valid
            or not sender_allow_send
            or (sender_live_mode and not sender_live_active)
        ):
            recommendation = 'HOLD'

        stage = 'BOOTSTRAP'
        if not stale_topics:
            if radar_status != 'VALID':
                stage = 'CHECK_FRONTEND'
            elif not observation_vel_valid:
                stage = 'CHECK_ADAPTER'
            elif sender_mode == 'SAFE_DRY_RUN' and sender_allow_send:
                stage = 'SAFE_DRY_RUN_READY'
            elif sender_mode == 'LIVE_SEND' and sender_live_active:
                stage = 'LIVE_SEND_ACTIVE'
            elif sender_mode == 'LIVE_SEND':
                stage = 'LIVE_SEND_BLOCKED'
            else:
                stage = 'CHECK_SENDER'

        scan_valid_ratio = self._last_scan_valid_ratio

        estimated_scan_hz = None
        if self._scan_freq_samples_hz:
            estimated_scan_hz = sum(self._scan_freq_samples_hz) / len(self._scan_freq_samples_hz)

        return {
            'timestamp_ms': self.get_clock().now().nanoseconds // 1_000_000,
            'mode': sender_mode,
            'stage': stage,
            'recommendation': recommendation,
            'alerts': alerts,
            'sender_block_streak': self._sender_block_streak,
            'radar': {
                'status': radar_status,
                'accept_motion': accept_motion,
                'quality': _round_or_none(radar.get('quality')),
                'vx_mps': _round_or_none(radar.get('vx_mps')),
                'vy_mps': _round_or_none(radar.get('vy_mps')),
                'yaw_rate_rps': _round_or_none(radar.get('yaw_rate_rps')),
                'reasons': radar.get('reasons', []),
                'warnings': radar.get('warnings', []),
            },
            'observation': {
                'status': str(observation.get('status', 'NO_INPUT')),
                'vel_valid': observation_vel_valid,
                'confidence': _round_or_none(observation.get('confidence')),
                'reject_reason': str(observation.get('reject_reason', '')),
                'downstream_recommendation': downstream_recommendation,
            },
            'sender': {
                'status': sender_phase,
                'allow_send': sender_allow_send,
                'reason': str(sender_status.get('reason', '')),
                'sequence': sender_status.get('sequence'),
                'live_send': sender_live_mode,
                'dry_run': not sender_live_mode,
            },
            'sender_debug': {
                'corrected_vel_x_cms': sender_debug.get('corrected_vel_x_cms'),
                'corrected_vel_y_cms': sender_debug.get('corrected_vel_y_cms'),
                'speed_norm_cms': speed_norm_cms,
                'vel_valid_out': vel_valid_out,
            },
            'scan': {
                'valid_ratio': _round_or_none(scan_valid_ratio),
                'estimated_frequency_hz': _round_or_none(estimated_scan_hz),
            },
            'stale': {
                'radar_status_sec': _round_or_none(self._topic_age_sec('radar_status')),
                'observation_status_sec': _round_or_none(
                    self._topic_age_sec('observation_status')
                ),
                'sender_status_sec': _round_or_none(self._topic_age_sec('sender_status')),
                'sender_debug_sec': _round_or_none(self._topic_age_sec('sender_debug')),
            },
        }

    def _publish_summary(self, summary: Dict[str, Any]) -> None:
        if self._summary_pub is None:
            return
        msg = String()
        msg.data = json.dumps(summary, sort_keys=True, ensure_ascii=True)
        self._summary_pub.publish(msg)

    def _render_console(self, summary: Dict[str, Any]) -> None:
        if not self._print_console:
            return

        panel = self._format_panel(summary)
        if panel == self._last_rendered_text and not self._clear_console:
            return
        self._last_rendered_text = panel

        if self._clear_console and sys.stdout.isatty():
            sys.stdout.write('\033[2J\033[H')
        sys.stdout.write(panel + '\n')
        sys.stdout.flush()

    def _format_panel(self, summary: Dict[str, Any]) -> str:
        recommendation = str(summary['recommendation'])
        recommendation_text = self._colorize_recommendation(recommendation)
        radar = summary['radar']
        observation = summary['observation']
        sender = summary['sender']
        sender_debug = summary['sender_debug']
        scan = summary['scan']

        lines = [
            '=== RADAR FLIGHT TEST PANEL ===',
            f"Recommendation: {recommendation_text}    Stage: {summary['stage']}    Mode: {summary['mode']}",
            (
                'Radar: '
                f"status={radar['status']} motion={_fmt_bool(radar['accept_motion'])} "
                f"q={_fmt_float(radar['quality'])} "
                f"vx={_fmt_signed(radar['vx_mps'], suffix='m/s')} "
                f"vy={_fmt_signed(radar['vy_mps'], suffix='m/s')} "
                f"yaw={_fmt_signed(radar['yaw_rate_rps'], suffix='rad/s')}"
            ),
            (
                'Radar Notes: '
                f"reasons={_fmt_text_list(radar['reasons'])} "
                f"warnings={_fmt_text_list(radar['warnings'])}"
            ),
            (
                'Observation: '
                f"status={observation['status']} vel_valid={_fmt_bool(observation['vel_valid'])} "
                f"conf={_fmt_float(observation['confidence'])} "
                f"reject={observation['reject_reason'] or '-'} "
                f"downstream={observation['downstream_recommendation']}"
            ),
            (
                'Sender: '
                f"status={sender['status']} allow_send={_fmt_bool(sender['allow_send'])} "
                f"seq={sender['sequence'] if sender['sequence'] is not None else 'n/a'} "
                f"reason={sender['reason'] or '-'} "
                f"block_streak={summary['sender_block_streak']}"
            ),
            (
                'Sender Out: '
                f"vx={sender_debug['corrected_vel_x_cms'] if sender_debug['corrected_vel_x_cms'] is not None else 'n/a'}cms "
                f"vy={sender_debug['corrected_vel_y_cms'] if sender_debug['corrected_vel_y_cms'] is not None else 'n/a'}cms "
                f"speed={sender_debug['speed_norm_cms'] if sender_debug['speed_norm_cms'] is not None else 'n/a'}cms "
                f"vel_valid_out={_fmt_bool(sender_debug['vel_valid_out'])}"
            ),
            (
                'Scan: '
                f"valid_ratio={_fmt_float(scan['valid_ratio'])} "
                f"freq={_fmt_float(scan['estimated_frequency_hz'], suffix='Hz')}"
            ),
            f"Alerts: {_fmt_text_list(summary['alerts'])}",
        ]
        return '\n'.join(lines)

    def _colorize_recommendation(self, recommendation: str) -> str:
        if not self._enable_color:
            return recommendation
        if recommendation == 'CONTINUE':
            return f'{COLOR_GREEN}{recommendation}{COLOR_RESET}'
        if recommendation == 'HOLD':
            return f'{COLOR_YELLOW}{recommendation}{COLOR_RESET}'
        return f'{COLOR_RED}{recommendation}{COLOR_RESET}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarTrialMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
