import json
import math
import statistics
from collections import deque
from typing import Deque, Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


STATUS_VALID = 'VALID'
STATUS_LOW_CONFIDENCE = 'LOW_CONFIDENCE'
STATUS_INVALID = 'INVALID'

STATE_STATIC_LIKELY = 'STATIC_LIKELY'
STATE_MOTION_LIKELY = 'MOTION_LIKELY'
STATE_JITTERY_UNCERTAIN = 'JITTERY_UNCERTAIN'


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def round_or_none(value: Optional[float], digits: int = 4) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def median_or_zero(values: Deque[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.median(values))


def mean_or_zero(values: Deque[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.fmean(values))


def std_or_zero(values: Deque[float]) -> float:
    if len(values) < 2:
        return 0.0
    return float(statistics.pstdev(values))


def max_abs_or_zero(values: Deque[float]) -> float:
    if not values:
        return 0.0
    return max(abs(float(value)) for value in values)


class Rf2oRadarBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('rf2o_radar_bridge_node')

        self.declare_parameter('rf2o_odom_topic', '/odom_rf2o')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_candidate_topic', '/radar/odom_candidate')
        self.declare_parameter('vel_candidate_topic', '/radar/vel_candidate')
        self.declare_parameter('match_quality_topic', '/radar/match_quality')
        self.declare_parameter('odom_status_topic', '/radar/odom_status')
        self.declare_parameter('match_debug_topic', '/radar/match_debug')
        self.declare_parameter('min_dt_sec', 0.04)
        self.declare_parameter('max_dt_sec', 0.20)
        self.declare_parameter('odom_stale_timeout_sec', 0.50)
        self.declare_parameter('scan_stale_timeout_sec', 0.50)
        self.declare_parameter('max_speed_mps', 1.20)
        self.declare_parameter('warn_speed_mps', 0.85)
        self.declare_parameter('max_yaw_rate_rps', 1.20)
        self.declare_parameter('warn_yaw_rate_rps', 0.85)
        self.declare_parameter('max_pose_jump_m', 0.25)
        self.declare_parameter('max_yaw_jump_deg', 20.0)
        self.declare_parameter('static_speed_threshold_mps', 0.05)
        self.declare_parameter('static_yaw_rate_threshold_rps', 0.08)
        self.declare_parameter('baseline_window_size', 100)
        self.declare_parameter('jitter_window_size', 20)
        self.declare_parameter('velocity_window_size', 5)
        self.declare_parameter('static_false_motion_trigger_frames', 8)
        self.declare_parameter('static_false_motion_clear_frames', 5)
        self.declare_parameter('static_jitter_speed_std_threshold', 0.04)
        self.declare_parameter('moving_jitter_speed_std_threshold', 0.12)
        self.declare_parameter('static_jitter_yaw_std_threshold', 0.08)
        self.declare_parameter('moving_jitter_yaw_std_threshold', 0.20)
        self.declare_parameter('quality_low_confidence_threshold', 0.45)
        self.declare_parameter('quality_valid_threshold', 0.62)
        self.declare_parameter('log_hz', 1.0)
        self.declare_parameter('baseline_log_hz', 0.5)

        self._rf2o_odom_topic = str(self.get_parameter('rf2o_odom_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._odom_candidate_topic = str(self.get_parameter('odom_candidate_topic').value)
        self._vel_candidate_topic = str(self.get_parameter('vel_candidate_topic').value)
        self._match_quality_topic = str(self.get_parameter('match_quality_topic').value)
        self._odom_status_topic = str(self.get_parameter('odom_status_topic').value)
        self._match_debug_topic = str(self.get_parameter('match_debug_topic').value)
        self._min_dt_sec = float(self.get_parameter('min_dt_sec').value)
        self._max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self._odom_stale_timeout_sec = float(self.get_parameter('odom_stale_timeout_sec').value)
        self._scan_stale_timeout_sec = float(self.get_parameter('scan_stale_timeout_sec').value)
        self._max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._warn_speed_mps = float(self.get_parameter('warn_speed_mps').value)
        self._max_yaw_rate_rps = float(self.get_parameter('max_yaw_rate_rps').value)
        self._warn_yaw_rate_rps = float(self.get_parameter('warn_yaw_rate_rps').value)
        self._max_pose_jump_m = float(self.get_parameter('max_pose_jump_m').value)
        self._max_yaw_jump_deg = float(self.get_parameter('max_yaw_jump_deg').value)
        self._static_speed_threshold_mps = float(
            self.get_parameter('static_speed_threshold_mps').value
        )
        self._static_yaw_rate_threshold_rps = float(
            self.get_parameter('static_yaw_rate_threshold_rps').value
        )
        self._baseline_window_size = max(10, int(self.get_parameter('baseline_window_size').value))
        self._jitter_window_size = max(5, int(self.get_parameter('jitter_window_size').value))
        self._velocity_window_size = max(3, int(self.get_parameter('velocity_window_size').value))
        self._static_false_motion_trigger_frames = max(
            2, int(self.get_parameter('static_false_motion_trigger_frames').value)
        )
        self._static_false_motion_clear_frames = max(
            2, int(self.get_parameter('static_false_motion_clear_frames').value)
        )
        self._static_jitter_speed_std_threshold = float(
            self.get_parameter('static_jitter_speed_std_threshold').value
        )
        self._moving_jitter_speed_std_threshold = float(
            self.get_parameter('moving_jitter_speed_std_threshold').value
        )
        self._static_jitter_yaw_std_threshold = float(
            self.get_parameter('static_jitter_yaw_std_threshold').value
        )
        self._moving_jitter_yaw_std_threshold = float(
            self.get_parameter('moving_jitter_yaw_std_threshold').value
        )
        self._quality_low_confidence_threshold = float(
            self.get_parameter('quality_low_confidence_threshold').value
        )
        self._quality_valid_threshold = float(
            self.get_parameter('quality_valid_threshold').value
        )
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._baseline_log_hz = max(0.1, float(self.get_parameter('baseline_log_hz').value))

        self._previous_odom: Optional[Odometry] = None
        self._last_odom_received_ns: Optional[int] = None
        self._last_scan_received_ns: Optional[int] = None
        self._last_log_ns = 0
        self._last_baseline_log_ns = 0

        self._vx_filter_window: Deque[float] = deque(maxlen=self._velocity_window_size)
        self._vy_filter_window: Deque[float] = deque(maxlen=self._velocity_window_size)
        self._yaw_filter_window: Deque[float] = deque(maxlen=self._velocity_window_size)

        self._vx_jitter_window: Deque[float] = deque(maxlen=self._jitter_window_size)
        self._vy_jitter_window: Deque[float] = deque(maxlen=self._jitter_window_size)
        self._yaw_jitter_window: Deque[float] = deque(maxlen=self._jitter_window_size)

        self._vx_baseline_window: Deque[float] = deque(maxlen=self._baseline_window_size)
        self._vy_baseline_window: Deque[float] = deque(maxlen=self._baseline_window_size)
        self._yaw_baseline_window: Deque[float] = deque(maxlen=self._baseline_window_size)

        self._static_false_motion_over_counter = 0
        self._static_false_motion_clear_counter = 0
        self._static_false_motion_active = False

        self._odom_pub = self.create_publisher(Odometry, self._odom_candidate_topic, 10)
        self._vel_pub = self.create_publisher(TwistStamped, self._vel_candidate_topic, 10)
        self._quality_pub = self.create_publisher(Float32, self._match_quality_topic, 10)
        self._status_pub = self.create_publisher(String, self._odom_status_topic, 10)
        self._debug_pub = self.create_publisher(String, self._match_debug_topic, 10)

        self.create_subscription(Odometry, self._rf2o_odom_topic, self._handle_odom, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, 10)

        self.get_logger().info(
            'RF2O radar bridge started '
            f'rf2o_odom_topic={self._rf2o_odom_topic} scan_topic={self._scan_topic}'
        )

    def _handle_scan(self, msg: LaserScan) -> None:
        self._last_scan_received_ns = self.get_clock().now().nanoseconds

    def _handle_odom(self, msg: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._last_odom_received_ns = now_ns

        if self._previous_odom is None:
            self._publish_initial_waiting_status(msg, now_ns)
            self._previous_odom = msg
            return

        result = self._build_bridge_result(self._previous_odom, msg, now_ns)
        self._publish_outputs(msg, result)
        self._previous_odom = msg

    def _publish_initial_waiting_status(self, msg: Odometry, now_ns: int) -> None:
        payload = {
            'status': STATUS_INVALID,
            'accept_motion': False,
            'quality': 0.0,
            'vx_mps': 0.0,
            'vy_mps': 0.0,
            'yaw_rate_rps': 0.0,
            'reasons': ['waiting_for_previous_odom'],
            'warnings': [],
            'stamp_ns': self._stamp_to_ns(msg),
        }
        self._publish_status(payload)
        self._publish_quality(0.0)
        self._publish_debug(
            {
                'stamp_ns': self._stamp_to_ns(msg),
                'dt_sec': None,
                'dx_world': None,
                'dy_world': None,
                'dyaw_rad': None,
                'vx_world': None,
                'vy_world': None,
                'raw_vx': None,
                'raw_vy': None,
                'raw_yaw_rate': None,
                'filt_vx': 0.0,
                'filt_vy': 0.0,
                'filt_yaw_rate': 0.0,
                'vx_std': 0.0,
                'vy_std': 0.0,
                'yaw_std': 0.0,
                'internal_motion_state': STATE_JITTERY_UNCERTAIN,
                'scan_fresh': self._is_scan_fresh(now_ns),
                'odom_fresh': True,
                'quality_components': {},
                'baseline_summary': self._build_baseline_summary(),
                'status': STATUS_INVALID,
                'reasons': ['waiting_for_previous_odom'],
                'warnings': [],
            }
        )

    def _build_bridge_result(
        self,
        previous_msg: Odometry,
        current_msg: Odometry,
        now_ns: int,
    ) -> Dict[str, object]:
        prev_stamp_ns = self._stamp_to_ns(previous_msg)
        curr_stamp_ns = self._stamp_to_ns(current_msg)
        dt_sec = (curr_stamp_ns - prev_stamp_ns) / 1e9

        prev_x = float(previous_msg.pose.pose.position.x)
        prev_y = float(previous_msg.pose.pose.position.y)
        curr_x = float(current_msg.pose.pose.position.x)
        curr_y = float(current_msg.pose.pose.position.y)

        prev_yaw = quaternion_to_yaw(
            previous_msg.pose.pose.orientation.x,
            previous_msg.pose.pose.orientation.y,
            previous_msg.pose.pose.orientation.z,
            previous_msg.pose.pose.orientation.w,
        )
        curr_yaw = quaternion_to_yaw(
            current_msg.pose.pose.orientation.x,
            current_msg.pose.pose.orientation.y,
            current_msg.pose.pose.orientation.z,
            current_msg.pose.pose.orientation.w,
        )

        dx_world = curr_x - prev_x
        dy_world = curr_y - prev_y
        dyaw_rad = normalize_angle(curr_yaw - prev_yaw)

        vx_world = dx_world / dt_sec if dt_sec > 0.0 else 0.0
        vy_world = dy_world / dt_sec if dt_sec > 0.0 else 0.0

        cos_yaw = math.cos(curr_yaw)
        sin_yaw = math.sin(curr_yaw)
        vx_body_raw = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body_raw = -sin_yaw * vx_world + cos_yaw * vy_world
        yaw_rate_raw = dyaw_rad / dt_sec if dt_sec > 0.0 else 0.0

        self._vx_filter_window.append(vx_body_raw)
        self._vy_filter_window.append(vy_body_raw)
        self._yaw_filter_window.append(yaw_rate_raw)

        vx_body_filtered = median_or_zero(self._vx_filter_window)
        vy_body_filtered = median_or_zero(self._vy_filter_window)
        yaw_rate_filtered = median_or_zero(self._yaw_filter_window)

        self._vx_jitter_window.append(vx_body_filtered)
        self._vy_jitter_window.append(vy_body_filtered)
        self._yaw_jitter_window.append(yaw_rate_filtered)

        self._vx_baseline_window.append(vx_body_filtered)
        self._vy_baseline_window.append(vy_body_filtered)
        self._yaw_baseline_window.append(yaw_rate_filtered)

        translation_norm_m = math.hypot(dx_world, dy_world)
        speed_norm_mps = math.hypot(vx_body_filtered, vy_body_filtered)
        yaw_jump_deg = abs(math.degrees(dyaw_rad))

        odom_fresh = self._is_odom_fresh(now_ns)
        scan_fresh = self._is_scan_fresh(now_ns)
        baseline_summary = self._build_baseline_summary()

        internal_motion_state = self._classify_motion_state(
            speed_norm_mps=speed_norm_mps,
            yaw_rate_rps=abs(yaw_rate_filtered),
            vx_std=std_or_zero(self._vx_jitter_window),
            vy_std=std_or_zero(self._vy_jitter_window),
            yaw_std=std_or_zero(self._yaw_jitter_window),
        )

        reasons: List[str] = []
        warnings: List[str] = []

        if dt_sec <= 0.0:
            reasons.append('non_positive_dt')
        if dt_sec < self._min_dt_sec:
            reasons.append('dt_too_small')
        if dt_sec > self._max_dt_sec:
            reasons.append('dt_too_large')
        if not odom_fresh:
            reasons.append('odom_stale')
        if not scan_fresh:
            reasons.append('scan_stale')
        if translation_norm_m > self._max_pose_jump_m:
            reasons.append('pose_jump_too_large')
        if yaw_jump_deg > self._max_yaw_jump_deg:
            reasons.append('yaw_jump_too_large')
        if speed_norm_mps > self._max_speed_mps:
            reasons.append('speed_too_large')
        if abs(yaw_rate_filtered) > self._max_yaw_rate_rps:
            reasons.append('yaw_rate_too_large')

        if self._warn_speed_mps < speed_norm_mps <= self._max_speed_mps:
            warnings.append('speed_near_limit')
        if self._warn_yaw_rate_rps < abs(yaw_rate_filtered) <= self._max_yaw_rate_rps:
            warnings.append('yaw_rate_near_limit')

        static_false_motion_active = self._update_static_false_motion_state(
            baseline_summary=baseline_summary,
            speed_norm_mps=speed_norm_mps,
            yaw_rate_rps=abs(yaw_rate_filtered),
            internal_motion_state=internal_motion_state,
        )
        if static_false_motion_active:
            warnings.append('static_false_motion_suspected')

        jitter_warning = self._is_jitter_warning(internal_motion_state)
        if jitter_warning:
            warnings.append('velocity_jitter_high')

        quality_components = {
            'freshness': self._compute_freshness_score(
                odom_fresh=odom_fresh,
                scan_fresh=scan_fresh,
            ),
            'dt': self._compute_dt_score(dt_sec),
            'motion_consistency': self._compute_motion_consistency_score(
                internal_motion_state=internal_motion_state,
                speed_norm_mps=speed_norm_mps,
                yaw_rate_rps=abs(yaw_rate_filtered),
                vx_std=std_or_zero(self._vx_jitter_window),
                vy_std=std_or_zero(self._vy_jitter_window),
                yaw_std=std_or_zero(self._yaw_jitter_window),
            ),
            'static_consistency': self._compute_static_consistency_score(
                internal_motion_state=internal_motion_state,
                static_false_motion_active=static_false_motion_active,
            ),
            'limit': self._compute_limit_score(
                translation_norm_m=translation_norm_m,
                yaw_jump_deg=yaw_jump_deg,
                speed_norm_mps=speed_norm_mps,
                yaw_rate_rps=abs(yaw_rate_filtered),
            ),
        }
        quality = round(
            0.28 * quality_components['freshness']
            + 0.14 * quality_components['dt']
            + 0.24 * quality_components['motion_consistency']
            + 0.16 * quality_components['static_consistency']
            + 0.18 * quality_components['limit'],
            4,
        )

        if reasons:
            status = STATUS_INVALID
        elif internal_motion_state == STATE_JITTERY_UNCERTAIN or warnings or quality < self._quality_valid_threshold:
            if quality < self._quality_low_confidence_threshold:
                if 'quality_too_low' not in warnings:
                    warnings.append('quality_too_low')
            elif quality < self._quality_valid_threshold:
                if 'quality_low_confidence' not in warnings:
                    warnings.append('quality_low_confidence')
            status = STATUS_LOW_CONFIDENCE
        else:
            status = STATUS_VALID

        debug_payload = {
            'stamp_ns': curr_stamp_ns,
            'dt_sec': round_or_none(dt_sec, 6),
            'dx_world': round_or_none(dx_world),
            'dy_world': round_or_none(dy_world),
            'dyaw_rad': round_or_none(dyaw_rad, 6),
            'vx_world': round_or_none(vx_world),
            'vy_world': round_or_none(vy_world),
            'raw_vx': round_or_none(vx_body_raw),
            'raw_vy': round_or_none(vy_body_raw),
            'raw_yaw_rate': round_or_none(yaw_rate_raw),
            'filt_vx': round_or_none(vx_body_filtered),
            'filt_vy': round_or_none(vy_body_filtered),
            'filt_yaw_rate': round_or_none(yaw_rate_filtered),
            'vx_std': round_or_none(std_or_zero(self._vx_jitter_window)),
            'vy_std': round_or_none(std_or_zero(self._vy_jitter_window)),
            'yaw_std': round_or_none(std_or_zero(self._yaw_jitter_window)),
            'translation_norm_m': round_or_none(translation_norm_m),
            'speed_norm_mps': round_or_none(speed_norm_mps),
            'yaw_jump_deg': round_or_none(yaw_jump_deg),
            'internal_motion_state': internal_motion_state,
            'scan_fresh': scan_fresh,
            'odom_fresh': odom_fresh,
            'quality_components': quality_components,
            'baseline_summary': baseline_summary,
            'status': status,
            'reasons': reasons,
            'warnings': warnings,
        }

        return {
            'stamp_ns': curr_stamp_ns,
            'vx_body_filtered': vx_body_filtered,
            'vy_body_filtered': vy_body_filtered,
            'yaw_rate_filtered': yaw_rate_filtered,
            'quality': quality,
            'status': status,
            'accept_motion': status == STATUS_VALID,
            'reasons': reasons,
            'warnings': warnings,
            'debug': debug_payload,
            'baseline_summary': baseline_summary,
        }

    def _classify_motion_state(
        self,
        *,
        speed_norm_mps: float,
        yaw_rate_rps: float,
        vx_std: float,
        vy_std: float,
        yaw_std: float,
    ) -> str:
        near_static = (
            speed_norm_mps <= self._static_speed_threshold_mps
            and yaw_rate_rps <= self._static_yaw_rate_threshold_rps
        )
        static_jitter_high = (
            vx_std > self._static_jitter_speed_std_threshold
            or vy_std > self._static_jitter_speed_std_threshold
            or yaw_std > self._static_jitter_yaw_std_threshold
        )
        moving_jitter_high = (
            vx_std > self._moving_jitter_speed_std_threshold
            or vy_std > self._moving_jitter_speed_std_threshold
            or yaw_std > self._moving_jitter_yaw_std_threshold
        )

        if near_static and not static_jitter_high:
            return STATE_STATIC_LIKELY
        if not near_static and not moving_jitter_high:
            return STATE_MOTION_LIKELY
        return STATE_JITTERY_UNCERTAIN

    def _update_static_false_motion_state(
        self,
        *,
        baseline_summary: Dict[str, object],
        speed_norm_mps: float,
        yaw_rate_rps: float,
        internal_motion_state: str,
    ) -> bool:
        baseline_speed_max = max(
            float(baseline_summary['vx']['max_abs']),
            float(baseline_summary['vy']['max_abs']),
        )
        baseline_yaw_max = float(baseline_summary['yaw_rate']['max_abs'])

        speed_trigger = max(self._static_speed_threshold_mps * 1.8, baseline_speed_max * 2.5, 0.08)
        yaw_trigger = max(
            self._static_yaw_rate_threshold_rps * 1.8,
            baseline_yaw_max * 2.5,
            0.12,
        )

        false_motion_now = (
            internal_motion_state != STATE_STATIC_LIKELY
            and (speed_norm_mps > speed_trigger or yaw_rate_rps > yaw_trigger)
        )

        if false_motion_now:
            self._static_false_motion_over_counter += 1
            self._static_false_motion_clear_counter = 0
        else:
            self._static_false_motion_clear_counter += 1
            self._static_false_motion_over_counter = 0

        if (
            not self._static_false_motion_active
            and self._static_false_motion_over_counter >= self._static_false_motion_trigger_frames
        ):
            self._static_false_motion_active = True
        elif (
            self._static_false_motion_active
            and self._static_false_motion_clear_counter >= self._static_false_motion_clear_frames
        ):
            self._static_false_motion_active = False

        return self._static_false_motion_active

    def _is_jitter_warning(self, internal_motion_state: str) -> bool:
        vx_std = std_or_zero(self._vx_jitter_window)
        vy_std = std_or_zero(self._vy_jitter_window)
        yaw_std = std_or_zero(self._yaw_jitter_window)

        if internal_motion_state == STATE_STATIC_LIKELY:
            return (
                vx_std > self._static_jitter_speed_std_threshold
                or vy_std > self._static_jitter_speed_std_threshold
                or yaw_std > self._static_jitter_yaw_std_threshold
            )
        if internal_motion_state == STATE_MOTION_LIKELY:
            return (
                vx_std > self._moving_jitter_speed_std_threshold
                or vy_std > self._moving_jitter_speed_std_threshold
                or yaw_std > self._moving_jitter_yaw_std_threshold
            )
        return True

    def _compute_freshness_score(self, *, odom_fresh: bool, scan_fresh: bool) -> float:
        if odom_fresh and scan_fresh:
            return 1.0
        if odom_fresh or scan_fresh:
            return 0.55
        return 0.05

    def _compute_dt_score(self, dt_sec: float) -> float:
        if dt_sec <= 0.0:
            return 0.0
        if dt_sec < self._min_dt_sec or dt_sec > self._max_dt_sec:
            return 0.15
        dt_center = 0.5 * (self._min_dt_sec + self._max_dt_sec)
        dt_span = max(0.5 * (self._max_dt_sec - self._min_dt_sec), 1e-3)
        normalized = abs(dt_sec - dt_center) / dt_span
        return round(clamp(1.0 - 0.25 * normalized, 0.60, 1.0), 4)

    def _compute_motion_consistency_score(
        self,
        *,
        internal_motion_state: str,
        speed_norm_mps: float,
        yaw_rate_rps: float,
        vx_std: float,
        vy_std: float,
        yaw_std: float,
    ) -> float:
        if internal_motion_state == STATE_STATIC_LIKELY:
            speed_term = clamp(
                1.0 - speed_norm_mps / max(self._static_speed_threshold_mps * 2.0, 1e-3),
                0.70,
                1.0,
            )
            yaw_term = clamp(
                1.0 - yaw_rate_rps / max(self._static_yaw_rate_threshold_rps * 2.0, 1e-3),
                0.70,
                1.0,
            )
            jitter_term = clamp(
                1.0
                - max(vx_std, vy_std) / max(self._static_jitter_speed_std_threshold * 2.5, 1e-3)
                - yaw_std / max(self._static_jitter_yaw_std_threshold * 4.0, 1e-3),
                0.55,
                1.0,
            )
            return round((0.35 * speed_term) + (0.25 * yaw_term) + (0.40 * jitter_term), 4)

        if internal_motion_state == STATE_MOTION_LIKELY:
            speed_term = clamp(1.0 - speed_norm_mps / max(self._max_speed_mps * 1.2, 1e-3), 0.60, 1.0)
            yaw_term = clamp(
                1.0 - yaw_rate_rps / max(self._max_yaw_rate_rps * 1.2, 1e-3),
                0.60,
                1.0,
            )
            jitter_term = clamp(
                1.0
                - max(vx_std, vy_std) / max(self._moving_jitter_speed_std_threshold * 2.5, 1e-3)
                - yaw_std / max(self._moving_jitter_yaw_std_threshold * 2.5, 1e-3),
                0.55,
                1.0,
            )
            return round((0.35 * speed_term) + (0.25 * yaw_term) + (0.40 * jitter_term), 4)

        uncertainty_term = clamp(
            1.0
            - max(vx_std, vy_std) / max(self._moving_jitter_speed_std_threshold * 2.0, 1e-3)
            - yaw_std / max(self._moving_jitter_yaw_std_threshold * 2.0, 1e-3),
            0.20,
            0.65,
        )
        return round(uncertainty_term, 4)

    def _compute_static_consistency_score(
        self,
        *,
        internal_motion_state: str,
        static_false_motion_active: bool,
    ) -> float:
        if static_false_motion_active:
            return 0.35
        if internal_motion_state == STATE_STATIC_LIKELY:
            return 1.0
        if internal_motion_state == STATE_MOTION_LIKELY:
            return 0.92
        return 0.58

    def _compute_limit_score(
        self,
        *,
        translation_norm_m: float,
        yaw_jump_deg: float,
        speed_norm_mps: float,
        yaw_rate_rps: float,
    ) -> float:
        translation_score = clamp(
            1.0 - translation_norm_m / max(self._max_pose_jump_m * 1.2, 1e-3),
            0.10,
            1.0,
        )
        yaw_jump_score = clamp(
            1.0 - yaw_jump_deg / max(self._max_yaw_jump_deg * 1.2, 1e-3),
            0.10,
            1.0,
        )
        speed_score = clamp(1.0 - speed_norm_mps / max(self._max_speed_mps * 1.1, 1e-3), 0.10, 1.0)
        yaw_rate_score = clamp(
            1.0 - yaw_rate_rps / max(self._max_yaw_rate_rps * 1.1, 1e-3),
            0.10,
            1.0,
        )
        return round(
            0.25 * translation_score
            + 0.25 * yaw_jump_score
            + 0.25 * speed_score
            + 0.25 * yaw_rate_score,
            4,
        )

    def _build_baseline_summary(self) -> Dict[str, object]:
        return {
            'vx': self._window_summary(self._vx_baseline_window),
            'vy': self._window_summary(self._vy_baseline_window),
            'yaw_rate': self._window_summary(self._yaw_baseline_window),
        }

    def _window_summary(self, values: Deque[float]) -> Dict[str, object]:
        return {
            'count': len(values),
            'mean': round_or_none(mean_or_zero(values)),
            'median': round_or_none(median_or_zero(values)),
            'std': round_or_none(std_or_zero(values)),
            'max_abs': round_or_none(max_abs_or_zero(values)),
        }

    def _publish_outputs(self, current_msg: Odometry, result: Dict[str, object]) -> None:
        odom_candidate = Odometry()
        odom_candidate.header = current_msg.header
        odom_candidate.child_frame_id = current_msg.child_frame_id
        odom_candidate.pose = current_msg.pose
        odom_candidate.twist.twist.linear.x = float(result['vx_body_filtered'])
        odom_candidate.twist.twist.linear.y = float(result['vy_body_filtered'])
        odom_candidate.twist.twist.angular.z = float(result['yaw_rate_filtered'])
        self._odom_pub.publish(odom_candidate)

        vel_candidate = TwistStamped()
        vel_candidate.header = current_msg.header
        vel_candidate.twist.linear.x = float(result['vx_body_filtered'])
        vel_candidate.twist.linear.y = float(result['vy_body_filtered'])
        vel_candidate.twist.linear.z = 0.0
        vel_candidate.twist.angular.x = 0.0
        vel_candidate.twist.angular.y = 0.0
        vel_candidate.twist.angular.z = float(result['yaw_rate_filtered'])
        self._vel_pub.publish(vel_candidate)

        status_payload = {
            'status': result['status'],
            'stamp_ns': result['stamp_ns'],
            'quality': round_or_none(result['quality']),
            'accept_motion': result['accept_motion'],
            'vx_mps': round_or_none(result['vx_body_filtered']),
            'vy_mps': round_or_none(result['vy_body_filtered']),
            'yaw_rate_rps': round_or_none(result['yaw_rate_filtered']),
            'reasons': result['reasons'],
            'warnings': result['warnings'],
        }
        self._publish_status(status_payload)
        self._publish_quality(float(result['quality']))
        self._publish_debug(result['debug'])
        self._maybe_log(status_payload)
        self._maybe_log_baseline(result['baseline_summary'])

    def _publish_quality(self, quality: float) -> None:
        msg = Float32()
        msg.data = float(quality)
        self._quality_pub.publish(msg)

    def _publish_status(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _publish_debug(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._debug_pub.publish(msg)

    def _stamp_to_ns(self, msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _is_scan_fresh(self, now_ns: int) -> bool:
        if self._last_scan_received_ns is None:
            return False
        return (now_ns - self._last_scan_received_ns) / 1e9 <= self._scan_stale_timeout_sec

    def _is_odom_fresh(self, now_ns: int) -> bool:
        if self._last_odom_received_ns is None:
            return False
        return (now_ns - self._last_odom_received_ns) / 1e9 <= self._odom_stale_timeout_sec

    def _maybe_log(self, status_payload: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._log_hz)
        if now_ns - self._last_log_ns < min_gap_ns:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            'rf2o_bridge '
            f"status={status_payload['status']} "
            f"quality={status_payload['quality']} "
            f"vx={status_payload['vx_mps']} "
            f"vy={status_payload['vy_mps']} "
            f"yaw_rate={status_payload['yaw_rate_rps']} "
            f"reasons={status_payload['reasons']} "
            f"warnings={status_payload['warnings']}"
        )

    def _maybe_log_baseline(self, baseline_summary: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._baseline_log_hz)
        if now_ns - self._last_baseline_log_ns < min_gap_ns:
            return
        self._last_baseline_log_ns = now_ns
        self.get_logger().info(
            'rf2o_bridge_baseline '
            f"vx={baseline_summary['vx']} "
            f"vy={baseline_summary['vy']} "
            f"yaw_rate={baseline_summary['yaw_rate']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rf2oRadarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
