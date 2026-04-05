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
        # Project default for the current N10 installation:
        # body convention is +x forward, +y left.
        # Field observation says the N10 zero-degree marker points roughly toward body -x
        # and is offset by about 5 deg relative to +y, which is most consistent with the
        # laser +x axis being at about +175 deg in base_link. The inverse fixed transform
        # from laser frame to base_link is therefore initialized to about -175 deg.
        self.declare_parameter('laser_to_base_yaw_deg', -175.0)
        self.declare_parameter('odom_candidate_topic', '/radar/odom_candidate')
        self.declare_parameter('vel_candidate_topic', '/radar/vel_candidate')
        self.declare_parameter('imu_candidate_topic', '/observation/imu_candidate_state')
        self.declare_parameter('enhanced_vel_candidate_topic', '/radar/vel_candidate_enhanced')
        self.declare_parameter('match_quality_topic', '/radar/match_quality')
        self.declare_parameter('match_quality_after_imu_topic', '/radar/match_quality_after_imu')
        self.declare_parameter('odom_status_topic', '/radar/odom_status')
        self.declare_parameter('match_debug_topic', '/radar/match_debug')
        self.declare_parameter('imu_enhanced_status_topic', '/radar/imu_enhanced_status')
        self.declare_parameter('imu_enhanced_debug_topic', '/radar/imu_enhanced_debug')
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
        self.declare_parameter('imu_stationary_enter_gyro_threshold', 120.0)
        self.declare_parameter('imu_stationary_exit_gyro_threshold', 180.0)
        self.declare_parameter('imu_stationary_enter_acc_delta_threshold', 180.0)
        self.declare_parameter('imu_stationary_exit_acc_delta_threshold', 260.0)
        self.declare_parameter('imu_stationary_hold_frames', 8)
        self.declare_parameter('imu_stationary_release_frames', 4)
        self.declare_parameter('imu_radar_consistency_speed_threshold', 0.08)
        self.declare_parameter('imu_radar_consistency_yaw_threshold', 0.12)
        self.declare_parameter('high_rotation_penalty_threshold', 220.0)
        self.declare_parameter('imu_acc_reference_window_size', 40)
        self.declare_parameter('imu_stale_timeout_sec', 0.5)
        self.declare_parameter('log_hz', 1.0)
        self.declare_parameter('baseline_log_hz', 0.5)

        self._rf2o_odom_topic = str(self.get_parameter('rf2o_odom_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._laser_to_base_yaw_deg = float(self.get_parameter('laser_to_base_yaw_deg').value)
        self._laser_to_base_yaw_rad = math.radians(self._laser_to_base_yaw_deg)
        self._odom_candidate_topic = str(self.get_parameter('odom_candidate_topic').value)
        self._vel_candidate_topic = str(self.get_parameter('vel_candidate_topic').value)
        self._imu_candidate_topic = str(self.get_parameter('imu_candidate_topic').value)
        self._enhanced_vel_candidate_topic = str(
            self.get_parameter('enhanced_vel_candidate_topic').value
        )
        self._match_quality_topic = str(self.get_parameter('match_quality_topic').value)
        self._match_quality_after_imu_topic = str(
            self.get_parameter('match_quality_after_imu_topic').value
        )
        self._odom_status_topic = str(self.get_parameter('odom_status_topic').value)
        self._match_debug_topic = str(self.get_parameter('match_debug_topic').value)
        self._imu_enhanced_status_topic = str(
            self.get_parameter('imu_enhanced_status_topic').value
        )
        self._imu_enhanced_debug_topic = str(
            self.get_parameter('imu_enhanced_debug_topic').value
        )
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
        self._imu_stationary_enter_gyro_threshold = float(
            self.get_parameter('imu_stationary_enter_gyro_threshold').value
        )
        self._imu_stationary_exit_gyro_threshold = float(
            self.get_parameter('imu_stationary_exit_gyro_threshold').value
        )
        self._imu_stationary_enter_acc_delta_threshold = float(
            self.get_parameter('imu_stationary_enter_acc_delta_threshold').value
        )
        self._imu_stationary_exit_acc_delta_threshold = float(
            self.get_parameter('imu_stationary_exit_acc_delta_threshold').value
        )
        self._imu_stationary_hold_frames = max(
            2, int(self.get_parameter('imu_stationary_hold_frames').value)
        )
        self._imu_stationary_release_frames = max(
            2, int(self.get_parameter('imu_stationary_release_frames').value)
        )
        self._imu_radar_consistency_speed_threshold = float(
            self.get_parameter('imu_radar_consistency_speed_threshold').value
        )
        self._imu_radar_consistency_yaw_threshold = float(
            self.get_parameter('imu_radar_consistency_yaw_threshold').value
        )
        self._high_rotation_penalty_threshold = float(
            self.get_parameter('high_rotation_penalty_threshold').value
        )
        self._imu_acc_reference_window_size = max(
            10, int(self.get_parameter('imu_acc_reference_window_size').value)
        )
        self._imu_stale_timeout_sec = max(0.1, float(self.get_parameter('imu_stale_timeout_sec').value))
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._baseline_log_hz = max(0.1, float(self.get_parameter('baseline_log_hz').value))

        self._previous_odom: Optional[Odometry] = None
        self._last_odom_received_ns: Optional[int] = None
        self._last_scan_received_ns: Optional[int] = None
        self._last_imu_received_ns: Optional[int] = None
        self._latest_imu_candidate: Optional[Dict[str, object]] = None
        self._last_log_ns = 0
        self._last_imu_enhancement_log_ns = 0
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
        self._imu_acc_norm_window: Deque[float] = deque(maxlen=self._imu_acc_reference_window_size)

        self._static_false_motion_over_counter = 0
        self._static_false_motion_clear_counter = 0
        self._static_false_motion_active = False
        self._imu_stationary_enter_counter = 0
        self._imu_stationary_release_counter = 0
        self._imu_stationary_flag = False
        self._imu_stationary_score = 0.0
        self._imu_stationary_reason = 'imu_not_ready'
        self._imu_acc_norm = 0.0
        self._imu_acc_delta = 0.0
        self._imu_raw_gyro_z = 0.0

        self._odom_pub = self.create_publisher(Odometry, self._odom_candidate_topic, 10)
        self._vel_pub = self.create_publisher(TwistStamped, self._vel_candidate_topic, 10)
        self._enhanced_vel_pub = self.create_publisher(
            TwistStamped, self._enhanced_vel_candidate_topic, 10
        )
        self._quality_pub = self.create_publisher(Float32, self._match_quality_topic, 10)
        self._quality_after_imu_pub = self.create_publisher(
            Float32, self._match_quality_after_imu_topic, 10
        )
        self._status_pub = self.create_publisher(String, self._odom_status_topic, 10)
        self._debug_pub = self.create_publisher(String, self._match_debug_topic, 10)
        self._imu_enhanced_status_pub = self.create_publisher(
            String, self._imu_enhanced_status_topic, 10
        )
        self._imu_enhanced_debug_pub = self.create_publisher(
            String, self._imu_enhanced_debug_topic, 10
        )

        self.create_subscription(Odometry, self._rf2o_odom_topic, self._handle_odom, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, 10)
        self.create_subscription(String, self._imu_candidate_topic, self._handle_imu_candidate, 10)

        self.get_logger().info(
            'RF2O radar bridge started '
            f'rf2o_odom_topic={self._rf2o_odom_topic} scan_topic={self._scan_topic} '
            f'laser_to_base_yaw_deg={self._laser_to_base_yaw_deg:.2f} '
            f'imu_candidate_topic={self._imu_candidate_topic}'
        )

    def _handle_scan(self, msg: LaserScan) -> None:
        self._last_scan_received_ns = self.get_clock().now().nanoseconds

    def _handle_imu_candidate(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._latest_imu_candidate = payload
        self._last_imu_received_ns = self.get_clock().now().nanoseconds
        if not bool(payload.get('valid', False)):
            self._imu_stationary_flag = False
            self._imu_stationary_score = 0.0
            self._imu_stationary_reason = 'imu_candidate_invalid'
            return
        self._update_imu_stationary_state(payload)

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
                'imu_enhancement': self._build_imu_debug_payload(
                    imu_available=False,
                    radar_confidence_before_imu=0.0,
                    radar_confidence_after_imu=0.0,
                    imu_radar_consistency_score=0.0,
                    consistency_reject_reason='waiting_for_previous_odom',
                    enhanced_vx=0.0,
                    enhanced_vy=0.0,
                    enhanced_yaw_rate=0.0,
                ),
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

        # rf2o pose is treated here as the laser/estimation frame pose in odom.
        # To recover velocity in aircraft/body axes, world velocity must be rotated into
        # base_link, not just into the laser frame. That requires the fixed yaw extrinsic
        # from laser frame to base_link.
        effective_body_yaw = normalize_angle(curr_yaw + self._laser_to_base_yaw_rad)
        cos_yaw = math.cos(effective_body_yaw)
        sin_yaw = math.sin(effective_body_yaw)
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
        imu_enhancement = self._apply_imu_enhancement(
            now_ns=now_ns,
            vx_body_filtered=vx_body_filtered,
            vy_body_filtered=vy_body_filtered,
            yaw_rate_filtered=yaw_rate_filtered,
            quality=quality,
            reasons=reasons,
            warnings=warnings,
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
            'yaw_estimation_frame_rad': round_or_none(curr_yaw, 6),
            'effective_body_yaw_rad': round_or_none(effective_body_yaw, 6),
            'laser_to_base_yaw_deg': round_or_none(self._laser_to_base_yaw_deg, 3),
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
            'imu_enhancement': self._build_imu_debug_payload(
                imu_available=bool(imu_enhancement['imu_available']),
                radar_confidence_before_imu=quality,
                radar_confidence_after_imu=float(imu_enhancement['radar_confidence_after_imu']),
                imu_radar_consistency_score=float(imu_enhancement['imu_radar_consistency_score']),
                consistency_reject_reason=str(imu_enhancement['consistency_reject_reason']),
                enhanced_vx=float(imu_enhancement['enhanced_vx']),
                enhanced_vy=float(imu_enhancement['enhanced_vy']),
                enhanced_yaw_rate=float(imu_enhancement['enhanced_yaw_rate']),
            ),
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
            'imu_enhancement': imu_enhancement,
        }

    def _update_imu_stationary_state(self, payload: Dict[str, object]) -> None:
        acc_x = float(payload.get('acc_x', 0.0))
        acc_y = float(payload.get('acc_y', 0.0))
        acc_z = float(payload.get('acc_z', 0.0))
        gyr_x = float(payload.get('gyr_x', 0.0))
        gyr_y = float(payload.get('gyr_y', 0.0))
        gyr_z = float(payload.get('gyr_z', 0.0))

        gyro_peak = max(abs(gyr_x), abs(gyr_y), abs(gyr_z))
        acc_norm = math.sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z)
        self._imu_acc_norm_window.append(acc_norm)
        acc_ref = median_or_zero(self._imu_acc_norm_window) if self._imu_acc_norm_window else acc_norm
        acc_delta = abs(acc_norm - acc_ref)

        enter_ok = (
            gyro_peak <= self._imu_stationary_enter_gyro_threshold
            and acc_delta <= self._imu_stationary_enter_acc_delta_threshold
        )
        exit_trigger = (
            gyro_peak >= self._imu_stationary_exit_gyro_threshold
            or acc_delta >= self._imu_stationary_exit_acc_delta_threshold
        )

        if self._imu_stationary_flag:
            if exit_trigger:
                self._imu_stationary_release_counter += 1
                self._imu_stationary_enter_counter = 0
            else:
                self._imu_stationary_release_counter = 0
            if self._imu_stationary_release_counter >= self._imu_stationary_release_frames:
                self._imu_stationary_flag = False
                self._imu_stationary_release_counter = 0
        else:
            if enter_ok:
                self._imu_stationary_enter_counter += 1
                self._imu_stationary_release_counter = 0
            else:
                self._imu_stationary_enter_counter = 0
            if self._imu_stationary_enter_counter >= self._imu_stationary_hold_frames:
                self._imu_stationary_flag = True
                self._imu_stationary_enter_counter = 0

        gyro_score = clamp(
            1.0 - gyro_peak / max(self._imu_stationary_exit_gyro_threshold, 1e-3),
            0.0,
            1.0,
        )
        acc_score = clamp(
            1.0 - acc_delta / max(self._imu_stationary_exit_acc_delta_threshold, 1e-3),
            0.0,
            1.0,
        )
        self._imu_stationary_score = round(0.65 * gyro_score + 0.35 * acc_score, 4)
        if self._imu_stationary_flag:
            self._imu_stationary_reason = 'gyro_low_and_acc_norm_stable'
        elif gyro_peak > self._imu_stationary_exit_gyro_threshold:
            self._imu_stationary_reason = 'gyro_above_exit_threshold'
        elif acc_delta > self._imu_stationary_exit_acc_delta_threshold:
            self._imu_stationary_reason = 'acc_norm_delta_above_exit_threshold'
        else:
            self._imu_stationary_reason = 'waiting_for_hold_frames'

        self._imu_acc_norm = acc_norm
        self._imu_acc_delta = acc_delta
        self._imu_raw_gyro_z = gyr_z

    def _apply_imu_enhancement(
        self,
        *,
        now_ns: int,
        vx_body_filtered: float,
        vy_body_filtered: float,
        yaw_rate_filtered: float,
        quality: float,
        reasons: List[str],
        warnings: List[str],
    ) -> Dict[str, object]:
        speed_norm = math.hypot(vx_body_filtered, vy_body_filtered)
        imu_available = self._is_imu_fresh(now_ns)
        radar_confidence_after_imu = float(quality)
        consistency_score = 1.0 if imu_available else 0.5
        consistency_reject_reason = ''
        enhanced_vx = float(vx_body_filtered)
        enhanced_vy = float(vy_body_filtered)
        enhanced_yaw_rate = float(yaw_rate_filtered)

        if not imu_available:
            return {
                'imu_available': False,
                'imu_stationary_flag': False,
                'imu_stationary_score': 0.0,
                'imu_stationary_reason': 'imu_stale_or_missing',
                'imu_radar_consistency_score': consistency_score,
                'consistency_reject_reason': 'imu_stale_or_missing',
                'radar_confidence_after_imu': radar_confidence_after_imu,
                'enhanced_vx': enhanced_vx,
                'enhanced_vy': enhanced_vy,
                'enhanced_yaw_rate': enhanced_yaw_rate,
                'rotation_penalty_applied': False,
            }

        if self._imu_stationary_flag:
            speed_ratio = speed_norm / max(self._imu_radar_consistency_speed_threshold, 1e-3)
            yaw_ratio = abs(yaw_rate_filtered) / max(self._imu_radar_consistency_yaw_threshold, 1e-3)
            mismatch_penalty = clamp(
                0.6 * clamp(speed_ratio / 2.0, 0.0, 1.0)
                + 0.4 * clamp(yaw_ratio / 2.0, 0.0, 1.0),
                0.0,
                1.0,
            )
            consistency_score = clamp(
                1.0 - mismatch_penalty * (0.55 + 0.35 * self._imu_stationary_score),
                0.05,
                1.0,
            )
            radar_confidence_after_imu *= consistency_score
            if (
                speed_norm <= self._imu_radar_consistency_speed_threshold * 1.8
                and abs(yaw_rate_filtered) <= self._imu_radar_consistency_yaw_threshold * 1.4
            ):
                enhanced_vx = 0.0
                enhanced_vy = 0.0
                enhanced_yaw_rate = 0.0
                consistency_reject_reason = 'imu_stationary_zeroed_small_radar_motion'
            elif mismatch_penalty > 0.55:
                scale = clamp(1.0 - 0.85 * self._imu_stationary_score, 0.05, 1.0)
                enhanced_vx *= scale
                enhanced_vy *= scale
                if abs(yaw_rate_filtered) <= self._imu_radar_consistency_yaw_threshold * 1.6:
                    enhanced_yaw_rate *= scale
                consistency_reject_reason = 'imu_stationary_radar_motion_mismatch'

        rotation_penalty_applied = False
        if abs(self._imu_raw_gyro_z) >= self._high_rotation_penalty_threshold:
            rotation_penalty_applied = True
            if speed_norm <= self._imu_radar_consistency_speed_threshold * 3.0:
                enhanced_vx *= 0.35
                enhanced_vy *= 0.35
                radar_confidence_after_imu *= 0.72
                consistency_score *= 0.82
                if not consistency_reject_reason:
                    consistency_reject_reason = 'high_rotation_translation_penalty'
            if (
                abs(yaw_rate_filtered) >= self._imu_radar_consistency_yaw_threshold
                and math.copysign(1.0, yaw_rate_filtered) != math.copysign(1.0, self._imu_raw_gyro_z)
            ):
                radar_confidence_after_imu *= 0.75
                consistency_score *= 0.7
                consistency_reject_reason = 'imu_radar_yaw_trend_mismatch'

        if 'pose_jump_too_large' in reasons or 'yaw_jump_too_large' in reasons:
            if self._imu_stationary_score >= 0.7:
                radar_confidence_after_imu *= 0.7
                consistency_score *= 0.75
                if not consistency_reject_reason:
                    consistency_reject_reason = 'jump_inconsistent_with_imu_stability'

        consistency_score = round(clamp(consistency_score, 0.0, 1.0), 4)
        radar_confidence_after_imu = round(clamp(radar_confidence_after_imu, 0.0, 1.0), 4)

        return {
            'imu_available': True,
            'imu_stationary_flag': self._imu_stationary_flag,
            'imu_stationary_score': self._imu_stationary_score,
            'imu_stationary_reason': self._imu_stationary_reason,
            'imu_radar_consistency_score': consistency_score,
            'consistency_reject_reason': consistency_reject_reason,
            'radar_confidence_after_imu': radar_confidence_after_imu,
            'enhanced_vx': round_or_none(enhanced_vx),
            'enhanced_vy': round_or_none(enhanced_vy),
            'enhanced_yaw_rate': round_or_none(enhanced_yaw_rate),
            'rotation_penalty_applied': rotation_penalty_applied,
        }

    def _build_imu_debug_payload(
        self,
        *,
        imu_available: bool,
        radar_confidence_before_imu: float,
        radar_confidence_after_imu: float,
        imu_radar_consistency_score: float,
        consistency_reject_reason: str,
        enhanced_vx: float,
        enhanced_vy: float,
        enhanced_yaw_rate: float,
    ) -> Dict[str, object]:
        return {
            'imu_available': imu_available,
            'imu_stationary_flag': self._imu_stationary_flag if imu_available else False,
            'imu_stationary_score': round_or_none(self._imu_stationary_score),
            'imu_stationary_reason': self._imu_stationary_reason if imu_available else 'imu_stale_or_missing',
            'raw_gyro_z': round_or_none(self._imu_raw_gyro_z),
            'raw_acc_norm': round_or_none(self._imu_acc_norm),
            'acc_norm_delta': round_or_none(self._imu_acc_delta),
            'radar_confidence_before_imu': round_or_none(radar_confidence_before_imu),
            'radar_confidence_after_imu': round_or_none(radar_confidence_after_imu),
            'imu_radar_consistency_score': round_or_none(imu_radar_consistency_score),
            'consistency_reject_reason': consistency_reject_reason,
            'radar_twist_candidate_enhanced': {
                'vx_mps': round_or_none(enhanced_vx),
                'vy_mps': round_or_none(enhanced_vy),
                'yaw_rate_rps': round_or_none(enhanced_yaw_rate),
            },
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
        imu_enhancement = result['imu_enhancement']

        enhanced_candidate = TwistStamped()
        enhanced_candidate.header = current_msg.header
        enhanced_candidate.twist.linear.x = float(imu_enhancement['enhanced_vx'])
        enhanced_candidate.twist.linear.y = float(imu_enhancement['enhanced_vy'])
        enhanced_candidate.twist.linear.z = 0.0
        enhanced_candidate.twist.angular.x = 0.0
        enhanced_candidate.twist.angular.y = 0.0
        enhanced_candidate.twist.angular.z = float(imu_enhancement['enhanced_yaw_rate'])
        self._enhanced_vel_pub.publish(enhanced_candidate)

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
        self._publish_quality_after_imu(float(imu_enhancement['radar_confidence_after_imu']))
        self._publish_imu_enhanced_status(
            {
                'stamp_ns': result['stamp_ns'],
                'imu_stationary_flag': imu_enhancement['imu_stationary_flag'],
                'imu_stationary_score': imu_enhancement['imu_stationary_score'],
                'imu_stationary_reason': imu_enhancement['imu_stationary_reason'],
                'imu_radar_consistency_score': imu_enhancement['imu_radar_consistency_score'],
                'consistency_reject_reason': imu_enhancement['consistency_reject_reason'],
                'radar_confidence_before_imu': result['quality'],
                'radar_confidence_after_imu': imu_enhancement['radar_confidence_after_imu'],
                'radar_twist_candidate_enhanced': {
                    'vx_mps': imu_enhancement['enhanced_vx'],
                    'vy_mps': imu_enhancement['enhanced_vy'],
                    'yaw_rate_rps': imu_enhancement['enhanced_yaw_rate'],
                },
            }
        )
        self._publish_imu_enhanced_debug(result['debug']['imu_enhancement'])
        self._publish_debug(result['debug'])
        self._maybe_log(status_payload)
        self._maybe_log_imu_enhancement(result['debug']['imu_enhancement'])
        self._maybe_log_baseline(result['baseline_summary'])

    def _publish_quality(self, quality: float) -> None:
        msg = Float32()
        msg.data = float(quality)
        self._quality_pub.publish(msg)

    def _publish_quality_after_imu(self, quality: float) -> None:
        msg = Float32()
        msg.data = float(quality)
        self._quality_after_imu_pub.publish(msg)

    def _publish_status(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _publish_debug(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._debug_pub.publish(msg)

    def _publish_imu_enhanced_status(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._imu_enhanced_status_pub.publish(msg)

    def _publish_imu_enhanced_debug(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._imu_enhanced_debug_pub.publish(msg)

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

    def _is_imu_fresh(self, now_ns: int) -> bool:
        if self._last_imu_received_ns is None or self._latest_imu_candidate is None:
            return False
        if not bool(self._latest_imu_candidate.get('valid', False)):
            return False
        return (now_ns - self._last_imu_received_ns) / 1e9 <= self._imu_stale_timeout_sec

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

    def _maybe_log_imu_enhancement(self, payload: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._log_hz)
        if now_ns - self._last_imu_enhancement_log_ns < min_gap_ns:
            return
        self._last_imu_enhancement_log_ns = now_ns
        self.get_logger().info(
            'rf2o_bridge_imu '
            f"stationary={payload['imu_stationary_flag']} "
            f"stationary_score={payload['imu_stationary_score']} "
            f"gyro_z={payload['raw_gyro_z']} "
            f"acc_norm={payload['raw_acc_norm']} "
            f"before={payload['radar_confidence_before_imu']} "
            f"after={payload['radar_confidence_after_imu']} "
            f"consistency={payload['imu_radar_consistency_score']} "
            f"reason={payload['consistency_reject_reason']}"
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
