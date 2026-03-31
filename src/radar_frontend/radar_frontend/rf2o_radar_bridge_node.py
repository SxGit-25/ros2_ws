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


def median_or_zero(values: Deque[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.median(values))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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
        self.declare_parameter('warn_speed_mps', 0.80)
        self.declare_parameter('max_yaw_rate_rps', 1.20)
        self.declare_parameter('warn_yaw_rate_rps', 0.80)
        self.declare_parameter('max_pose_jump_m', 0.25)
        self.declare_parameter('max_yaw_jump_deg', 20.0)
        self.declare_parameter('static_speed_threshold_mps', 0.05)
        self.declare_parameter('static_yaw_rate_threshold_rps', 0.08)
        self.declare_parameter('static_false_motion_frames', 3)
        self.declare_parameter('velocity_window_size', 3)
        self.declare_parameter('min_quality', 0.60)
        self.declare_parameter('low_confidence_quality', 0.40)
        self.declare_parameter('log_hz', 1.0)

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
        self._static_false_motion_frames = int(
            self.get_parameter('static_false_motion_frames').value
        )
        self._velocity_window_size = max(1, int(self.get_parameter('velocity_window_size').value))
        self._min_quality = float(self.get_parameter('min_quality').value)
        self._low_confidence_quality = float(
            self.get_parameter('low_confidence_quality').value
        )
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))

        self._previous_odom: Optional[Odometry] = None
        self._last_odom_msg: Optional[Odometry] = None
        self._last_odom_received_ns: Optional[int] = None
        self._last_scan_received_ns: Optional[int] = None
        self._static_false_motion_counter = 0
        self._last_log_ns = 0

        self._vx_window: Deque[float] = deque(maxlen=self._velocity_window_size)
        self._vy_window: Deque[float] = deque(maxlen=self._velocity_window_size)
        self._yaw_rate_window: Deque[float] = deque(maxlen=self._velocity_window_size)

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
        self._last_odom_msg = msg

        if self._previous_odom is None:
            self._publish_initial_waiting_status(msg)
            self._previous_odom = msg
            return

        result = self._build_bridge_result(self._previous_odom, msg, now_ns)
        self._publish_outputs(msg, result)
        self._previous_odom = msg

    def _publish_initial_waiting_status(self, msg: Odometry) -> None:
        status_payload = {
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
        self._publish_status(status_payload)
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
                'vx_body_raw': None,
                'vy_body_raw': None,
                'vx_body_filtered': 0.0,
                'vy_body_filtered': 0.0,
                'yaw_rate_raw': None,
                'yaw_rate_filtered': 0.0,
                'scan_fresh': self._is_scan_fresh(self.get_clock().now().nanoseconds),
                'odom_fresh': True,
                'quality_components': {},
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

        self._vx_window.append(vx_body_raw)
        self._vy_window.append(vy_body_raw)
        self._yaw_rate_window.append(yaw_rate_raw)

        vx_body_filtered = median_or_zero(self._vx_window)
        vy_body_filtered = median_or_zero(self._vy_window)
        yaw_rate_filtered = median_or_zero(self._yaw_rate_window)

        translation_norm_m = math.hypot(dx_world, dy_world)
        speed_norm_mps = math.hypot(vx_body_filtered, vy_body_filtered)
        yaw_jump_deg = abs(math.degrees(dyaw_rad))

        odom_fresh = self._is_odom_fresh(now_ns)
        scan_fresh = self._is_scan_fresh(now_ns)

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

        inside_static_band = (
            speed_norm_mps <= self._static_speed_threshold_mps
            and abs(yaw_rate_filtered) <= self._static_yaw_rate_threshold_rps
        )
        if inside_static_band:
            self._static_false_motion_counter = 0
        else:
            self._static_false_motion_counter += 1
            if self._static_false_motion_counter >= self._static_false_motion_frames:
                warnings.append('static_false_motion_suspected')

        vx_jitter = self._window_jitter(self._vx_window)
        vy_jitter = self._window_jitter(self._vy_window)
        yaw_jitter = self._window_jitter(self._yaw_rate_window)
        if vx_jitter > 0.08 or vy_jitter > 0.08 or yaw_jitter > 0.12:
            warnings.append('velocity_jitter_high')

        quality_components = self._compute_quality_components(
            dt_sec=dt_sec,
            odom_fresh=odom_fresh,
            scan_fresh=scan_fresh,
            speed_norm_mps=speed_norm_mps,
            yaw_rate_rps=abs(yaw_rate_filtered),
            translation_norm_m=translation_norm_m,
            yaw_jump_deg=yaw_jump_deg,
            static_motion_counter=self._static_false_motion_counter,
        )
        quality = round(
            quality_components['freshness']
            * quality_components['dt']
            * quality_components['speed']
            * quality_components['yaw_rate']
            * quality_components['jump']
            * quality_components['static_consistency'],
            4,
        )

        if reasons:
            status = STATUS_INVALID
        elif warnings or quality < self._min_quality:
            if quality < self._low_confidence_quality:
                if 'quality_too_low' not in warnings:
                    warnings.append('quality_too_low')
            else:
                if 'quality_low_confidence' not in warnings:
                    warnings.append('quality_low_confidence')
            status = STATUS_LOW_CONFIDENCE
        else:
            status = STATUS_VALID

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
            'debug': {
                'stamp_ns': curr_stamp_ns,
                'dt_sec': round_or_none(dt_sec, 6),
                'dx_world': round_or_none(dx_world),
                'dy_world': round_or_none(dy_world),
                'dyaw_rad': round_or_none(dyaw_rad, 6),
                'vx_world': round_or_none(vx_world),
                'vy_world': round_or_none(vy_world),
                'vx_body_raw': round_or_none(vx_body_raw),
                'vy_body_raw': round_or_none(vy_body_raw),
                'vx_body_filtered': round_or_none(vx_body_filtered),
                'vy_body_filtered': round_or_none(vy_body_filtered),
                'yaw_rate_raw': round_or_none(yaw_rate_raw),
                'yaw_rate_filtered': round_or_none(yaw_rate_filtered),
                'translation_norm_m': round_or_none(translation_norm_m),
                'speed_norm_mps': round_or_none(speed_norm_mps),
                'yaw_jump_deg': round_or_none(yaw_jump_deg),
                'scan_fresh': scan_fresh,
                'odom_fresh': odom_fresh,
                'quality_components': quality_components,
                'status': status,
                'reasons': reasons,
                'warnings': warnings,
            },
        }

    def _compute_quality_components(
        self,
        *,
        dt_sec: float,
        odom_fresh: bool,
        scan_fresh: bool,
        speed_norm_mps: float,
        yaw_rate_rps: float,
        translation_norm_m: float,
        yaw_jump_deg: float,
        static_motion_counter: int,
    ) -> Dict[str, float]:
        freshness_score = 1.0 if odom_fresh and scan_fresh else 0.1
        dt_center = 0.5 * (self._min_dt_sec + self._max_dt_sec)
        dt_span = max(0.5 * (self._max_dt_sec - self._min_dt_sec), 1e-3)
        dt_score = clamp(1.0 - abs(dt_sec - dt_center) / dt_span, 0.1, 1.0)
        speed_score = clamp(1.0 - speed_norm_mps / max(self._max_speed_mps, 1e-3), 0.1, 1.0)
        yaw_rate_score = clamp(
            1.0 - yaw_rate_rps / max(self._max_yaw_rate_rps, 1e-3), 0.1, 1.0
        )
        jump_score = clamp(
            min(
                1.0 - translation_norm_m / max(self._max_pose_jump_m, 1e-3),
                1.0 - yaw_jump_deg / max(self._max_yaw_jump_deg, 1e-3),
            ),
            0.1,
            1.0,
        )
        static_consistency_score = 1.0 if static_motion_counter == 0 else 0.65
        return {
            'freshness': round(freshness_score, 4),
            'dt': round(dt_score, 4),
            'speed': round(speed_score, 4),
            'yaw_rate': round(yaw_rate_score, 4),
            'jump': round(jump_score, 4),
            'static_consistency': round(static_consistency_score, 4),
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

    def _window_jitter(self, values: Deque[float]) -> float:
        if len(values) < 2:
            return 0.0
        return max(values) - min(values)

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
