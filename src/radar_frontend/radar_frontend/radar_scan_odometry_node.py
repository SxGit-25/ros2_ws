import json
import math
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from radar_frontend.quality_gate import (
    STATUS_INVALID,
    STATUS_LOW_CONFIDENCE,
    STATUS_VALID,
    evaluate_odometry_candidate,
)
from radar_frontend.scan_matching_utils import match_scan_summaries
from radar_frontend.scan_metrics import round_or_none, scan_to_summary


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    msg = Quaternion()
    msg.w = math.cos(yaw_rad * 0.5)
    msg.x = 0.0
    msg.y = 0.0
    msg.z = math.sin(yaw_rad * 0.5)
    return msg


class RadarScanOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('radar_scan_odometry_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_distance_threshold_m', 1.5)
        self.declare_parameter('sector_mode', 'six')
        self.declare_parameter('sector_count', 6)
        self.declare_parameter('log_hz', 1.0)
        self.declare_parameter('min_valid_points', 80)
        self.declare_parameter('min_valid_ratio', 0.20)
        self.declare_parameter('min_overlap_ratio', 0.35)
        self.declare_parameter('max_median_residual_m', 0.25)
        self.declare_parameter('max_translation_norm_m', 0.80)
        self.declare_parameter('max_translation_step_m', 0.80)
        self.declare_parameter('max_yaw_search_deg', 12.0)
        self.declare_parameter('yaw_search_step_deg', 1.0)
        self.declare_parameter('min_quality', 0.60)
        self.declare_parameter('low_confidence_quality', 0.40)
        self.declare_parameter('high_confidence_quality', 0.82)
        self.declare_parameter('min_dt_sec', 0.03)
        self.declare_parameter('max_dt_sec', 0.25)
        self.declare_parameter('sector_median_jump_threshold_m', 0.80)
        self.declare_parameter('min_inlier_ratio', 0.55)
        self.declare_parameter('search_edge_ratio_limit', 0.80)
        self.declare_parameter('static_translation_deadband_m', 0.03)
        self.declare_parameter('static_yaw_deadband_deg', 0.35)
        self.declare_parameter('publish_debug', True)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._obstacle_distance_threshold_m = float(
            self.get_parameter('obstacle_distance_threshold_m').value
        )
        self._sector_mode = str(self.get_parameter('sector_mode').value).strip().lower()
        self._sector_count = max(1, int(self.get_parameter('sector_count').value))
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._min_valid_points = int(self.get_parameter('min_valid_points').value)
        self._min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self._min_overlap_ratio = float(self.get_parameter('min_overlap_ratio').value)
        self._max_median_residual_m = float(self.get_parameter('max_median_residual_m').value)
        self._max_translation_norm_m = float(self.get_parameter('max_translation_norm_m').value)
        self._max_translation_step_m = float(self.get_parameter('max_translation_step_m').value)
        self._max_yaw_search_deg = float(self.get_parameter('max_yaw_search_deg').value)
        self._yaw_search_step_deg = float(self.get_parameter('yaw_search_step_deg').value)
        self._min_quality = float(self.get_parameter('min_quality').value)
        self._low_confidence_quality = float(
            self.get_parameter('low_confidence_quality').value
        )
        self._high_confidence_quality = float(
            self.get_parameter('high_confidence_quality').value
        )
        self._min_dt_sec = float(self.get_parameter('min_dt_sec').value)
        self._max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self._sector_median_jump_threshold_m = float(
            self.get_parameter('sector_median_jump_threshold_m').value
        )
        self._min_inlier_ratio = float(self.get_parameter('min_inlier_ratio').value)
        self._search_edge_ratio_limit = float(
            self.get_parameter('search_edge_ratio_limit').value
        )
        self._static_translation_deadband_m = float(
            self.get_parameter('static_translation_deadband_m').value
        )
        self._static_yaw_deadband_deg = float(
            self.get_parameter('static_yaw_deadband_deg').value
        )
        self._publish_debug = bool(self.get_parameter('publish_debug').value)

        self._previous_summary: Optional[Dict[str, object]] = None
        self._previous_stamp_ns: Optional[int] = None
        self._last_log_time_ns = 0

        self._odom_x_m = 0.0
        self._odom_y_m = 0.0
        self._odom_yaw_rad = 0.0

        self._odom_pub = self.create_publisher(Odometry, '/radar/odom_candidate', 10)
        self._vel_pub = self.create_publisher(TwistStamped, '/radar/vel_candidate', 10)
        self._quality_pub = self.create_publisher(Float32, '/radar/match_quality', 10)
        self._status_pub = self.create_publisher(String, '/radar/odom_status', 10)
        self._debug_pub = self.create_publisher(String, '/radar/match_debug', 10)

        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, 10)
        self.get_logger().info(
            'Radar scan odometry started '
            f'scan_topic={self._scan_topic} min_quality={self._min_quality:.2f} '
            f'goal=stable_zero_motion_before_precise_dxdy'
        )

    def _handle_scan(self, scan: LaserScan) -> None:
        current_summary = scan_to_summary(
            scan,
            sector_mode=self._sector_mode,
            sector_count=self._sector_count,
            obstacle_distance_threshold_m=self._obstacle_distance_threshold_m,
        )
        current_stamp_ns = int(current_summary['stamp_ns'])

        if self._previous_summary is None or self._previous_stamp_ns is None:
            self._publish_status(
                status=STATUS_INVALID,
                payload={
                    'status': STATUS_INVALID,
                    'reason': 'waiting_for_previous_frame',
                    'stamp_ns': current_stamp_ns,
                },
            )
            self._previous_summary = current_summary
            self._previous_stamp_ns = current_stamp_ns
            return

        delta_ns = current_stamp_ns - self._previous_stamp_ns
        timestamp_increasing = delta_ns > 0
        dt_sec = delta_ns / 1e9 if delta_ns > 0 else 0.0

        angle_increment_rad = abs(float(scan.angle_increment))
        match_result = match_scan_summaries(
            self._previous_summary,
            current_summary,
            angle_increment_rad=angle_increment_rad,
            max_yaw_search_deg=self._max_yaw_search_deg,
            yaw_search_step_deg=self._yaw_search_step_deg,
            max_translation_step_m=self._max_translation_step_m,
        )

        gate = evaluate_odometry_candidate(
            dt_sec=dt_sec,
            timestamp_increasing=timestamp_increasing,
            current_summary=current_summary,
            previous_summary=self._previous_summary,
            match_result=match_result,
            min_valid_points=self._min_valid_points,
            min_valid_ratio=self._min_valid_ratio,
            min_overlap_ratio=self._min_overlap_ratio,
            max_median_residual_m=self._max_median_residual_m,
            max_translation_norm_m=self._max_translation_norm_m,
            min_quality=self._min_quality,
            low_confidence_quality=self._low_confidence_quality,
            max_dt_sec=self._max_dt_sec,
            min_dt_sec=self._min_dt_sec,
            sector_median_jump_threshold_m=self._sector_median_jump_threshold_m,
            min_inlier_ratio=self._min_inlier_ratio,
            search_edge_ratio_limit=self._search_edge_ratio_limit,
            high_confidence_quality=self._high_confidence_quality,
            static_translation_deadband_m=self._static_translation_deadband_m,
            static_yaw_deadband_deg=self._static_yaw_deadband_deg,
        )

        quality_value = float(match_result.get('quality', 0.0)) if match_result.get('success') else 0.0
        quality_msg = Float32()
        quality_msg.data = quality_value
        self._quality_pub.publish(quality_msg)

        status_payload = {
            'status': gate['status'],
            'stamp_ns': current_stamp_ns,
            'dt_sec': round_or_none(dt_sec, 6),
            'quality': round_or_none(quality_value),
            'accept_motion': gate['accept_motion'],
            'reasons': gate['reasons'],
            'warnings': gate['warnings'],
        }

        if match_result.get('success'):
            dx_m = float(match_result['delta_x_m'])
            dy_m = float(match_result['delta_y_m'])
            delta_yaw_rad = float(match_result['delta_yaw_rad'])

            inside_static_deadband = (
                math.hypot(dx_m, dy_m) <= self._static_translation_deadband_m
                and abs(math.degrees(delta_yaw_rad)) <= self._static_yaw_deadband_deg
            )
            if inside_static_deadband and quality_value < self._high_confidence_quality:
                dx_m = 0.0
                dy_m = 0.0
                delta_yaw_rad = 0.0
                gate['accept_motion'] = False
                if gate['status'] == STATUS_VALID:
                    gate['status'] = STATUS_LOW_CONFIDENCE
                if 'zeroed_by_static_deadband' not in gate['warnings']:
                    gate['warnings'].append('zeroed_by_static_deadband')

            status_payload['status'] = gate['status']
            status_payload['accept_motion'] = gate['accept_motion']
            status_payload['reasons'] = gate['reasons']
            status_payload['warnings'] = gate['warnings']

            vx_mps = dx_m / dt_sec if dt_sec > 0.0 else 0.0
            vy_mps = dy_m / dt_sec if dt_sec > 0.0 else 0.0
            yaw_rate_rps = delta_yaw_rad / dt_sec if dt_sec > 0.0 else 0.0

            status_payload.update(
                {
                    'delta_x_m': round_or_none(dx_m),
                    'delta_y_m': round_or_none(dy_m),
                    'delta_yaw_deg': round_or_none(math.degrees(delta_yaw_rad)),
                    'vx_mps': round_or_none(vx_mps),
                    'vy_mps': round_or_none(vy_mps),
                    'yaw_rate_rps': round_or_none(yaw_rate_rps),
                    'overlap_ratio': round_or_none(float(match_result['overlap_ratio'])),
                    'median_residual_m': round_or_none(float(match_result['median_residual_m'])),
                    'inlier_ratio': round_or_none(float(match_result.get('inlier_ratio', 0.0))),
                    'edge_ratio': round_or_none(float(match_result.get('edge_ratio', 0.0))),
                    'max_sector_jump_m': round_or_none(
                        float(match_result.get('max_sector_jump_m') or 0.0)
                    ),
                    'refined_shift_bins': round_or_none(
                        float(match_result.get('refined_shift_bins', match_result['shift_bins'])),
                        3,
                    ),
                    'comparable_points': int(match_result['comparable_points']),
                }
            )

            if gate['accept_motion']:
                self._odom_x_m += dx_m
                self._odom_y_m += dy_m
                self._odom_yaw_rad += delta_yaw_rad

                self._publish_velocity(scan, vx_mps, vy_mps, yaw_rate_rps)
                self._publish_odometry(scan, vx_mps, vy_mps, yaw_rate_rps)
            elif gate['status'] == STATUS_LOW_CONFIDENCE:
                self._publish_zero_velocity(scan)
                self._publish_odometry(scan, 0.0, 0.0, 0.0)
        else:
            if gate['status'] != STATUS_VALID:
                self._publish_zero_velocity(scan)
                self._publish_odometry(scan, 0.0, 0.0, 0.0)

        self._publish_status(gate['status'], status_payload)
        if self._publish_debug:
            self._publish_debug_payload(current_summary, match_result, gate, dt_sec)
        self._maybe_log(status_payload)

        self._previous_summary = current_summary
        self._previous_stamp_ns = current_stamp_ns

    def _publish_velocity(self, scan: LaserScan, vx_mps: float, vy_mps: float, yaw_rate_rps: float) -> None:
        msg = TwistStamped()
        msg.header = scan.header
        msg.twist.linear.x = vx_mps
        msg.twist.linear.y = vy_mps
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = yaw_rate_rps
        self._vel_pub.publish(msg)

    def _publish_zero_velocity(self, scan: LaserScan) -> None:
        self._publish_velocity(scan, 0.0, 0.0, 0.0)

    def _publish_odometry(self, scan: LaserScan, vx_mps: float, vy_mps: float, yaw_rate_rps: float) -> None:
        msg = Odometry()
        msg.header = scan.header
        msg.child_frame_id = 'radar_odom_candidate'
        msg.pose.pose.position.x = self._odom_x_m
        msg.pose.pose.position.y = self._odom_y_m
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(self._odom_yaw_rad)
        msg.twist.twist.linear.x = vx_mps
        msg.twist.twist.linear.y = vy_mps
        msg.twist.twist.angular.z = yaw_rate_rps
        self._odom_pub.publish(msg)

    def _publish_status(self, status: str, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _publish_debug_payload(
        self,
        current_summary: Dict[str, object],
        match_result: Dict[str, object],
        gate: Dict[str, object],
        dt_sec: float,
    ) -> None:
        payload = {
            'stamp_ns': current_summary['stamp_ns'],
            'valid_points': current_summary['valid_points'],
            'valid_ratio': round_or_none(float(current_summary['valid_ratio'])),
            'nearest_valid_m': round_or_none(current_summary['nearest_valid_m']),
            'median_valid_m': round_or_none(current_summary['median_valid_m']),
            'sectors': current_summary['sectors_payload'],
            'match': match_result,
            'gate': gate,
            'dt_sec': round_or_none(dt_sec, 6),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        self._debug_pub.publish(msg)

    def _maybe_log(self, status_payload: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._log_hz)
        if now_ns - self._last_log_time_ns < min_gap_ns:
            return
        self._last_log_time_ns = now_ns
        self.get_logger().info(
            'radar_odom '
            f"status={status_payload['status']} "
            f"dt={status_payload.get('dt_sec')} "
            f"q={status_payload.get('quality')} "
            f"dx={status_payload.get('delta_x_m')} "
            f"dy={status_payload.get('delta_y_m')} "
            f"dyaw_deg={status_payload.get('delta_yaw_deg')} "
            f"vx={status_payload.get('vx_mps')} "
            f"vy={status_payload.get('vy_mps')} "
            f"yaw_rate={status_payload.get('yaw_rate_rps')} "
            f"reasons={status_payload.get('reasons')} "
            f"warnings={status_payload.get('warnings')}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarScanOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
