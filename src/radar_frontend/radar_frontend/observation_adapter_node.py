import json
import math
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from radar_frontend.observation_candidate_state import ObservationCandidateState


def _clamp_int(value: float, low: int, high: int) -> int:
    return max(low, min(high, int(round(value))))


class ObservationAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('observation_adapter_node')

        self.declare_parameter('odom_topic', '/radar/odom_candidate')
        self.declare_parameter('vel_topic', '/radar/vel_candidate')
        self.declare_parameter('quality_topic', '/radar/match_quality')
        self.declare_parameter('status_topic', '/radar/odom_status')
        self.declare_parameter('source_name', 'radar_frontend')
        self.declare_parameter('candidate_frame_id', 'body')
        self.declare_parameter('min_quality_for_velocity', 0.72)
        self.declare_parameter('max_speed_mps', 1.50)
        self.declare_parameter('max_yaw_rate_rps', 1.50)
        self.declare_parameter('reject_on_warnings', True)
        self.declare_parameter('publish_hz', 10.0)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._vel_topic = str(self.get_parameter('vel_topic').value)
        self._quality_topic = str(self.get_parameter('quality_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._source_name = str(self.get_parameter('source_name').value)
        self._candidate_frame_id = str(self.get_parameter('candidate_frame_id').value)
        self._min_quality_for_velocity = float(
            self.get_parameter('min_quality_for_velocity').value
        )
        self._max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._max_yaw_rate_rps = float(self.get_parameter('max_yaw_rate_rps').value)
        self._reject_on_warnings = bool(self.get_parameter('reject_on_warnings').value)
        self._publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self._last_odom: Optional[Odometry] = None
        self._last_vel: Optional[TwistStamped] = None
        self._last_quality: Optional[float] = None
        self._last_status: Optional[Dict[str, object]] = None
        self._last_log_ns = 0

        self._candidate_pub = self.create_publisher(String, '/observation/radar_candidate', 10)
        self._status_pub = self.create_publisher(String, '/observation/radar_status', 10)

        self.create_subscription(Odometry, self._odom_topic, self._handle_odom, 10)
        self.create_subscription(TwistStamped, self._vel_topic, self._handle_vel, 10)
        self.create_subscription(Float32, self._quality_topic, self._handle_quality, 10)
        self.create_subscription(String, self._status_topic, self._handle_status, 10)
        self.create_timer(1.0 / self._publish_hz, self._publish_candidate)

        self.get_logger().info(
            'Observation adapter started '
            f'odom_topic={self._odom_topic} vel_topic={self._vel_topic} '
            f'quality_topic={self._quality_topic} status_topic={self._status_topic}'
        )

    def _handle_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _handle_vel(self, msg: TwistStamped) -> None:
        self._last_vel = msg

    def _handle_quality(self, msg: Float32) -> None:
        self._last_quality = float(msg.data)

    def _handle_status(self, msg: String) -> None:
        try:
            self._last_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._last_status = {
                'status': 'INVALID',
                'accept_motion': False,
                'reasons': ['status_json_decode_failed'],
                'warnings': [],
            }

    def _publish_candidate(self) -> None:
        candidate = self._build_candidate()
        candidate_msg = String()
        candidate_msg.data = json.dumps(candidate.to_dict(), sort_keys=True, ensure_ascii=True)
        self._candidate_pub.publish(candidate_msg)

        status_payload = {
            'source_name': candidate.source_name,
            'timestamp_ms': candidate.timestamp_ms,
            'status': candidate.status,
            'confidence': round(candidate.confidence, 4),
            'vel_valid': candidate.vel_valid,
            'pos_valid': candidate.pos_valid,
            'dist_valid': candidate.dist_valid,
            'reject_reason': candidate.reject_reason,
            'debug_info': candidate.debug_info,
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload, sort_keys=True, ensure_ascii=True)
        self._status_pub.publish(status_msg)
        self._maybe_log(status_payload)

    def _build_candidate(self) -> ObservationCandidateState:
        now_ms = self.get_clock().now().nanoseconds // 1_000_000

        if self._last_status is None or self._last_quality is None or self._last_vel is None:
            return ObservationCandidateState(
                timestamp_ms=int(now_ms),
                source_name=self._source_name,
                frame_id=self._candidate_frame_id,
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
                confidence=0.0,
                status='INVALID',
                reject_reason='missing_frontend_inputs',
                debug_info='Waiting for odom/vel/quality/status',
            )

        status = str(self._last_status.get('status', 'INVALID'))
        accept_motion = bool(self._last_status.get('accept_motion', False))
        warnings = [str(item) for item in self._last_status.get('warnings', [])]
        reasons = [str(item) for item in self._last_status.get('reasons', [])]
        quality = float(self._last_quality)
        vx = float(self._last_vel.twist.linear.x)
        vy = float(self._last_vel.twist.linear.y)
        vz = float(self._last_vel.twist.linear.z)
        yaw_rate = float(self._last_vel.twist.angular.z)
        speed_norm = math.hypot(vx, vy)

        vel_valid = True
        reject_reason = ''

        if status != 'VALID':
            vel_valid = False
            reject_reason = f'status_{status.lower()}'
        elif not accept_motion:
            vel_valid = False
            reject_reason = 'accept_motion_false'
        elif quality < self._min_quality_for_velocity:
            vel_valid = False
            reject_reason = 'quality_below_velocity_threshold'
        elif speed_norm > self._max_speed_mps:
            vel_valid = False
            reject_reason = 'speed_too_large'
        elif abs(yaw_rate) > self._max_yaw_rate_rps:
            vel_valid = False
            reject_reason = 'yaw_rate_too_large'
        elif reasons:
            vel_valid = False
            reject_reason = reasons[0]
        elif self._reject_on_warnings and warnings:
            vel_valid = False
            reject_reason = f'warning:{warnings[0]}'

        if vel_valid:
            vel_x_cms = _clamp_int(vx * 100.0, -32768, 32767)
            vel_y_cms = _clamp_int(vy * 100.0, -32768, 32767)
            vel_z_cms = _clamp_int(vz * 100.0, -32768, 32767)
        else:
            vel_x_cms = 0
            vel_y_cms = 0
            vel_z_cms = 0

        # First adapter version keeps position invalid on purpose.
        pos_valid = False
        dist_valid = False

        debug_info_parts = [
            f'quality={quality:.3f}',
            f'frontend_status={status}',
            f'accept_motion={accept_motion}',
            f'speed_norm={speed_norm:.3f}',
            f'yaw_rate={yaw_rate:.3f}',
        ]
        if warnings:
            debug_info_parts.append(f'warnings={warnings}')
        if reasons:
            debug_info_parts.append(f'reasons={reasons}')

        timestamp_ms = int(now_ms)
        if self._last_vel.header.stamp.sec != 0 or self._last_vel.header.stamp.nanosec != 0:
            timestamp_ms = (
                int(self._last_vel.header.stamp.sec) * 1000
                + int(self._last_vel.header.stamp.nanosec) // 1_000_000
            )

        return ObservationCandidateState(
            timestamp_ms=timestamp_ms,
            source_name=self._source_name,
            frame_id=self._candidate_frame_id,
            pos_x_cm=0,
            pos_y_cm=0,
            pos_z_cm=0,
            vel_x_cms=vel_x_cms,
            vel_y_cms=vel_y_cms,
            vel_z_cms=vel_z_cms,
            dist_direction=0,
            dist_angle_deg=0,
            dist_cm=0,
            pos_valid=pos_valid,
            vel_valid=vel_valid,
            dist_valid=dist_valid,
            confidence=max(0.0, min(1.0, quality)),
            status=status if vel_valid else 'REJECTED',
            reject_reason=reject_reason,
            debug_info='; '.join(debug_info_parts),
        )

    def _maybe_log(self, status_payload: Dict[str, object]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 1_000_000_000:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            'observation_adapter '
            f"status={status_payload['status']} "
            f"confidence={status_payload['confidence']} "
            f"vel_valid={status_payload['vel_valid']} "
            f"pos_valid={status_payload['pos_valid']} "
            f"dist_valid={status_payload['dist_valid']} "
            f"reject_reason={status_payload['reject_reason']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObservationAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
