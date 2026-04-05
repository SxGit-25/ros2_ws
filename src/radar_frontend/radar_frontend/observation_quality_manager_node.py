import json
import math
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

from radar_frontend.observation_candidate_state import ObservationCandidateState


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _round_or_none(value: Optional[float], digits: int = 4) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _stamp_to_ms(stamp) -> int:
    return int(stamp.sec) * 1000 + int(stamp.nanosec) // 1_000_000


def _json_loads_or_none(payload: str) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    return data


class ObservationQualityManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('observation_quality_manager_node')

        self.declare_parameter('imu_candidate_topic', '/observation/imu_candidate_state')
        self.declare_parameter('flow_candidate_state_topic', '/observation/flow_candidate_state')
        self.declare_parameter('radar_twist_candidate_topic', '/radar/vel_candidate_enhanced')
        self.declare_parameter('radar_confidence_topic', '/radar/match_quality_after_imu')
        self.declare_parameter('radar_status_topic', '/radar/imu_enhanced_status')
        self.declare_parameter('flow_twist_candidate_topic', '/observation/flow_twist_candidate')
        self.declare_parameter('flow_candidate_status_topic', '/observation/flow_candidate_status')
        self.declare_parameter(
            'radar_observation_candidate_topic', '/observation/radar_observation_candidate'
        )
        self.declare_parameter(
            'flow_observation_candidate_topic', '/observation/flow_observation_candidate'
        )
        self.declare_parameter('stationary_status_topic', '/observation/stationary_status')
        self.declare_parameter('health_report_topic', '/observation/health_report')
        self.declare_parameter('frame_id', 'body_unverified')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('flow_quality_valid_threshold', 80)
        self.declare_parameter('flow_alt_min_cm', 15)
        self.declare_parameter('flow_alt_max_cm', 350)
        self.declare_parameter('flow_jump_speed_threshold', 0.80)
        self.declare_parameter('flow_stale_timeout_sec', 0.5)
        self.declare_parameter('radar_stale_timeout_sec', 0.5)
        self.declare_parameter('radar_min_confidence', 0.45)
        self.declare_parameter('flow_min_confidence', 0.45)
        self.declare_parameter('radar_flow_consistency_speed_threshold', 0.18)
        self.declare_parameter('stationary_speed_threshold_mps', 0.05)
        self.declare_parameter('stationary_hold_frames', 4)
        self.declare_parameter('stationary_release_frames', 2)
        self.declare_parameter('log_hz', 1.0)

        self._imu_candidate_topic = str(self.get_parameter('imu_candidate_topic').value)
        self._flow_candidate_state_topic = str(
            self.get_parameter('flow_candidate_state_topic').value
        )
        self._radar_twist_candidate_topic = str(
            self.get_parameter('radar_twist_candidate_topic').value
        )
        self._radar_confidence_topic = str(self.get_parameter('radar_confidence_topic').value)
        self._radar_status_topic = str(self.get_parameter('radar_status_topic').value)
        self._flow_twist_candidate_topic = str(
            self.get_parameter('flow_twist_candidate_topic').value
        )
        self._flow_candidate_status_topic = str(
            self.get_parameter('flow_candidate_status_topic').value
        )
        self._radar_observation_candidate_topic = str(
            self.get_parameter('radar_observation_candidate_topic').value
        )
        self._flow_observation_candidate_topic = str(
            self.get_parameter('flow_observation_candidate_topic').value
        )
        self._stationary_status_topic = str(self.get_parameter('stationary_status_topic').value)
        self._health_report_topic = str(self.get_parameter('health_report_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self._flow_quality_valid_threshold = int(
            self.get_parameter('flow_quality_valid_threshold').value
        )
        self._flow_alt_min_cm = int(self.get_parameter('flow_alt_min_cm').value)
        self._flow_alt_max_cm = int(self.get_parameter('flow_alt_max_cm').value)
        self._flow_jump_speed_threshold = float(
            self.get_parameter('flow_jump_speed_threshold').value
        )
        self._flow_stale_timeout_sec = float(
            self.get_parameter('flow_stale_timeout_sec').value
        )
        self._radar_stale_timeout_sec = float(
            self.get_parameter('radar_stale_timeout_sec').value
        )
        self._radar_min_confidence = float(self.get_parameter('radar_min_confidence').value)
        self._flow_min_confidence = float(self.get_parameter('flow_min_confidence').value)
        self._radar_flow_consistency_speed_threshold = float(
            self.get_parameter('radar_flow_consistency_speed_threshold').value
        )
        self._stationary_speed_threshold_mps = float(
            self.get_parameter('stationary_speed_threshold_mps').value
        )
        self._stationary_hold_frames = max(
            2, int(self.get_parameter('stationary_hold_frames').value)
        )
        self._stationary_release_frames = max(
            1, int(self.get_parameter('stationary_release_frames').value)
        )
        self._log_hz = max(0.2, float(self.get_parameter('log_hz').value))

        self._latest_imu_candidate: Optional[Dict[str, Any]] = None
        self._latest_flow_candidate: Optional[Dict[str, Any]] = None
        self._latest_radar_status: Optional[Dict[str, Any]] = None
        self._latest_radar_twist: Optional[TwistStamped] = None
        self._latest_radar_confidence: Optional[float] = None
        self._last_flow_sample: Optional[Tuple[float, float]] = None
        self._last_log_ns = 0
        self._stationary_hold_counter = 0
        self._stationary_release_counter = 0
        self._stationary_flag = False

        self._flow_twist_pub = self.create_publisher(TwistStamped, self._flow_twist_candidate_topic, 10)
        self._flow_status_pub = self.create_publisher(String, self._flow_candidate_status_topic, 10)
        self._radar_candidate_pub = self.create_publisher(
            String, self._radar_observation_candidate_topic, 10
        )
        self._flow_candidate_pub = self.create_publisher(
            String, self._flow_observation_candidate_topic, 10
        )
        self._stationary_status_pub = self.create_publisher(String, self._stationary_status_topic, 10)
        self._health_report_pub = self.create_publisher(String, self._health_report_topic, 10)

        self.create_subscription(String, self._imu_candidate_topic, self._handle_imu_candidate, 10)
        self.create_subscription(
            String, self._flow_candidate_state_topic, self._handle_flow_candidate, 10
        )
        self.create_subscription(
            TwistStamped, self._radar_twist_candidate_topic, self._handle_radar_twist, 10
        )
        self.create_subscription(
            String, self._radar_status_topic, self._handle_radar_status, 10
        )
        self.create_subscription(
            Float32, self._radar_confidence_topic, self._handle_radar_confidence, 10
        )
        self.create_timer(1.0 / self._publish_hz, self._publish_once)

        self.get_logger().info(
            'Observation quality manager started '
            f'radar_twist_topic={self._radar_twist_candidate_topic} '
            f'flow_candidate_topic={self._flow_candidate_state_topic}'
        )

    def _handle_imu_candidate(self, msg: String) -> None:
        self._latest_imu_candidate = _json_loads_or_none(msg.data)

    def _handle_flow_candidate(self, msg: String) -> None:
        self._latest_flow_candidate = _json_loads_or_none(msg.data)

    def _handle_radar_twist(self, msg: TwistStamped) -> None:
        self._latest_radar_twist = msg

    def _handle_radar_status(self, msg: String) -> None:
        self._latest_radar_status = _json_loads_or_none(msg.data)

    def _handle_radar_confidence(self, msg: Float32) -> None:
        self._latest_radar_confidence = float(msg.data)

    def _publish_once(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now_ms = now_ns // 1_000_000

        radar_eval = self._build_radar_candidate(now_ns, now_ms)
        flow_eval = self._build_flow_candidate(now_ns, now_ms)
        self._apply_cross_source_arbitration(
            radar_eval=radar_eval,
            flow_eval=flow_eval,
        )
        stationary_payload = self._build_stationary_payload(
            now_ms=now_ms,
            radar_eval=radar_eval,
            flow_eval=flow_eval,
        )
        health_payload = self._build_health_report(
            now_ms=now_ms,
            radar_eval=radar_eval,
            flow_eval=flow_eval,
            stationary_payload=stationary_payload,
        )

        self._publish_flow_twist(flow_eval)
        self._publish_json(self._flow_status_pub, flow_eval['status_payload'])
        self._publish_json(self._radar_candidate_pub, radar_eval['candidate'].to_dict())
        self._publish_json(self._flow_candidate_pub, flow_eval['candidate'].to_dict())
        self._publish_json(self._stationary_status_pub, stationary_payload)
        self._publish_json(self._health_report_pub, health_payload)
        self._maybe_log(health_payload)

    def _apply_cross_source_arbitration(
        self,
        *,
        radar_eval: Dict[str, Any],
        flow_eval: Dict[str, Any],
    ) -> None:
        imu_stationary = bool(self._latest_radar_status.get('imu_stationary_flag', False)) if self._latest_radar_status else False
        radar_speed = float(radar_eval['speed_mps'])
        flow_speed = float(flow_eval['speed_mps'])
        radar_valid = bool(radar_eval['candidate'].vel_valid)
        flow_valid = bool(flow_eval['candidate'].vel_valid)

        if imu_stationary and flow_valid and flow_speed <= self._stationary_speed_threshold_mps and radar_speed > self._stationary_speed_threshold_mps * 2.0:
            adjusted_conf = _clamp(float(radar_eval['confidence']) * 0.45, 0.0, 1.0)
            adjusted_valid = adjusted_conf >= self._radar_min_confidence
            radar_eval['confidence'] = adjusted_conf
            radar_eval['reject_reason'] = 'radar_conflicts_with_static_imu_and_flow'
            radar_eval['candidate'] = self._make_candidate(
                timestamp_ms=radar_eval['candidate'].timestamp_ms,
                source_name=radar_eval['candidate'].source_name,
                vx_mps=radar_eval['candidate'].vel_x_cms / 100.0 if adjusted_valid else 0.0,
                vy_mps=radar_eval['candidate'].vel_y_cms / 100.0 if adjusted_valid else 0.0,
                vz_mps=radar_eval['candidate'].vel_z_cms / 100.0 if adjusted_valid else 0.0,
                valid=adjusted_valid,
                confidence=adjusted_conf,
                reject_reason=radar_eval['reject_reason'],
                debug_info=radar_eval['candidate'].debug_info + '; cross_source=flow_supported_static',
            )
            radar_eval['status_payload']['radar_confidence_after_imu'] = round(adjusted_conf, 4)
            radar_eval['status_payload']['radar_reject_reason'] = radar_eval['reject_reason']

        if imu_stationary and radar_valid and radar_speed <= self._stationary_speed_threshold_mps and flow_speed > self._stationary_speed_threshold_mps * 2.0:
            adjusted_conf = _clamp(float(flow_eval['confidence']) * 0.45, 0.0, 1.0)
            adjusted_valid = adjusted_conf >= self._flow_min_confidence
            flow_eval['confidence'] = adjusted_conf
            flow_eval['reject_reason'] = 'flow_conflicts_with_static_imu_and_radar'
            flow_eval['candidate'] = self._make_candidate(
                timestamp_ms=flow_eval['candidate'].timestamp_ms,
                source_name=flow_eval['candidate'].source_name,
                vx_mps=flow_eval['raw_vx'] if adjusted_valid else 0.0,
                vy_mps=flow_eval['raw_vy'] if adjusted_valid else 0.0,
                vz_mps=0.0,
                valid=adjusted_valid,
                confidence=adjusted_conf,
                reject_reason=flow_eval['reject_reason'],
                debug_info=flow_eval['candidate'].debug_info + '; cross_source=radar_supported_static',
            )
            flow_eval['status_payload']['flow_confidence'] = round(adjusted_conf, 4)
            flow_eval['status_payload']['flow_reject_reason'] = flow_eval['reject_reason']
            flow_eval['status_payload']['flow_valid'] = adjusted_valid

    def _build_radar_candidate(self, now_ns: int, now_ms: int) -> Dict[str, Any]:
        if (
            self._latest_radar_twist is None
            or self._latest_radar_confidence is None
            or self._latest_radar_status is None
        ):
            candidate = self._make_candidate(
                timestamp_ms=int(now_ms),
                source_name='radar_enhanced',
                vx_mps=0.0,
                vy_mps=0.0,
                vz_mps=0.0,
                valid=False,
                confidence=0.0,
                reject_reason='missing_radar_inputs',
                debug_info='Waiting for enhanced radar twist/confidence/status',
            )
            return {
                'candidate': candidate,
                'speed_mps': 0.0,
                'reject_reason': 'missing_radar_inputs',
                'confidence': 0.0,
                'status_payload': {
                    'radar_confidence_after_imu': 0.0,
                    'radar_reject_reason': 'missing_radar_inputs',
                },
            }

        stamp_ms = _stamp_to_ms(self._latest_radar_twist.header.stamp)
        age_sec = max((now_ms - stamp_ms) / 1000.0, 0.0)
        vx = float(self._latest_radar_twist.twist.linear.x)
        vy = float(self._latest_radar_twist.twist.linear.y)
        vz = float(self._latest_radar_twist.twist.linear.z)
        speed_norm = math.hypot(vx, vy)
        confidence = _clamp(float(self._latest_radar_confidence), 0.0, 1.0)
        reject_reason = ''
        valid = True

        if age_sec > self._radar_stale_timeout_sec:
            valid = False
            reject_reason = 'radar_stale'
        elif confidence < self._radar_min_confidence:
            valid = False
            reject_reason = 'radar_confidence_below_threshold'

        status_reason = str(self._latest_radar_status.get('consistency_reject_reason', ''))
        if status_reason and confidence < max(self._radar_min_confidence, 0.6):
            valid = False
            reject_reason = status_reason

        candidate = self._make_candidate(
            timestamp_ms=stamp_ms if stamp_ms > 0 else int(now_ms),
            source_name='radar_enhanced',
            vx_mps=vx if valid else 0.0,
            vy_mps=vy if valid else 0.0,
            vz_mps=vz if valid else 0.0,
            valid=valid,
            confidence=confidence,
            reject_reason=reject_reason,
            debug_info=(
                f"radar_confidence_after_imu={confidence:.3f}; "
                f"imu_radar_consistency_score={self._latest_radar_status.get('imu_radar_consistency_score', 0.0)}; "
                f"status_reason={status_reason or '-'}"
            ),
        )
        return {
            'candidate': candidate,
            'speed_mps': speed_norm,
            'reject_reason': reject_reason,
            'confidence': confidence,
            'status_payload': {
                'radar_confidence_after_imu': confidence,
                'radar_reject_reason': reject_reason or status_reason,
            },
        }

    def _build_flow_candidate(self, now_ns: int, now_ms: int) -> Dict[str, Any]:
        if self._latest_flow_candidate is None:
            candidate = self._make_candidate(
                timestamp_ms=int(now_ms),
                source_name='flow_observation',
                vx_mps=0.0,
                vy_mps=0.0,
                vz_mps=0.0,
                valid=False,
                confidence=0.0,
                reject_reason='missing_flow_input',
                debug_info='Waiting for flow_candidate_state',
            )
            return {
                'candidate': candidate,
                'raw_vx': 0.0,
                'raw_vy': 0.0,
                'speed_mps': 0.0,
                'confidence': 0.0,
                'reject_reason': 'missing_flow_input',
                'status_payload': {
                    'flow_confidence': 0.0,
                    'flow_reject_reason': 'missing_flow_input',
                    'flow_valid': False,
                },
            }

        stamp_ms = int(self._latest_flow_candidate.get('stamp_ms', now_ms))
        age_sec = max((now_ms - stamp_ms) / 1000.0, 0.0)
        vx = float(self._latest_flow_candidate.get('flow_vx', 0.0))
        vy = float(self._latest_flow_candidate.get('flow_vy', 0.0))
        speed_norm = math.hypot(vx, vy)
        flow_state = int(self._latest_flow_candidate.get('flow_state', 0))
        flow_quality = int(self._latest_flow_candidate.get('flow_quality', 0))
        alt_cm = int(self._latest_flow_candidate.get('alt_cm', 0))
        raw_valid = bool(self._latest_flow_candidate.get('valid', False)) and flow_state != 0

        jump_detected = False
        if self._last_flow_sample is not None:
            prev_vx, prev_vy = self._last_flow_sample
            jump_detected = math.hypot(vx - prev_vx, vy - prev_vy) > self._flow_jump_speed_threshold
        self._last_flow_sample = (vx, vy)

        quality_score = _clamp(flow_quality / max(self._flow_quality_valid_threshold, 1), 0.0, 1.0)
        freshness_score = 1.0 if age_sec <= self._flow_stale_timeout_sec else 0.1
        altitude_ok = self._flow_alt_min_cm <= alt_cm <= self._flow_alt_max_cm
        altitude_score = 1.0 if altitude_ok else 0.15
        jump_score = 0.35 if jump_detected else 1.0
        state_score = 1.0 if raw_valid else 0.0
        confidence = _clamp(
            0.30 * state_score
            + 0.25 * quality_score
            + 0.20 * freshness_score
            + 0.15 * altitude_score
            + 0.10 * jump_score,
            0.0,
            1.0,
        )

        reject_reason = ''
        valid = True
        if not raw_valid:
            valid = False
            reject_reason = 'flow_state_invalid'
        elif flow_quality < self._flow_quality_valid_threshold:
            valid = False
            reject_reason = 'flow_quality_below_threshold'
        elif not altitude_ok:
            valid = False
            reject_reason = 'flow_alt_out_of_range'
        elif age_sec > self._flow_stale_timeout_sec:
            valid = False
            reject_reason = 'flow_stale'
        elif jump_detected:
            valid = False
            reject_reason = 'flow_jump_detected'
        elif confidence < self._flow_min_confidence:
            valid = False
            reject_reason = 'flow_confidence_below_threshold'

        candidate = self._make_candidate(
            timestamp_ms=stamp_ms,
            source_name='flow_observation',
            vx_mps=vx if valid else 0.0,
            vy_mps=vy if valid else 0.0,
            vz_mps=0.0,
            valid=valid,
            confidence=confidence,
            reject_reason=reject_reason,
            debug_info=(
                f"flow_state={flow_state}; flow_quality={flow_quality}; alt_cm={alt_cm}; "
                f"stale_sec={age_sec:.3f}; jump_detected={jump_detected}"
            ),
        )
        return {
            'candidate': candidate,
            'raw_vx': vx,
            'raw_vy': vy,
            'speed_mps': speed_norm,
            'confidence': confidence,
            'reject_reason': reject_reason,
            'status_payload': {
                'flow_confidence': round(confidence, 4),
                'flow_reject_reason': reject_reason,
                'flow_valid': valid,
                'flow_quality': flow_quality,
                'flow_alt_cm': alt_cm,
                'flow_jump_detected': jump_detected,
            },
        }

    def _build_stationary_payload(
        self,
        *,
        now_ms: int,
        radar_eval: Dict[str, Any],
        flow_eval: Dict[str, Any],
    ) -> Dict[str, Any]:
        imu_stationary = bool(self._latest_radar_status.get('imu_stationary_flag', False)) if self._latest_radar_status else False
        radar_small = radar_eval['speed_mps'] <= self._stationary_speed_threshold_mps
        flow_small = flow_eval['speed_mps'] <= self._stationary_speed_threshold_mps

        if imu_stationary and radar_small and (not flow_eval['candidate'].vel_valid or flow_small):
            self._stationary_hold_counter += 1
            self._stationary_release_counter = 0
        else:
            self._stationary_release_counter += 1
            self._stationary_hold_counter = 0

        if not self._stationary_flag and self._stationary_hold_counter >= self._stationary_hold_frames:
            self._stationary_flag = True
        elif self._stationary_flag and self._stationary_release_counter >= self._stationary_release_frames:
            self._stationary_flag = False

        return {
            'timestamp_ms': int(now_ms),
            'stationary_flag': self._stationary_flag,
            'imu_stationary_flag': imu_stationary,
            'imu_stationary_score': _round_or_none(
                self._latest_radar_status.get('imu_stationary_score', 0.0)
                if self._latest_radar_status
                else 0.0
            ),
            'radar_speed_mps': _round_or_none(radar_eval['speed_mps']),
            'flow_speed_mps': _round_or_none(flow_eval['speed_mps']),
        }

    def _build_health_report(
        self,
        *,
        now_ms: int,
        radar_eval: Dict[str, Any],
        flow_eval: Dict[str, Any],
        stationary_payload: Dict[str, Any],
    ) -> Dict[str, Any]:
        radar_candidate = radar_eval['candidate']
        flow_candidate = flow_eval['candidate']
        radar_confidence = float(radar_eval['confidence'])
        flow_confidence = float(flow_eval['confidence'])
        radar_reject_reason = str(radar_eval['reject_reason'])
        flow_reject_reason = str(flow_eval['reject_reason'])
        notes = []

        if stationary_payload['stationary_flag']:
            if radar_candidate.vel_valid and flow_candidate.vel_valid:
                health_state = 'STATIONARY_MULTI_SOURCE_OK'
                notes.append('imu_radar_flow_support_static')
            elif radar_candidate.vel_valid:
                health_state = 'STATIONARY_FLOW_DEGRADED'
                notes.append('flow_degraded_while_static')
            elif flow_candidate.vel_valid:
                health_state = 'STATIONARY_RADAR_DEGRADED'
                notes.append('radar_degraded_while_static')
            else:
                health_state = 'STATIONARY_NO_CONFIDENT_SOURCE'
                notes.append('both_sources_degraded_while_static')
        else:
            if radar_candidate.vel_valid and flow_candidate.vel_valid:
                speed_gap = math.hypot(
                    radar_candidate.vel_x_cms / 100.0 - flow_candidate.vel_x_cms / 100.0,
                    radar_candidate.vel_y_cms / 100.0 - flow_candidate.vel_y_cms / 100.0,
                )
                if speed_gap <= self._radar_flow_consistency_speed_threshold:
                    health_state = 'MULTI_SOURCE_AGREE'
                    notes.append('radar_and_flow_consistent')
                else:
                    health_state = 'MULTI_SOURCE_DISAGREE'
                    notes.append('radar_and_flow_disagree')
            elif radar_candidate.vel_valid:
                health_state = 'RADAR_ONLY'
                notes.append('flow_unavailable_or_rejected')
            elif flow_candidate.vel_valid:
                health_state = 'FLOW_ONLY'
                notes.append('radar_unavailable_or_rejected')
            else:
                health_state = 'NO_RELIABLE_SOURCE'
                notes.append('both_sources_invalid')

        if (
            stationary_payload['stationary_flag']
            and not radar_candidate.vel_valid
            and flow_candidate.vel_valid
        ):
            notes.append('imu_static_radar_downgraded_flow_preserved')
        if (
            stationary_payload['stationary_flag']
            and radar_candidate.vel_valid
            and not flow_candidate.vel_valid
        ):
            notes.append('imu_static_flow_downgraded_radar_preserved')

        return {
            'timestamp_ms': int(now_ms),
            'stationary_flag': stationary_payload['stationary_flag'],
            'observation_health_state': health_state,
            'radar_confidence_after_imu': _round_or_none(radar_confidence),
            'flow_confidence': _round_or_none(flow_confidence),
            'radar_valid': radar_candidate.vel_valid,
            'flow_valid': flow_candidate.vel_valid,
            'radar_reject_reason': radar_reject_reason,
            'flow_reject_reason': flow_reject_reason,
            'notes': notes,
        }

    def _publish_flow_twist(self, flow_eval: Dict[str, Any]) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = float(flow_eval['raw_vx'])
        msg.twist.linear.y = float(flow_eval['raw_vy'])
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self._flow_twist_pub.publish(msg)

    def _publish_json(self, publisher, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        publisher.publish(msg)

    def _maybe_log(self, health_payload: Dict[str, Any]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._log_hz)
        if now_ns - self._last_log_ns < min_gap_ns:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            'observation_quality_manager '
            f"state={health_payload['observation_health_state']} "
            f"stationary={health_payload['stationary_flag']} "
            f"radar_conf={health_payload['radar_confidence_after_imu']} "
            f"flow_conf={health_payload['flow_confidence']} "
            f"radar_reason={health_payload['radar_reject_reason']} "
            f"flow_reason={health_payload['flow_reject_reason']}"
        )

    def _make_candidate(
        self,
        *,
        timestamp_ms: int,
        source_name: str,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        valid: bool,
        confidence: float,
        reject_reason: str,
        debug_info: str,
    ) -> ObservationCandidateState:
        return ObservationCandidateState(
            timestamp_ms=int(timestamp_ms),
            source_name=source_name,
            frame_id=self._frame_id,
            pos_x_cm=0,
            pos_y_cm=0,
            pos_z_cm=0,
            vel_x_cms=int(round(vx_mps * 100.0)) if valid else 0,
            vel_y_cms=int(round(vy_mps * 100.0)) if valid else 0,
            vel_z_cms=int(round(vz_mps * 100.0)) if valid else 0,
            dist_direction=0,
            dist_angle_deg=0,
            dist_cm=0,
            pos_valid=False,
            vel_valid=bool(valid),
            dist_valid=False,
            confidence=_clamp(confidence, 0.0, 1.0),
            status='VALID' if valid else 'REJECTED',
            reject_reason=reject_reason,
            debug_info=debug_info,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObservationQualityManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
