import json
import math
import statistics
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


def _stamp_to_ns(scan: LaserScan) -> int:
    return int(scan.header.stamp.sec) * 1_000_000_000 + int(scan.header.stamp.nanosec)


def _normalize_angle_deg(angle_deg: float) -> float:
    normalized = (angle_deg + 180.0) % 360.0 - 180.0
    # Keep 180 on the positive side to simplify the rear sector.
    if normalized == -180.0 and angle_deg > 0.0:
        return 180.0
    return normalized


def _safe_median(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return float(statistics.median(values))


def _round_or_none(value: Optional[float], digits: int = 4) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RadarScanInspectorNode(Node):
    def __init__(self) -> None:
        super().__init__('radar_scan_inspector_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_distance_threshold_m', 1.5)
        self.declare_parameter('sector_mode', 'six')
        self.declare_parameter('sector_count', 6)
        self.declare_parameter('log_hz', 1.0)
        self.declare_parameter('enable_stability_analysis', True)
        self.declare_parameter('min_valid_ratio_threshold', 0.20)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._obstacle_threshold_m = float(
            self.get_parameter('obstacle_distance_threshold_m').value
        )
        self._sector_mode = str(self.get_parameter('sector_mode').value).strip().lower()
        self._sector_count = max(1, int(self.get_parameter('sector_count').value))
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._enable_stability = bool(self.get_parameter('enable_stability_analysis').value)
        self._min_valid_ratio_threshold = float(
            self.get_parameter('min_valid_ratio_threshold').value
        )

        self._previous_summary: Optional[Dict[str, object]] = None
        self._previous_stamp_ns: Optional[int] = None
        self._last_log_time_ns = 0

        self._quality_pub = self.create_publisher(String, '/radar/scan_quality', 10)
        self._sector_pub = self.create_publisher(String, '/radar/sector_metrics', 10)
        self._stability_pub = self.create_publisher(String, '/radar/scan_stability', 10)
        self._stability_score_pub = self.create_publisher(Float32, '/radar/scan_stability_score', 10)

        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, 10)
        self.get_logger().info(
            'Radar scan inspector started '
            f'scan_topic={self._scan_topic} sector_mode={self._sector_mode} '
            f'sector_count={self._sector_count} obstacle_threshold={self._obstacle_threshold_m:.2f}m'
        )

    def _build_sector_defs(self) -> List[Dict[str, object]]:
        if self._sector_mode == 'uniform':
            width = 360.0 / float(self._sector_count)
            defs: List[Dict[str, object]] = []
            start = -180.0
            for index in range(self._sector_count):
                end = start + width
                defs.append(
                    {
                        'name': f'sector_{index}',
                        'ranges_deg': [(start, end)],
                    }
                )
                start = end
            return defs

        return [
            {'name': 'front', 'ranges_deg': [(-30.0, 30.0)]},
            {'name': 'left_front', 'ranges_deg': [(30.0, 90.0)]},
            {'name': 'left', 'ranges_deg': [(90.0, 150.0)]},
            {'name': 'rear', 'ranges_deg': [(150.0, 180.0), (-180.0, -150.0)]},
            {'name': 'right', 'ranges_deg': [(-150.0, -90.0)]},
            {'name': 'right_front', 'ranges_deg': [(-90.0, -30.0)]},
        ]

    def _angle_in_sector(self, angle_deg: float, sector_ranges: Sequence[Tuple[float, float]]) -> bool:
        for start_deg, end_deg in sector_ranges:
            if start_deg <= angle_deg < end_deg:
                return True
        return False

    def _handle_scan(self, scan: LaserScan) -> None:
        stamp_ns = _stamp_to_ns(scan)
        dt_sec = None
        freq_hz = None
        stamp_increasing = True
        if self._previous_stamp_ns is not None:
            delta_ns = stamp_ns - self._previous_stamp_ns
            stamp_increasing = delta_ns > 0
            if delta_ns > 0:
                dt_sec = delta_ns / 1e9
                freq_hz = 1.0 / dt_sec if dt_sec > 0.0 else None
        self._previous_stamp_ns = stamp_ns

        sector_defs = self._build_sector_defs()
        sector_values: Dict[str, List[float]] = {item['name']: [] for item in sector_defs}

        total_points = len(scan.ranges)
        valid_ranges: List[float] = []
        nan_inf_count = 0
        below_min_count = 0
        above_max_count = 0

        for index, raw_range in enumerate(scan.ranges):
            if not math.isfinite(raw_range):
                nan_inf_count += 1
                continue

            if raw_range < scan.range_min:
                below_min_count += 1
                continue

            if raw_range > scan.range_max:
                above_max_count += 1
                continue

            valid_ranges.append(float(raw_range))
            angle_rad = float(scan.angle_min) + index * float(scan.angle_increment)
            angle_deg = _normalize_angle_deg(math.degrees(angle_rad))
            for sector in sector_defs:
                if self._angle_in_sector(angle_deg, sector['ranges_deg']):
                    sector_values[str(sector['name'])].append(float(raw_range))
                    break

        valid_count = len(valid_ranges)
        valid_ratio = float(valid_count) / float(total_points) if total_points > 0 else 0.0
        nearest_m = min(valid_ranges) if valid_ranges else None
        farthest_m = max(valid_ranges) if valid_ranges else None
        median_m = _safe_median(valid_ranges)

        sectors_payload: Dict[str, Dict[str, object]] = {}
        sector_medians_raw: Dict[str, Optional[float]] = {}
        for sector in sector_defs:
            name = str(sector['name'])
            values = sector_values[name]
            nearest_sector = min(values) if values else None
            median_sector = _safe_median(values)
            sector_medians_raw[name] = median_sector
            sectors_payload[name] = {
                'valid_count': len(values),
                'nearest_m': _round_or_none(nearest_sector),
                'median_m': _round_or_none(median_sector),
                'has_near_obstacle': bool(
                    nearest_sector is not None and nearest_sector <= self._obstacle_threshold_m
                ),
            }

        quality_payload = {
            'stamp_ns': stamp_ns,
            'frame_id': scan.header.frame_id,
            'total_points': total_points,
            'valid_points': valid_count,
            'valid_ratio': round(valid_ratio, 4),
            'min_valid_ratio_threshold': round(self._min_valid_ratio_threshold, 4),
            'meets_valid_ratio_threshold': valid_ratio >= self._min_valid_ratio_threshold,
            'nan_inf_points': nan_inf_count,
            'below_range_min_points': below_min_count,
            'above_range_max_points': above_max_count,
            'range_min_m': round(float(scan.range_min), 4),
            'range_max_m': round(float(scan.range_max), 4),
            'nearest_valid_m': _round_or_none(nearest_m),
            'farthest_valid_m': _round_or_none(farthest_m),
            'median_valid_m': _round_or_none(median_m),
            'dt_sec': _round_or_none(dt_sec, 6),
            'estimated_frequency_hz': _round_or_none(freq_hz, 3),
            'timestamp_increasing': stamp_increasing,
            'valid_rule': 'finite and range_min <= r <= range_max',
        }

        sectors_message = {
            'stamp_ns': stamp_ns,
            'sector_mode': self._sector_mode,
            'sector_count': len(sectors_payload),
            'obstacle_distance_threshold_m': round(self._obstacle_threshold_m, 4),
            'sectors': sectors_payload,
        }

        stability_payload = self._build_stability_payload(
            stamp_ns=stamp_ns,
            valid_count=valid_count,
            valid_ratio=valid_ratio,
            nearest_m=nearest_m,
            sector_medians=sector_medians_raw,
        )

        self._publish_string(self._quality_pub, quality_payload)
        self._publish_string(self._sector_pub, sectors_message)
        self._publish_string(self._stability_pub, stability_payload)

        score_msg = Float32()
        score_msg.data = float(stability_payload['overall_difference_score'])
        self._stability_score_pub.publish(score_msg)

        self._maybe_log(quality_payload, sectors_payload, stability_payload)

        self._previous_summary = {
            'valid_count': valid_count,
            'valid_ratio': valid_ratio,
            'nearest_m': nearest_m,
            'sector_medians': sector_medians_raw,
        }

    def _build_stability_payload(
        self,
        stamp_ns: int,
        valid_count: int,
        valid_ratio: float,
        nearest_m: Optional[float],
        sector_medians: Dict[str, Optional[float]],
    ) -> Dict[str, object]:
        if not self._enable_stability:
            return {
                'stamp_ns': stamp_ns,
                'enabled': False,
                'has_previous_frame': self._previous_summary is not None,
                'overall_difference_score': 0.0,
            }

        if self._previous_summary is None:
            return {
                'stamp_ns': stamp_ns,
                'enabled': True,
                'has_previous_frame': False,
                'overall_difference_score': 0.0,
            }

        previous_valid_count = int(self._previous_summary['valid_count'])
        previous_valid_ratio = float(self._previous_summary['valid_ratio'])
        previous_nearest_m = self._previous_summary['nearest_m']
        previous_sector_medians = self._previous_summary['sector_medians']

        valid_count_delta = valid_count - previous_valid_count
        valid_ratio_delta = valid_ratio - previous_valid_ratio

        nearest_delta_m = None
        if nearest_m is not None and previous_nearest_m is not None:
            nearest_delta_m = nearest_m - float(previous_nearest_m)

        sector_median_deltas: Dict[str, Optional[float]] = {}
        comparable_sector_deltas: List[float] = []
        for name, current_value in sector_medians.items():
            previous_value = previous_sector_medians.get(name)
            if current_value is None or previous_value is None:
                sector_median_deltas[name] = None
                continue
            delta = float(current_value) - float(previous_value)
            sector_median_deltas[name] = _round_or_none(delta)
            comparable_sector_deltas.append(abs(delta))

        mean_sector_delta_m = (
            sum(comparable_sector_deltas) / len(comparable_sector_deltas)
            if comparable_sector_deltas
            else None
        )
        max_sector_delta_m = max(comparable_sector_deltas) if comparable_sector_deltas else None

        score = (
            0.40 * _clamp(abs(valid_ratio_delta), 0.0, 1.0)
            + 0.30
            * _clamp(abs(nearest_delta_m) if nearest_delta_m is not None else 1.0, 0.0, 1.0)
            + 0.30 * _clamp(mean_sector_delta_m if mean_sector_delta_m is not None else 1.0, 0.0, 1.0)
        )

        return {
            'stamp_ns': stamp_ns,
            'enabled': True,
            'has_previous_frame': True,
            'valid_count_delta': valid_count_delta,
            'valid_ratio_delta': _round_or_none(valid_ratio_delta),
            'nearest_delta_m': _round_or_none(nearest_delta_m),
            'sector_median_deltas_m': sector_median_deltas,
            'mean_sector_median_delta_m': _round_or_none(mean_sector_delta_m),
            'max_sector_median_delta_m': _round_or_none(max_sector_delta_m),
            'overall_difference_score': round(score, 4),
        }

    def _publish_string(self, publisher, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, ensure_ascii=True)
        publisher.publish(msg)

    def _maybe_log(
        self,
        quality_payload: Dict[str, object],
        sectors_payload: Dict[str, Dict[str, object]],
        stability_payload: Dict[str, object],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        min_gap_ns = int(1e9 / self._log_hz)
        if now_ns - self._last_log_time_ns < min_gap_ns:
            return
        self._last_log_time_ns = now_ns

        front_sector = sectors_payload.get('front')
        front_summary = ''
        if front_sector is not None:
            front_summary = (
                f" front_nearest={front_sector['nearest_m']}m"
                f" front_median={front_sector['median_m']}m"
                f" front_obstacle={front_sector['has_near_obstacle']}"
            )

        self.get_logger().info(
            'scan_quality '
            f"total={quality_payload['total_points']} valid={quality_payload['valid_points']} "
            f"ratio={quality_payload['valid_ratio']:.3f} "
            f"nan_inf={quality_payload['nan_inf_points']} "
            f"below={quality_payload['below_range_min_points']} "
            f"above={quality_payload['above_range_max_points']} "
            f"nearest={quality_payload['nearest_valid_m']}m "
            f"median={quality_payload['median_valid_m']}m "
            f"dt={quality_payload['dt_sec']}s "
            f"freq={quality_payload['estimated_frequency_hz']}Hz "
            f"stamp_ok={quality_payload['timestamp_increasing']}"
            f'{front_summary} '
            f"stability={stability_payload['overall_difference_score']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarScanInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
