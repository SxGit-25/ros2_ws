import math
import statistics
from typing import Dict, List, Optional, Sequence, Tuple

from sensor_msgs.msg import LaserScan


def stamp_to_ns(scan: LaserScan) -> int:
    return int(scan.header.stamp.sec) * 1_000_000_000 + int(scan.header.stamp.nanosec)


def normalize_angle_deg(angle_deg: float) -> float:
    normalized = (angle_deg + 180.0) % 360.0 - 180.0
    if normalized == -180.0 and angle_deg > 0.0:
        return 180.0
    return normalized


def safe_median(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return float(statistics.median(values))


def round_or_none(value: Optional[float], digits: int = 4) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def build_sector_defs(sector_mode: str, sector_count: int) -> List[Dict[str, object]]:
    if sector_mode == 'uniform':
        width = 360.0 / float(max(1, sector_count))
        defs: List[Dict[str, object]] = []
        start = -180.0
        for index in range(max(1, sector_count)):
            end = start + width
            defs.append({'name': f'sector_{index}', 'ranges_deg': [(start, end)]})
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


def angle_in_sector(angle_deg: float, sector_ranges: Sequence[Tuple[float, float]]) -> bool:
    for start_deg, end_deg in sector_ranges:
        if start_deg <= angle_deg < end_deg:
            return True
    return False


def scan_to_summary(
    scan: LaserScan,
    sector_mode: str,
    sector_count: int,
    obstacle_distance_threshold_m: float,
) -> Dict[str, object]:
    sector_defs = build_sector_defs(sector_mode, sector_count)
    sector_values: Dict[str, List[float]] = {str(item['name']): [] for item in sector_defs}

    total_points = len(scan.ranges)
    valid_ranges: List[float] = []
    valid_angles_rad: List[float] = []
    valid_points_xy: List[Tuple[float, float]] = []
    valid_indices: List[int] = []
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

        angle_rad = float(scan.angle_min) + index * float(scan.angle_increment)
        angle_deg = normalize_angle_deg(math.degrees(angle_rad))
        range_value = float(raw_range)

        valid_indices.append(index)
        valid_ranges.append(range_value)
        valid_angles_rad.append(angle_rad)
        valid_points_xy.append((range_value * math.cos(angle_rad), range_value * math.sin(angle_rad)))

        for sector in sector_defs:
            if angle_in_sector(angle_deg, sector['ranges_deg']):
                sector_values[str(sector['name'])].append(range_value)
                break

    valid_count = len(valid_ranges)
    valid_ratio = float(valid_count) / float(total_points) if total_points > 0 else 0.0

    sectors_payload: Dict[str, Dict[str, object]] = {}
    sector_medians_raw: Dict[str, Optional[float]] = {}
    for sector in sector_defs:
        name = str(sector['name'])
        values = sector_values[name]
        nearest_sector = min(values) if values else None
        median_sector = safe_median(values)
        sector_medians_raw[name] = median_sector
        sectors_payload[name] = {
            'valid_count': len(values),
            'nearest_m': round_or_none(nearest_sector),
            'median_m': round_or_none(median_sector),
            'has_near_obstacle': bool(
                nearest_sector is not None and nearest_sector <= obstacle_distance_threshold_m
            ),
        }

    return {
        'stamp_ns': stamp_to_ns(scan),
        'frame_id': scan.header.frame_id,
        'total_points': total_points,
        'valid_points': valid_count,
        'valid_ratio': valid_ratio,
        'nan_inf_points': nan_inf_count,
        'below_range_min_points': below_min_count,
        'above_range_max_points': above_max_count,
        'range_min_m': float(scan.range_min),
        'range_max_m': float(scan.range_max),
        'nearest_valid_m': min(valid_ranges) if valid_ranges else None,
        'farthest_valid_m': max(valid_ranges) if valid_ranges else None,
        'median_valid_m': safe_median(valid_ranges),
        'valid_ranges': valid_ranges,
        'valid_angles_rad': valid_angles_rad,
        'valid_points_xy': valid_points_xy,
        'valid_indices': valid_indices,
        'sector_medians_raw': sector_medians_raw,
        'sectors_payload': sectors_payload,
    }
