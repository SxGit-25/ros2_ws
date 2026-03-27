import math
import statistics
from typing import Dict, List, Optional, Tuple


def _rotate_point(x: float, y: float, yaw_rad: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def _safe_median(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return float(statistics.median(values))


def match_scan_summaries(
    previous_summary: Dict[str, object],
    current_summary: Dict[str, object],
    *,
    angle_increment_rad: float,
    max_yaw_search_deg: float,
    yaw_search_step_deg: float,
    max_translation_step_m: float,
) -> Dict[str, object]:
    if angle_increment_rad <= 0.0:
        return {'success': False, 'reason': 'invalid_angle_increment'}

    prev_ranges = list(previous_summary['valid_ranges'])
    curr_ranges = list(current_summary['valid_ranges'])
    prev_points = list(previous_summary['valid_points_xy'])
    curr_points = list(current_summary['valid_points_xy'])

    sample_count = min(len(prev_ranges), len(curr_ranges))
    if sample_count < 20:
        return {'success': False, 'reason': 'too_few_comparable_points'}

    max_shift_bins = int(round(math.radians(max_yaw_search_deg) / angle_increment_rad))
    step_bins = max(1, int(round(math.radians(max(yaw_search_step_deg, 0.1)) / angle_increment_rad)))

    best_result: Optional[Dict[str, object]] = None
    for shift_bins in range(-max_shift_bins, max_shift_bins + 1, step_bins):
        residuals: List[float] = []
        diffs_x: List[float] = []
        diffs_y: List[float] = []
        comparable_points = 0
        yaw_rad = shift_bins * angle_increment_rad

        start_prev = max(0, shift_bins)
        start_curr = max(0, -shift_bins)
        overlap = min(len(prev_ranges) - start_prev, len(curr_ranges) - start_curr)
        if overlap <= 0:
            continue

        for offset in range(overlap):
            prev_index = start_prev + offset
            curr_index = start_curr + offset

            prev_range = prev_ranges[prev_index]
            curr_range = curr_ranges[curr_index]
            residuals.append(abs(prev_range - curr_range))

            prev_x, prev_y = prev_points[prev_index]
            curr_x, curr_y = curr_points[curr_index]
            prev_rot_x, prev_rot_y = _rotate_point(prev_x, prev_y, -yaw_rad)
            diffs_x.append(prev_rot_x - curr_x)
            diffs_y.append(prev_rot_y - curr_y)
            comparable_points += 1

        if comparable_points == 0:
            continue

        overlap_ratio = float(comparable_points) / float(max(len(prev_ranges), len(curr_ranges)))
        median_residual_m = _safe_median(residuals)
        dx_m = _safe_median(diffs_x)
        dy_m = _safe_median(diffs_y)

        if median_residual_m is None or dx_m is None or dy_m is None:
            continue

        translation_norm = math.hypot(dx_m, dy_m)
        translation_penalty = min(1.0, translation_norm / max(max_translation_step_m, 1e-3))
        residual_penalty = min(1.0, median_residual_m / 1.0)
        quality = max(
            0.0,
            1.0
            - 0.55 * residual_penalty
            - 0.25 * translation_penalty
            - 0.20 * max(0.0, 1.0 - overlap_ratio),
        )

        candidate = {
            'success': True,
            'shift_bins': shift_bins,
            'delta_yaw_rad': yaw_rad,
            'delta_yaw_deg': math.degrees(yaw_rad),
            'delta_x_m': dx_m,
            'delta_y_m': dy_m,
            'translation_norm_m': translation_norm,
            'overlap_ratio': overlap_ratio,
            'median_residual_m': median_residual_m,
            'quality': quality,
            'comparable_points': comparable_points,
        }

        if best_result is None or candidate['quality'] > best_result['quality']:
            best_result = candidate

    if best_result is None:
        return {'success': False, 'reason': 'no_candidate'}
    return best_result
