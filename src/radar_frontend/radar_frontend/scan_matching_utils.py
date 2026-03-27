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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _compute_sector_jump_metrics(
    previous_summary: Dict[str, object],
    current_summary: Dict[str, object],
) -> Tuple[Optional[float], Optional[float], float]:
    jumps: List[float] = []
    previous_sectors = previous_summary['sector_medians_raw']
    current_sectors = current_summary['sector_medians_raw']

    for name, current_value in current_sectors.items():
        previous_value = previous_sectors.get(name)
        if current_value is None or previous_value is None:
            continue
        jumps.append(abs(float(current_value) - float(previous_value)))

    if not jumps:
        return None, None, 0.2

    mean_jump = sum(jumps) / len(jumps)
    max_jump = max(jumps)
    sector_consistency_score = math.exp(-mean_jump / 0.20)
    return mean_jump, max_jump, sector_consistency_score


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

    mean_sector_jump_m, max_sector_jump_m, sector_consistency_score = _compute_sector_jump_metrics(
        previous_summary,
        current_summary,
    )

    candidates: List[Dict[str, object]] = []
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
        inlier_residual_threshold_m = 0.12
        inlier_ratio = (
            float(sum(1 for residual in residuals if residual <= inlier_residual_threshold_m))
            / float(len(residuals))
            if residuals
            else 0.0
        )
        edge_ratio = (
            abs(float(shift_bins)) / float(max_shift_bins)
            if max_shift_bins > 0
            else 0.0
        )
        overlap_score = overlap_ratio ** 1.15
        residual_score = math.exp(-median_residual_m / 0.08)
        translation_score = math.exp(
            -translation_norm / max(max_translation_step_m * 0.45, 1e-3)
        )
        edge_score = max(0.15, 1.0 - 0.75 * edge_ratio)
        quality = _clamp(
            overlap_score
            * residual_score
            * max(0.05, inlier_ratio)
            * max(0.10, translation_score)
            * max(0.15, sector_consistency_score)
            * edge_score,
            0.0,
            1.0,
        )

        candidates.append(
            {
                'success': True,
                'shift_bins': shift_bins,
                'delta_yaw_rad': yaw_rad,
                'delta_yaw_deg': math.degrees(yaw_rad),
                'delta_x_m': dx_m,
                'delta_y_m': dy_m,
                'translation_norm_m': translation_norm,
                'overlap_ratio': overlap_ratio,
                'median_residual_m': median_residual_m,
                'inlier_ratio': inlier_ratio,
                'edge_ratio': edge_ratio,
                'mean_sector_jump_m': mean_sector_jump_m,
                'max_sector_jump_m': max_sector_jump_m,
                'sector_consistency_score': sector_consistency_score,
                'quality': quality,
                'comparable_points': comparable_points,
            }
        )

    if not candidates:
        return {'success': False, 'reason': 'no_candidate'}

    best_index = max(range(len(candidates)), key=lambda index: candidates[index]['quality'])
    best_result = dict(candidates[best_index])

    refined_shift_bins = float(best_result['shift_bins'])
    if 0 < best_index < len(candidates) - 1:
        left_quality = float(candidates[best_index - 1]['quality'])
        center_quality = float(candidates[best_index]['quality'])
        right_quality = float(candidates[best_index + 1]['quality'])
        denominator = left_quality - 2.0 * center_quality + right_quality
        if abs(denominator) > 1e-9:
            offset = 0.5 * (left_quality - right_quality) / denominator
            offset = _clamp(offset, -1.0, 1.0)
            refined_shift_bins += offset * step_bins

    refined_yaw_rad = refined_shift_bins * angle_increment_rad
    prev_rotated_x: List[float] = []
    prev_rotated_y: List[float] = []
    curr_matched_x: List[float] = []
    curr_matched_y: List[float] = []
    best_shift_bins_int = int(best_result['shift_bins'])
    start_prev = max(0, best_shift_bins_int)
    start_curr = max(0, -best_shift_bins_int)
    overlap = min(len(prev_points) - start_prev, len(curr_points) - start_curr)
    for offset in range(max(0, overlap)):
        prev_index = start_prev + offset
        curr_index = start_curr + offset
        prev_x, prev_y = prev_points[prev_index]
        curr_x, curr_y = curr_points[curr_index]
        prev_rot_x, prev_rot_y = _rotate_point(prev_x, prev_y, -refined_yaw_rad)
        prev_rotated_x.append(prev_rot_x)
        prev_rotated_y.append(prev_rot_y)
        curr_matched_x.append(curr_x)
        curr_matched_y.append(curr_y)

    refined_dx_m = None
    refined_dy_m = None
    if prev_rotated_x and curr_matched_x:
        refined_dx_m = _safe_median(
            [prev_rotated_x[index] - curr_matched_x[index] for index in range(len(curr_matched_x))]
        )
        refined_dy_m = _safe_median(
            [prev_rotated_y[index] - curr_matched_y[index] for index in range(len(curr_matched_y))]
        )

    if refined_dx_m is not None and refined_dy_m is not None:
        best_result['delta_x_m'] = refined_dx_m
        best_result['delta_y_m'] = refined_dy_m
        best_result['translation_norm_m'] = math.hypot(refined_dx_m, refined_dy_m)

    best_result['refined_shift_bins'] = refined_shift_bins
    best_result['delta_yaw_rad'] = refined_yaw_rad
    best_result['delta_yaw_deg'] = math.degrees(refined_yaw_rad)
    best_result['search_step_bins'] = step_bins
    best_result['search_window_bins'] = max_shift_bins
    best_result['search_edge_hit'] = bool(best_result['edge_ratio'] >= 0.95)
    return best_result
