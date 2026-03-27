from typing import Dict, List


STATUS_VALID = 'VALID'
STATUS_LOW_CONFIDENCE = 'LOW_CONFIDENCE'
STATUS_INVALID = 'INVALID'


def evaluate_odometry_candidate(
    *,
    dt_sec: float,
    timestamp_increasing: bool,
    current_summary: Dict[str, object],
    previous_summary: Dict[str, object],
    match_result: Dict[str, object],
    min_valid_points: int,
    min_valid_ratio: float,
    min_overlap_ratio: float,
    max_median_residual_m: float,
    max_translation_norm_m: float,
    min_quality: float,
    low_confidence_quality: float,
    max_dt_sec: float,
    min_dt_sec: float,
    sector_median_jump_threshold_m: float,
    min_inlier_ratio: float,
    search_edge_ratio_limit: float,
    high_confidence_quality: float,
    static_translation_deadband_m: float,
    static_yaw_deadband_deg: float,
) -> Dict[str, object]:
    reasons: List[str] = []
    warnings: List[str] = []

    if not timestamp_increasing:
        reasons.append('timestamp_not_increasing')
    if dt_sec <= 0.0:
        reasons.append('non_positive_dt')
    if dt_sec < min_dt_sec:
        reasons.append('dt_too_small')
    if dt_sec > max_dt_sec:
        reasons.append('dt_too_large')

    if int(current_summary['valid_points']) < min_valid_points:
        reasons.append('current_valid_points_too_low')
    if int(previous_summary['valid_points']) < min_valid_points:
        reasons.append('previous_valid_points_too_low')

    if float(current_summary['valid_ratio']) < min_valid_ratio:
        reasons.append('current_valid_ratio_too_low')
    if float(previous_summary['valid_ratio']) < min_valid_ratio:
        reasons.append('previous_valid_ratio_too_low')

    if not bool(match_result.get('success', False)):
        reasons.append(str(match_result.get('reason', 'match_failed')))
    else:
        if float(match_result['overlap_ratio']) < min_overlap_ratio:
            reasons.append('overlap_ratio_too_low')
        if float(match_result['median_residual_m']) > max_median_residual_m:
            reasons.append('median_residual_too_high')
        if float(match_result['translation_norm_m']) > max_translation_norm_m:
            reasons.append('translation_too_large')
        if float(match_result.get('inlier_ratio', 0.0)) < min_inlier_ratio:
            reasons.append('inlier_ratio_too_low')

        quality = float(match_result['quality'])
        if quality < low_confidence_quality:
            reasons.append('quality_too_low')
        elif quality < min_quality:
            warnings.append('quality_low_confidence')

        max_sector_jump_m = float(match_result.get('max_sector_jump_m') or 0.0)
        if max_sector_jump_m > sector_median_jump_threshold_m:
            warnings.append('sector_jump_high')
            if quality < high_confidence_quality:
                reasons.append('sector_jump_with_insufficient_quality')

        edge_ratio = float(match_result.get('edge_ratio', 0.0))
        if edge_ratio >= search_edge_ratio_limit:
            warnings.append('search_result_near_edge')
            if quality < high_confidence_quality:
                reasons.append('search_result_near_edge')

        delta_yaw_deg = abs(float(match_result.get('delta_yaw_deg', 0.0)))
        translation_norm_m = float(match_result.get('translation_norm_m', 0.0))
        if (
            translation_norm_m <= static_translation_deadband_m
            and delta_yaw_deg <= static_yaw_deadband_deg
            and quality < high_confidence_quality
        ):
            warnings.append('inside_static_deadband')

    if reasons:
        status = STATUS_INVALID
    elif warnings:
        status = STATUS_LOW_CONFIDENCE
    else:
        status = STATUS_VALID

    return {
        'status': status,
        'reasons': reasons,
        'warnings': warnings,
        'accept_motion': status == STATUS_VALID,
    }
