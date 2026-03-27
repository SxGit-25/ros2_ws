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

        quality = float(match_result['quality'])
        if quality < low_confidence_quality:
            reasons.append('quality_too_low')
        elif quality < min_quality:
            warnings.append('quality_low_confidence')

        jumps = []
        current_sectors = current_summary['sector_medians_raw']
        previous_sectors = previous_summary['sector_medians_raw']
        for name, current_median in current_sectors.items():
            previous_median = previous_sectors.get(name)
            if current_median is None or previous_median is None:
                continue
            jumps.append(abs(float(current_median) - float(previous_median)))
        if jumps and max(jumps) > sector_median_jump_threshold_m:
            warnings.append('sector_jump_high')

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
