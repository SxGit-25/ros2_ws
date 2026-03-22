from dataclasses import dataclass


@dataclass(frozen=True)
class ExternalObservationState:
    timestamp_ms: int
    pos_x_cm: int
    pos_y_cm: int
    pos_z_cm: int
    vel_x_cms: int
    vel_y_cms: int
    vel_z_cms: int
    dist_direction: int
    dist_angle_deg: int
    dist_cm: int
    pos_valid: bool
    vel_valid: bool
    dist_valid: bool
    source_name: str = 'mock'
    frame_id: str = 'body'
    debug_info: str = ''

    @staticmethod
    def build_profile(
        profile: str,
        timestamp_ms: int,
        pos_x_cm: int,
        pos_y_cm: int,
        pos_z_cm: int,
        vel_x_cms: int,
        vel_y_cms: int,
        vel_z_cms: int,
        dist_direction: int,
        dist_angle_deg: int,
        dist_cm: int,
    ) -> 'ExternalObservationState':
        profile_name = profile.strip().lower()

        if profile_name == 'all_invalid':
            return ExternalObservationState(
                timestamp_ms=timestamp_ms,
                pos_x_cm=pos_x_cm,
                pos_y_cm=pos_y_cm,
                pos_z_cm=pos_z_cm,
                vel_x_cms=vel_x_cms,
                vel_y_cms=vel_y_cms,
                vel_z_cms=vel_z_cms,
                dist_direction=dist_direction,
                dist_angle_deg=dist_angle_deg,
                dist_cm=dist_cm,
                pos_valid=False,
                vel_valid=False,
                dist_valid=False,
                source_name='profile:all_invalid',
                debug_info='All fields invalid',
            )

        if profile_name == 'dist_only':
            return ExternalObservationState(
                timestamp_ms=timestamp_ms,
                pos_x_cm=pos_x_cm,
                pos_y_cm=pos_y_cm,
                pos_z_cm=pos_z_cm,
                vel_x_cms=vel_x_cms,
                vel_y_cms=vel_y_cms,
                vel_z_cms=vel_z_cms,
                dist_direction=dist_direction,
                dist_angle_deg=dist_angle_deg,
                dist_cm=dist_cm,
                pos_valid=False,
                vel_valid=False,
                dist_valid=True,
                source_name='profile:dist_only',
                debug_info='Only distance valid',
            )

        if profile_name == 'pos_only':
            return ExternalObservationState(
                timestamp_ms=timestamp_ms,
                pos_x_cm=pos_x_cm,
                pos_y_cm=pos_y_cm,
                pos_z_cm=pos_z_cm,
                vel_x_cms=vel_x_cms,
                vel_y_cms=vel_y_cms,
                vel_z_cms=vel_z_cms,
                dist_direction=dist_direction,
                dist_angle_deg=dist_angle_deg,
                dist_cm=dist_cm,
                pos_valid=True,
                vel_valid=False,
                dist_valid=False,
                source_name='profile:pos_only',
                debug_info='Only position valid',
            )

        return ExternalObservationState(
            timestamp_ms=timestamp_ms,
            pos_x_cm=pos_x_cm,
            pos_y_cm=pos_y_cm,
            pos_z_cm=pos_z_cm,
            vel_x_cms=vel_x_cms,
            vel_y_cms=vel_y_cms,
            vel_z_cms=vel_z_cms,
            dist_direction=dist_direction,
            dist_angle_deg=dist_angle_deg,
            dist_cm=dist_cm,
            pos_valid=True,
            vel_valid=True,
            dist_valid=True,
            source_name='profile:all_valid',
            debug_info='All fields valid',
        )
