import argparse
from pathlib import Path

from ano_receiver_bridge.observation_state import ExternalObservationState
from ano_receiver_bridge.private_protocol import (
    checksum_steps,
    checksum_value_and_bytes,
    encode_external_observation_frame,
    layout_rows,
    packet_to_hex,
    state_to_frame,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Inspect an offline UART3 private observation frame.')
    parser.add_argument('--profile', default='all_valid')
    parser.add_argument('--seq', type=int, default=1)
    parser.add_argument('--remote-tick', type=int, default=1234)
    parser.add_argument('--pos-x-cm', type=int, default=100)
    parser.add_argument('--pos-y-cm', type=int, default=0)
    parser.add_argument('--pos-z-cm', type=int, default=120)
    parser.add_argument('--vel-x-cms', type=int, default=0)
    parser.add_argument('--vel-y-cms', type=int, default=0)
    parser.add_argument('--vel-z-cms', type=int, default=0)
    parser.add_argument('--dist-direction', type=int, default=1)
    parser.add_argument('--dist-angle-deg', type=int, default=270)
    parser.add_argument('--dist-cm', type=int, default=120)
    parser.add_argument('--output-file')
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    state = ExternalObservationState.build_profile(
        profile=args.profile,
        timestamp_ms=args.remote_tick,
        pos_x_cm=args.pos_x_cm,
        pos_y_cm=args.pos_y_cm,
        pos_z_cm=args.pos_z_cm,
        vel_x_cms=args.vel_x_cms,
        vel_y_cms=args.vel_y_cms,
        vel_z_cms=args.vel_z_cms,
        dist_direction=args.dist_direction,
        dist_angle_deg=args.dist_angle_deg,
        dist_cm=args.dist_cm,
    )
    frame = state_to_frame(state, sequence=args.seq)
    packet = encode_external_observation_frame(frame)
    checksum, checksum_hex = checksum_value_and_bytes(packet)

    print('Frame Fields')
    for key, value in frame.__dict__.items():
        print(f'- {key}: {value}')

    print('\nPacket Summary')
    print(f'- total_length: {len(packet)}')
    print(f'- hex: {packet_to_hex(packet)}')
    print(f'- checksum: 0x{checksum:04X} ({checksum_hex})')

    print('\nChecksum Steps')
    for step in checksum_steps(packet[:-2]):
        print(
            f"- offset={step['offset']:02d} byte=0x{step['byte']:02X} "
            f"running_sum=0x{step['running_sum']:04X}"
        )

    print('\nOffset Layout')
    for row in layout_rows(packet):
        print(
            f"- offset={row['offset']:02d} size={row['size']} name={row['name']} "
            f"type={row['type']} endian={row['endian'] or '-'} "
            f"value={row['value']} hex={row['hex']}"
        )

    if args.output_file:
        output_path = Path(args.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(packet)
        print(f'\nWrote binary frame to {output_path}')


if __name__ == '__main__':
    main()
