"""Safe, opt-in example for the Python torso/head Commander client."""

import argparse
from typing import Optional, Sequence

from .commander import Commander


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group()
    target.add_argument('--named', help='SRDF torso_head named target')
    target.add_argument(
        '--joints', nargs=3, type=float, metavar=('WAIST', 'YAW', 'PITCH'),
        help='joint target in radians: waist_pitch head_yaw head_pitch',
    )
    parser.add_argument(
        '--execute', action='store_true',
        help='actually send the selected motion command; omitted by default',
    )
    args = parser.parse_args(argv)
    if args.execute != (args.named is not None or args.joints is not None):
        parser.error('specify both --execute and exactly one of --named or --joints')

    with Commander() as robot:
        if not robot.wait_until_ready(10.0):
            print('Commander bridge is not ready; start bringup with enable_commander:=true.')
            return 2
        if not args.execute:
            print('Commander bridge is ready. No motion sent; use --execute with a target.')
            return 0

        result = (
            robot.move_torso_head_named(args.named)
            if args.named is not None
            else robot.move_torso_head_joints(args.joints)
        )
        if not result:
            print(f'Torso/head move failed [{result.error_name}]: {result.message}')
            return 1
        print(f'Torso/head move succeeded: {result.message}')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
