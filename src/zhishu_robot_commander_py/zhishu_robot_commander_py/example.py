"""Minimal executable customer example."""

from .commander import ArmGroup, Commander, GripperGroup


def main() -> int:
    with Commander() as robot:
        if not robot.wait_until_ready():
            print('Commander bridge is not ready; start the robot bringup first.')
            return 2

        result = robot.move_named(ArmGroup.BOTH, 'both_arms_home')
        if not result:
            print(f'Move failed [{result.error_name}]: {result.message}')
            return 1

        result = robot.open_gripper(GripperGroup.BOTH)
        if not result:
            print(f'Gripper failed [{result.error_name}]: {result.message}')
            return 1

        print('Robot reached both_arms_home and both grippers are open.')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
