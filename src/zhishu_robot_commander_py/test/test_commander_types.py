import math

from zhishu_robot_commander_py import Commander
from zhishu_robot_commander_py import CommanderError
from zhishu_robot_commander_py import ErrorCode
from zhishu_robot_commander_py import PoseTarget
from zhishu_robot_commander_py import Result
import pytest


def test_error_codes_match_cpp_protocol():
    assert [code.value for code in ErrorCode] == list(range(10))


def test_pose_target_normalizes_quaternion():
    message = PoseTarget(0.4, 0.2, 0.7, qz=2.0, qw=2.0).to_message()
    assert message.header.frame_id == 'base_link'
    norm = math.sqrt(
        message.pose.orientation.z**2 + message.pose.orientation.w**2
    )
    assert norm == pytest.approx(1.0)


@pytest.mark.parametrize(
    'target',
    [
        PoseTarget(math.nan, 0.0, 0.0),
        PoseTarget(0.0, 0.0, 0.0, qw=0.0),
        PoseTarget(0.0, 0.0, 0.0, frame_id=''),
    ],
)
def test_pose_target_rejects_invalid_values(target):
    with pytest.raises(ValueError):
        target.to_message()


def test_joint_validation_rejects_wrong_count_and_non_finite_values():
    with pytest.raises(ValueError, match='exactly 7'):
        Commander._validated_values([0.0] * 6, 7, 'joints')
    with pytest.raises(ValueError, match='finite'):
        Commander._validated_values([0.0] * 6 + [math.inf], 7, 'joints')


def test_result_boolean_and_exception_api():
    success = Result(True, ErrorCode.SUCCESS, command='test')
    assert success
    assert success.raise_for_error() is success

    failure = Result(
        False,
        ErrorCode.PLANNING_FAILED,
        'no path',
        command='left_arm/pose_command',
        error_name='PLANNING_FAILED',
    )
    assert not failure
    with pytest.raises(CommanderError, match='no path'):
        failure.raise_for_error()
