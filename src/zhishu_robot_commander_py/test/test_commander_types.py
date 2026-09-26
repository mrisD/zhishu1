import math

from zhishu_robot_commander_py import Commander
from zhishu_robot_commander_py import CommanderError
from zhishu_robot_commander_py import ErrorCode
from zhishu_robot_commander_py import PoseTarget
from zhishu_robot_commander_py import Result
from zhishu_robot_commander_py import torso_head_example
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


def test_torso_head_commands_use_bridge_topics_and_fixed_joint_order():
    robot = Commander.__new__(Commander)
    sent = []
    robot._send = lambda command, message, **kwargs: (
        sent.append((command, message, kwargs)) or Result(True, ErrorCode.SUCCESS)
    )

    assert robot.move_torso_head_joints([0.1, 0.2, -0.3])
    assert sent[0][0] == 'torso_head/joint_command'
    assert list(sent[0][1].data) == pytest.approx([0.1, 0.2, -0.3])
    assert robot.move_torso_head_named(' torso_head_origin ')
    assert sent[1][0] == 'torso_head/named_target'
    assert sent[1][1].data == 'torso_head_origin'

    assert robot.move_torso_head_joints([0.0, 0.0]).code == ErrorCode.INVALID_ARGUMENT
    assert robot.move_torso_head_joints([0.0, math.nan, 0.0]).code == ErrorCode.INVALID_ARGUMENT
    assert robot.move_torso_head_named(' ').code == ErrorCode.INVALID_ARGUMENT
    assert len(sent) == 2


def test_torso_head_example_does_not_move_without_explicit_opt_in(monkeypatch):
    class FakeCommander:
        def __init__(self):
            self.commands = []

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def wait_until_ready(self, _timeout):
            return True

        def move_torso_head_joints(self, joints):
            self.commands.append(list(joints))
            return Result(True, ErrorCode.SUCCESS)

    fake = FakeCommander()
    monkeypatch.setattr(torso_head_example, 'Commander', lambda: fake)

    assert torso_head_example.main([]) == 0
    assert fake.commands == []
    with pytest.raises(SystemExit) as error:
        torso_head_example.main(['--joints', '0', '0.1', '-0.1'])
    assert error.value.code == 2
    assert fake.commands == []

    assert torso_head_example.main(['--execute', '--joints', '0', '0.1', '-0.1']) == 0
    assert fake.commands == [[0.0, 0.1, -0.1]]
