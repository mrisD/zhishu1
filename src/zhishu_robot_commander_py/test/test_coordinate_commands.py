import threading
from types import SimpleNamespace
import pytest
from zhishu_robot_commander_py import Commander, PoseTarget, Result, ErrorCode


def capture_robot():
    robot = Commander.__new__(Commander)
    robot._cartesian_client = object()
    sent = []
    robot._call_service = lambda client, request, command, timeout: (
        sent.append(request) or (Result(True, ErrorCode.SUCCESS), None))
    return robot, sent


def test_pose_defaults_and_explicit_names_are_per_command():
    robot, sent = capture_robot()
    target = PoseTarget(.1, .2, .3)
    assert robot.move_pose('right', target)
    assert not sent[-1].tcp
    assert sent[-1].right_tool == ''
    assert sent[-1].right_wobj == 'wobj0'
    assert robot.move_pose('right', target, tool='tool1', wobj='wobj1', cartesian=True)
    assert sent[-1].right_tool == 'tool1'
    assert sent[-1].right_wobj == 'wobj1'
    assert sent[-1].cartesian
    assert robot.move_pose('right', target)
    assert sent[-1].right_tool == ''
    assert sent[-1].right_wobj == 'wobj0'


def test_tcp_default_is_distinguished_from_explicit_tool0():
    robot, sent = capture_robot()
    target = PoseTarget(.1, .2, .3)
    assert robot.move_tcp_pose('left', target)
    assert sent[-1].tcp
    assert sent[-1].left_tool == ''  # Server resolves configured gripper TCP.
    assert robot.move_tcp_pose('left', target, tool='tool0')
    assert sent[-1].left_tool == 'tool0'


def test_dual_arm_frames_are_independent():
    robot, sent = capture_robot()
    target = PoseTarget(.1, .2, .3)
    assert robot.move_both_tcp_poses(target, target, left_tool='toolA', right_tool='toolB',
                                     left_wobj='wobjA', right_wobj='wobjB')
    assert sent[-1].group == 'both'
    assert (sent[-1].left_tool, sent[-1].right_tool) == ('toolA', 'toolB')
    assert (sent[-1].left_wobj, sent[-1].right_wobj) == ('wobjA', 'wobjB')


@pytest.mark.parametrize('kwargs', [{'tool': ''}, {'wobj': ''}, {'tool': 1}, {'wobj': None}])
def test_explicit_empty_or_invalid_names_rejected_locally(kwargs):
    robot, sent = capture_robot()
    result = robot.move_tcp_pose('right', PoseTarget(.1, .2, .3), **kwargs)
    assert result.code == ErrorCode.INVALID_ARGUMENT
    assert sent == []


class Future:
    def __init__(self):
        self.callback = None
        self.response = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def finish(self, success=True):
        self.response = SimpleNamespace(success=success, error_code=0 if success else 1,
                                        native_code=0, message='done')
        self.callback(self)


def test_service_timeout_blocks_followup_until_late_result():
    robot = Commander.__new__(Commander)
    robot._closed = False
    robot._condition = threading.Condition()
    robot._pending = None
    robot._last_late_result = None
    robot._default_timeout = .01
    robot._discovery_timeout = .01
    future = Future()
    client = SimpleNamespace(wait_for_service=lambda **kw: True, call_async=lambda req: future)
    result, _ = robot._call_service(client, object(), 'move_cartesian', None)
    assert result.code == ErrorCode.TIMEOUT
    assert robot.busy
    result, _ = robot._call_service(client, object(), 'move_cartesian', None)
    assert result.code == ErrorCode.BUSY
    future.finish()
    assert not robot.busy
    assert robot.last_late_result.success


def test_coordinate_management_paths_and_list():
    robot = Commander.__new__(Commander)
    robot._coordinate_client = object()
    calls = []
    response = SimpleNamespace(frames_yaml='version: 1\n')
    robot._call_service = lambda client, request, *args: (
        calls.append(request) or (Result(True, ErrorCode.SUCCESS), response))
    assert robot.load_coordinate_frames('').code == ErrorCode.INVALID_ARGUMENT
    assert calls == []
    assert robot.load_coordinate_frames('/tmp/manual_frames.yaml')
    assert calls[-1].operation == 'load'
    assert robot.save_coordinate_frames('/tmp/new_frames.yaml')
    result, yaml = robot.get_coordinate_frames()
    assert result
    assert yaml == 'version: 1\n'
    assert calls[-1].operation == 'list'
