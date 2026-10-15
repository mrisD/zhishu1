import math
from types import SimpleNamespace
import pytest
from zhishu_robot_commander_py import Commander, PoseTarget, Offs, RelTool, Result, ErrorCode
from zhishu_robot_commander_py.pose_math import _multiply


def point():
    return PoseTarget(.1, .2, .3, qz=math.sqrt(.5), qw=math.sqrt(.5), frame_id='wobj_test')


def test_offs_uses_object_axes_without_changing_orientation_or_source():
    p = point()
    out = Offs(p, .01, -.02, .03)
    assert (out.x, out.y, out.z) == pytest.approx((.11, .18, .33))
    assert (out.qz, out.qw, out.frame_id) == (p.qz, p.qw, p.frame_id)
    assert p.x == .1


def test_reltool_translates_before_rotation_in_original_tool_axes():
    out = RelTool(point(), .01, 0, 0, rz_rad=math.pi/2)
    assert (out.x, out.y, out.z) == pytest.approx((.1, .21, .3))
    assert abs(out.qz) == pytest.approx(1.)


def test_combined_rotation_matches_intrinsic_xyz_not_fixed_rpy():
    p = point()
    out = RelTool(p, 0, 0, 0, .2, -.3, .4)
    qx = (math.sin(.1), 0, 0, math.cos(.1))
    qy = (0, math.sin(-.15), 0, math.cos(-.15))
    qz = (0, 0, math.sin(.2), math.cos(.2))
    expected = _multiply(_multiply(_multiply((p.qx, p.qy, p.qz, p.qw), qx), qy), qz)
    assert (out.qx, out.qy, out.qz, out.qw) == pytest.approx(expected)
    reversed_order = _multiply(_multiply(_multiply((p.qx, p.qy, p.qz, p.qw), qz), qy), qx)
    assert (out.qx, out.qy, out.qz, out.qw) != pytest.approx(reversed_order)


def test_pose_stamped_metadata_and_no_mutation():
    p = point().to_message()
    p.header.stamp.sec = 42
    out = Commander.Offs(Commander.RelTool(p, .01, 0, 0), 0, 0, .02)
    assert out.header == p.header
    assert out.pose.position.y == pytest.approx(.21)
    assert out.pose.position.z == pytest.approx(.32)
    assert p.pose.position.y == .2


@pytest.mark.parametrize('helper,args', [(Offs, (math.nan, 0, 0)),
                                       (RelTool, (0, 0, 0, math.inf))])
def test_bad_offsets_fail_without_motion(helper, args):
    with pytest.raises(ValueError):
        helper(point(), *args)


def test_large_finite_quaternion_is_normalized_without_overflow():
    p = PoseTarget(.1, .2, .3, qz=1e300, qw=1e300)
    out = RelTool(p, .01, 0, 0)
    assert (out.x, out.y) == pytest.approx((.1, .21))


def test_movej_and_movel_dispatch_and_tool0_default():
    robot = Commander.__new__(Commander)
    calls = []
    robot.move_pose = lambda group, target, **kw: (
        calls.append((group, target, kw)) or Result(True, ErrorCode.SUCCESS))
    p = point()
    assert robot.MoveJ('right', p)
    assert calls[-1][2]['tool'] == 'tool0'
    assert calls[-1][2]['wobj'] == 'wobj0'
    assert calls[-1][2]['cartesian'] is False
    assert robot.MoveL('left', p, tool='tool1', wobj='wobj1')
    assert calls[-1][2]['tool'] == 'tool1'
    assert calls[-1][2]['wobj'] == 'wobj1'
    assert calls[-1][2]['cartesian'] is True


def test_moveabsj_preserves_radians_and_checks_names_and_counts():
    robot = Commander.__new__(Commander)
    robot._abs_joint_client = object()
    calls = []
    robot._call_service = lambda client, request, *args: (
        calls.append(request) or (Result(True, ErrorCode.SUCCESS), None))
    joints = [.1, .2, -.3, .4, -.5, .6, -.7]
    assert robot.MoveAbsJ('right', joints, tool='tool1', wobj='wobj1')
    assert list(calls[-1].joints) == joints
    assert (calls[-1].tool, calls[-1].wobj) == ('tool1', 'wobj1')
    assert robot.MoveAbsJ('both', joints*2)
    assert robot.MoveAbsJ('right', joints[:6]).code == ErrorCode.INVALID_ARGUMENT
    assert robot.MoveAbsJ('right', [math.nan]*7).code == ErrorCode.INVALID_ARGUMENT
    assert robot.MoveAbsJ('right', joints, tool='').code == ErrorCode.INVALID_ARGUMENT
    assert robot.MoveAbsJ('wrong', joints).code == ErrorCode.INVALID_ARGUMENT
    assert len(calls) == 2
