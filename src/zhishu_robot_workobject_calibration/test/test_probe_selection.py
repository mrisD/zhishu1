"""Check simulation-only probe selection and preserve the legacy both default."""
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


@unittest.skipIf(get_package_share_directory is None, 'Requires installed ROS workspace')
class ProbeSelectionTests(unittest.TestCase):
    def robot(self, arm=None):
        moveit = Path(get_package_share_directory('zhishu_robot_moveit_config'))
        tcp = Path(get_package_share_directory('zhishu_robot_tcp_calibration'))
        command = ['xacro', str(moveit/'config/zhishu_robot.urdf.xacro'),
                   'use_gazebo:=true', 'use_fake_hardware:=false', 'tcp_calibration_enabled:=true',
                   'initial_positions_file:='+str(tcp/'config/tcp_initial_positions.yaml')]
        if arm is not None:
            command.append('tcp_calibration_arm:='+arm)
        result = subprocess.run(command, text=True, capture_output=True, check=True, timeout=30)
        return ET.fromstring(result.stdout)

    def test_legacy_default_both(self):
        robot = self.robot()
        for side in ('left', 'right'):
            self.assertIsNotNone(robot.find("link[@name='%s_calibration_probe']" % side))

    def test_right_only(self):
        robot = self.robot('right')
        self.assertIsNone(robot.find("link[@name='left_calibration_probe']"))
        probe = robot.find("link[@name='right_calibration_probe']")
        self.assertIsNotNone(probe)
        self.assertEqual(len(probe.findall('collision')), 2)
        self.assertIsNotNone(robot.find("joint[@name='tcp_calibration_world_anchor']"))
        self.assertIn('gz_ros2_control/GazeboSimSystem', ET.tostring(robot, encoding='unicode'))

    def test_left_only(self):
        robot = self.robot('left')
        self.assertIsNone(robot.find("link[@name='right_calibration_probe']"))
        self.assertIsNotNone(robot.find("link[@name='left_calibration_probe']"))


if __name__ == '__main__':
    unittest.main()
