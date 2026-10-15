"""Exercise the real fixture request handler; requires ROS Python messages."""
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

try:
    from geometry_msgs.msg import PoseStamped
    module_path = Path(__file__).parents[1] / 'scripts/tcp_fixture_spawner.py'
    spec = importlib.util.spec_from_file_location('fixture_protocol_subject', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
except ModuleNotFoundError:
    module = None


@unittest.skipIf(module is None, 'ROS Python messages are unavailable')
class FixtureProtocol(unittest.TestCase):
    def fixture(self):
        node = object.__new__(module.FixtureSpawner)
        node.lock = threading.Lock()
        node.spawning = True
        node.last_stamp = None
        node.publisher = Mock()
        node.pose_publisher = Mock()
        node.get_logger = Mock(return_value=Mock())
        return node

    def message(self, seconds):
        pose = PoseStamped()
        pose.header.stamp.sec = seconds
        pose.header.frame_id = 'base_link'
        return pose

    def test_success_acknowledges_the_exact_request(self):
        node = self.fixture()
        node.spawn = Mock(return_value=True)
        request = self.message(31)
        node.run_request(request)
        node.pose_publisher.publish.assert_called_once_with(request)
        self.assertEqual(node.last_stamp, (31, 0))
        self.assertFalse(node.spawning)

    def test_failure_never_reuses_a_previous_success_ack(self):
        node = self.fixture()
        node.last_stamp = (30, 0)
        node.spawn = Mock(return_value=False)
        node.run_request(self.message(31))
        node.pose_publisher.publish.assert_not_called()
        self.assertEqual(node.last_stamp, (30, 0))
        self.assertFalse(node.spawning)
        self.assertFalse(node.publisher.publish.call_args.args[0].data)

    def test_failed_request_can_be_retried(self):
        node = self.fixture()
        node.spawn = Mock(side_effect=[RuntimeError('creation failed'), True])
        request = self.message(31)
        node.run_request(request)
        self.assertFalse(node.spawning)
        node.run_request(request)
        node.pose_publisher.publish.assert_called_once_with(request)
        self.assertEqual(node.last_stamp, (31, 0))


if __name__ == '__main__':
    unittest.main()
