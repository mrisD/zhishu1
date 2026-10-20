"""Regressions for Gazebo removal acknowledgements and physical clearance."""
import unittest

from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.boolean_pb2 import Boolean
from run_pallet_nine_continuous import clear_receiving_bin


class FakeTransport:
    def __init__(self, remove_ack=True, remove_effect=True, initially_present=True):
        self.present = initially_present
        self.remove_ack = remove_ack
        self.remove_effect = remove_effect
        self.removals = []

    def request(self, service, request, *_):
        if service.endswith('/scene/info'):
            scene = Scene()
            scene.model.add(name='pallet_box_r2_c0', id=43)
            if self.present:
                scene.model.add(name='pallet_box_r0_c2', id=42)
            return True, scene
        self.removals.append((request.name, request.id))
        if self.remove_effect:
            self.present = False
        return self.remove_ack, Boolean(data=self.remove_ack)


class ClearanceTests(unittest.TestCase):
    def clear(self, node):
        clear_receiving_bin(node, 'pallet_box_r0_c2', lambda _: None, sleep=lambda _: None)

    def test_timed_out_ack_with_actual_removal_succeeds(self):
        node = FakeTransport(remove_ack=False)
        self.clear(node)
        self.assertEqual(node.removals, [('pallet_box_r0_c2', 42)])

    def test_positive_ack_without_actual_removal_is_rejected(self):
        node = FakeTransport(remove_effect=False)
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            self.clear(node)
        self.assertEqual(len(node.removals), 3)

    def test_already_absent_is_idempotent_without_removing_other_box(self):
        node = FakeTransport(initially_present=False)
        self.clear(node)
        self.assertEqual(node.removals, [])


if __name__ == '__main__':
    unittest.main()
