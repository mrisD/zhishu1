import unittest

from zhishu_robot_receiving_sim.mock_models import (
    LesMockModel,
    ReceivingOrder,
    ScannerMockModel,
)


class MockModelsTest(unittest.TestCase):
    def test_les_mock_covers_dispatch_feedback_and_completion(self):
        order = ReceivingOrder("ORDER-1", "PART-1", "RACK-1")
        model = LesMockModel(order)
        self.assertTrue(model.receive_and_inspect()["accepted"])
        self.assertEqual(model.dispatch_task()["part_barcode"], "PART-1")
        self.assertEqual(model.feedback_not_pushed()["feedback_count"], 1)
        self.assertTrue(model.complete_task()["completed"])
        self.assertTrue(model.completed)
        self.assertTrue(model.notify_agv_return()["agv_return_requested"])

    def test_scanner_mock_injects_one_mismatch_then_matches(self):
        model = ScannerMockModel("PART-1", "RACK-1", mismatch_attempts=1)
        first = model.scan_part_box()
        second = model.scan_part_box()
        self.assertFalse(first["matched"])
        self.assertTrue(second["matched"])
        self.assertTrue(model.scan_rack_sku()["matched"])

    def test_scanner_reset_restarts_fault_injection(self):
        model = ScannerMockModel("PART-1", "RACK-1", mismatch_attempts=1)
        model.scan_part_box()
        model.reset()
        self.assertFalse(model.scan_part_box()["matched"])


if __name__ == "__main__":
    unittest.main()
