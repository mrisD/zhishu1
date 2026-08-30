import unittest

from zhishu_robot_receiving_sim.flow import FlowState, ReceivingFlow, is_subsequence


def advance_to_barcode(flow):
    while flow.state != FlowState.PART_BARCODE_SCAN:
        flow.advance()


class ReceivingFlowTest(unittest.TestCase):
    def test_subsequence_helper(self):
        self.assertTrue(is_subsequence(["a", "c"], ["a", "b", "c"]))
        self.assertFalse(is_subsequence(["c", "a"], ["a", "b", "c"]))

    def test_success_path_reaches_receipt_complete(self):
        flow = ReceivingFlow()
        advance_to_barcode(flow)
        self.assertEqual(flow.barcode_attempt, 1)
        self.assertEqual(flow.barcode_result(True), FlowState.RACK_SKU_SCAN)
        while not flow.terminal:
            flow.advance()
        self.assertEqual(flow.state, FlowState.RECEIPT_COMPLETE)

    def test_barcode_mismatch_returns_through_les_feedback(self):
        flow = ReceivingFlow(max_barcode_attempts=3)
        advance_to_barcode(flow)
        self.assertEqual(flow.barcode_result(False), FlowState.LES_NOT_PUSHED)
        self.assertEqual(flow.advance(), FlowState.PART_BARCODE_SCAN)
        self.assertEqual(flow.barcode_attempt, 2)
        self.assertEqual(flow.barcode_result(True), FlowState.RACK_SKU_SCAN)

    def test_barcode_retry_limit_fails_flow(self):
        flow = ReceivingFlow(max_barcode_attempts=2)
        advance_to_barcode(flow)
        self.assertEqual(flow.barcode_result(False), FlowState.LES_NOT_PUSHED)
        flow.advance()
        self.assertEqual(flow.barcode_result(False), FlowState.FAILED)
        self.assertIn("retry limit", flow.failure_reason)

    def test_invalid_transition_is_rejected(self):
        flow = ReceivingFlow()
        with self.assertRaises(RuntimeError):
            flow.barcode_result(True)
        advance_to_barcode(flow)
        with self.assertRaises(RuntimeError):
            flow.advance()

    def test_invalid_retry_configuration_is_rejected(self):
        for attempts in (0, 21):
            with self.subTest(attempts=attempts):
                with self.assertRaises(ValueError):
                    ReceivingFlow(attempts)


if __name__ == "__main__":
    unittest.main()
