"""ROS services that emulate part-box and rack barcode readers."""

import json
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .mock_models import ScannerMockModel


class ScannerMockNode(Node):
    def __init__(self) -> None:
        super().__init__("scanner_mock")
        self.declare_parameter("expected_part_barcode", "PART-BOX-001")
        self.declare_parameter("expected_rack_sku", "RACK-A01")
        self.declare_parameter("part_mismatch_attempts", 1)
        self.declare_parameter("latency_seconds", 0.15)
        self._latency = float(self.get_parameter("latency_seconds").value)
        if self._latency < 0.0 or self._latency > 10.0:
            raise ValueError("latency_seconds must be in [0, 10]")
        self._model = ScannerMockModel(
            str(self.get_parameter("expected_part_barcode").value),
            str(self.get_parameter("expected_rack_sku").value),
            int(self.get_parameter("part_mismatch_attempts").value),
        )
        self.create_service(
            Trigger, "/scanner_mock/scan_part_box", self._scan_part_box
        )
        self.create_service(Trigger, "/scanner_mock/scan_rack_sku", self._scan_rack)
        self.create_service(Trigger, "/scanner_mock/reset", self._reset)
        self.get_logger().info("Scanner Mock services are ready")

    def _reply(self, response, payload) -> Trigger.Response:
        if self._latency:
            time.sleep(self._latency)
        response.success = True
        response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _scan_part_box(self, _request, response):
        return self._reply(response, self._model.scan_part_box())

    def _scan_rack(self, _request, response):
        return self._reply(response, self._model.scan_rack_sku())

    def _reset(self, _request, response):
        self._model.reset()
        return self._reply(response, {"reset": True})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScannerMockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
