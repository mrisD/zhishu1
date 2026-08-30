"""ROS services that emulate the LES calls used by the receiving flow."""

import json
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .mock_models import LesMockModel, ReceivingOrder


class LesMockNode(Node):
    def __init__(self) -> None:
        super().__init__("les_mock")
        self.declare_parameter("order_id", "RCV-SIM-001")
        self.declare_parameter("part_barcode", "PART-BOX-001")
        self.declare_parameter("rack_sku", "RACK-A01")
        self.declare_parameter("quality_pass", True)
        self.declare_parameter("latency_seconds", 0.25)
        self._latency = float(self.get_parameter("latency_seconds").value)
        if self._latency < 0.0 or self._latency > 10.0:
            raise ValueError("latency_seconds must be in [0, 10]")
        self._model = LesMockModel(
            ReceivingOrder(
                order_id=str(self.get_parameter("order_id").value),
                part_barcode=str(self.get_parameter("part_barcode").value),
                rack_sku=str(self.get_parameter("rack_sku").value),
            ),
            quality_pass=bool(self.get_parameter("quality_pass").value),
        )
        self.create_service(
            Trigger, "/les_mock/receive_and_inspect", self._receive_and_inspect
        )
        self.create_service(Trigger, "/les_mock/dispatch_task", self._dispatch_task)
        self.create_service(
            Trigger, "/les_mock/feedback_not_pushed", self._feedback_not_pushed
        )
        self.create_service(Trigger, "/les_mock/complete_task", self._complete_task)
        self.create_service(
            Trigger, "/les_mock/notify_agv_return", self._notify_agv_return
        )
        self.get_logger().info("LES Mock services are ready")

    def _reply(self, response, payload, success=True):
        if self._latency:
            time.sleep(self._latency)
        response.success = success
        response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _receive_and_inspect(self, _request, response):
        payload = self._model.receive_and_inspect()
        return self._reply(response, payload, bool(payload["accepted"]))

    def _dispatch_task(self, _request, response):
        return self._reply(response, self._model.dispatch_task())

    def _feedback_not_pushed(self, _request, response):
        return self._reply(response, self._model.feedback_not_pushed())

    def _complete_task(self, _request, response):
        return self._reply(response, self._model.complete_task())

    def _notify_agv_return(self, _request, response):
        return self._reply(response, self._model.notify_agv_return())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LesMockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
