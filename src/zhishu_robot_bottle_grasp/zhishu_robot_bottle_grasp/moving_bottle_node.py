#!/usr/bin/env python3
"""Deterministic moving-bottle source for the Gazebo acceptance world."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger


class MovingBottleNode(Node):
    def __init__(self):
        super().__init__("moving_bottle")
        defaults = {
            "speed": 0.055,
            "start_x": 0.62,
            "start_y": -0.31,
            "start_z": 1.22,
            "travel_distance": 0.62,
            "auto_reset": True,
            "model_name": "bottle",
            "set_pose_service": "/world/bottle_grasp/set_pose",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._publisher = self.create_publisher(Twist, "/bottle/cmd_vel", 10)
        self._detach_publisher = self.create_publisher(Empty, "/bottle_grasp/detach", 10)
        self._set_pose = self.create_client(
            SetEntityPose, self.get_parameter("set_pose_service").value
        )
        self.create_service(SetBool, "~/set_motion", self._set_motion)
        self.create_service(Trigger, "~/reset", self._reset_service)
        self._enabled = True
        self._cycle_started = self.get_clock().now()
        self._startup_detach_until = time.monotonic() + 1.5
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Moving bottle enabled at {self.get_parameter('speed').value:.3f} m/s"
        )

    def _command(self, speed):
        command = Twist()
        command.linear.y = float(speed)
        self._publisher.publish(command)

    def _tick(self):
        if time.monotonic() < self._startup_detach_until:
            self._detach_publisher.publish(Empty())
        speed = float(self.get_parameter("speed").value) if self._enabled else 0.0
        self._command(speed)
        if not self._enabled or not self.get_parameter("auto_reset").value or speed <= 0.0:
            return
        period = float(self.get_parameter("travel_distance").value) / speed
        elapsed = (self.get_clock().now() - self._cycle_started).nanoseconds * 1e-9
        if elapsed >= period:
            self._request_reset()

    def _set_motion(self, request, response):
        self._enabled = bool(request.data)
        if self._enabled:
            self._cycle_started = self.get_clock().now()
        else:
            self._command(0.0)
        response.success = True
        response.message = "moving" if self._enabled else "stopped"
        return response

    def _reset_service(self, _request, response):
        response.success = self._request_reset()
        response.message = "reset requested" if response.success else "set_pose unavailable"
        return response

    def _request_reset(self):
        self._detach_publisher.publish(Empty())
        if not self._set_pose.service_is_ready():
            self._set_pose.wait_for_service(timeout_sec=0.1)
        if not self._set_pose.service_is_ready():
            return False
        request = SetEntityPose.Request()
        request.entity.name = self.get_parameter("model_name").value
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(self.get_parameter("start_x").value)
        request.pose.position.y = float(self.get_parameter("start_y").value)
        request.pose.position.z = float(self.get_parameter("start_z").value)
        request.pose.orientation.w = 1.0
        self._set_pose.call_async(request)
        self._cycle_started = self.get_clock().now()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MovingBottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._command(0.0)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
