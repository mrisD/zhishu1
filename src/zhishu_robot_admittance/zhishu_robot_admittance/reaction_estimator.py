"""Estimate bottom support using only two wrist F/T sensors, not contacts."""
import math
from geometry_msgs.msg import PoseStamped, WrenchStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from .admittance_math import rotate_vector
from .reaction_math import bottom_reaction


class ReactionEstimator(Node):
    def __init__(self):
        super().__init__('bottom_reaction_estimator')
        self.mass = float(self.declare_parameter('box_mass_kg', 15.0).value)
        self.tools = float(self.declare_parameter('total_tool_mass_kg', 0.56).value)
        bottom_reaction(0.0, 0.0, self.mass, self.tools)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.samples = {}
        self.base = None
        self.filtered = None
        self.last_pair = None
        self.last_time = None
        self.publisher = self.create_publisher(WrenchStamped,
            '/pallet_bin/estimated_bottom_reaction', 10)
        self.raw_publisher = self.create_publisher(WrenchStamped,
            '/pallet_bin/raw_bottom_reaction', 10)
        for side in ('left', 'right'):
            self.create_subscription(WrenchStamped, f'/{side}_wrist_ft',
                lambda msg, side=side: self.samples.update({side: (msg, self.get_clock().now().nanoseconds)}),
                qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/pallet_bin/measured_base_pose',
            self.base_callback, 10)
        self.create_timer(0.01, self.update)
        self.get_logger().info('Wrist-only quasi-static support estimator; 15 kg box, 0.56 kg tools; no loaded tare')

    def base_callback(self, msg):
        self.base = (msg, self.get_clock().now().nanoseconds)

    def update(self):
        now = self.get_clock().now()
        if len(self.samples) != 2 or self.base is None:
            return
        if any((now.nanoseconds - received) * 1e-9 > 0.10
               for _, received in self.samples.values()):
            self.filtered = None
            return
        if (now.nanoseconds - self.base[1]) * 1e-9 > 0.5:
            self.filtered = None
            return
        stamps = tuple(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                       for msg, _ in self.samples.values())
        if abs(stamps[0] - stamps[1]) > 20_000_000 or stamps == self.last_pair:
            return
        q = self.base[0].pose.orientation
        base_rotation = [q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in base_rotation) or abs(sum(v*v for v in base_rotation)-1.0) > 0.01:
            return
        forces = []
        try:
            for side, frame in (('left', 'L_hand_link'), ('right', 'R_hand_link')):
                msg, _ = self.samples[side]
                f, t = msg.wrench.force, msg.wrench.torque
                values = [f.x, f.y, f.z, t.x, t.y, t.z]
                if not all(math.isfinite(v) for v in values):
                    return
                transform = self.tf.lookup_transform('base_footprint', frame, Time())
                r = transform.transform.rotation
                force = rotate_vector(values[:3], [r.x, r.y, r.z, r.w])
                forces.append(rotate_vector(force, base_rotation))
        except TransformException:
            return
        value = bottom_reaction(forces[0][2], forces[1][2], self.mass, self.tools)
        dt = max(0.0, (now.nanoseconds - (self.last_time or now.nanoseconds)) * 1e-9)
        alpha = 1.0 - math.exp(-2.0 * math.pi * 2.0 * dt)
        self.filtered = value if self.filtered is None else self.filtered + alpha * (value - self.filtered)
        self.last_pair, self.last_time = stamps, now.nanoseconds
        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'world'
        message.wrench.force.z = value
        self.raw_publisher.publish(message)
        message.wrench.force.z = self.filtered
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ReactionEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
