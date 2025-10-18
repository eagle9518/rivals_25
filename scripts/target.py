#!/usr/bin/env python3
imort math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener
from rivals.msg import ShooterTarget


class ShooterToTarget(Node):
    def __init__(self):
        super().__init__('shooter_to_target')

        # --- Parameters ---
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('shooter_frame', 'shooter_link')
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('alliance', 'blue')
        self.declare_parameter('top_tower_button', 4)
        self.declare_parameter('bottom_tower_button', 5)
        self.declare_parameter('level_buttons', {})
        self.declare_parameter('target_map', {})

        self.camera_frame = self.get_parameter('camera_frame').value
        self.shooter_frame = self.get_parameter('shooter_frame').value
        self.update_rate = self.get_parameter('update_rate').value
        self.alliance = self.get_parameter('alliance').value.lower()
        self.top_tower_button = self.get_parameter('top_tower_button').value
        self.bottom_tower_button = self.get_parameter('bottom_tower_button').value
        self.level_buttons = self.get_parameter('level_buttons').value
        self.target_map = self.get_parameter('target_map').value

        # --- TF setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers/Subscribers ---
        self.pub = self.create_publisher(ShooterTarget, 'shooter_to_target', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(1.0 / self.update_rate, self.update)

        # --- State ---
        self.active_tower = "top"
        self.active_level = "L1"
        self.active_target = self.resolve_target()

        self.get_logger().info(f"Started ShooterToTarget node. Alliance={self.alliance}, Default target={self.active_target}")

    # --------------------------------------------------------
    def resolve_target(self):
        key = f"{self.alliance}_{self.active_tower}_{self.active_level}"
        return self.target_map.get(key, None)

    # --------------------------------------------------------
    def joy_callback(self, msg: Joy):
        # Tower selection
        if len(msg.buttons) > self.top_tower_button and msg.buttons[self.top_tower_button]:
            self.active_tower = "top"
        elif len(msg.buttons) > self.bottom_tower_button and msg.buttons[self.bottom_tower_button]:
            self.active_tower = "bottom"

        # Level selection
        for idx, level_name in self.level_buttons.items():
            idx = int(idx)
            if len(msg.buttons) > idx and msg.buttons[idx]:
                self.active_level = level_name
                break

        # Resolve new target
        new_target = self.resolve_target()
        if new_target and new_target != self.active_target:
            self.active_target = new_target
            self.get_logger().info(f"Target switched to {self.active_target}")

    # --------------------------------------------------------
    def update(self):
        if not self.active_target:
            return

        try:
            world_to_target = self.tf_buffer.lookup_transform('world', self.active_target, rclpy.time.Time())
            world_to_shooter = self.tf_buffer.lookup_transform('world', self.shooter_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"TF lookup failed: {e}")
            return

        tx = world_to_target.transform.translation.x - world_to_shooter.transform.translation.x
        ty = world_to_target.transform.translation.y - world_to_shooter.transform.translation.y
        tz = world_to_target.transform.translation.z - world_to_shooter.transform.translation.z

        distance = math.sqrt(tx**2 + ty**2 + tz**2)
        yaw_error = math.atan2(ty, tx)
        pitch_error = math.atan2(-tz, math.sqrt(tx**2 + ty**2))

        msg = ShooterTarget()
        msg.distance = distance
        msg.yaw_error = yaw_error
        msg.pitch_error = pitch_error

        self.pub.publish(msg)
        self.get_logger().debug(f"{self.active_target}: dist={distance:.2f}, yaw={yaw_error:.2f}, pitch={pitch_error:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ShooterToTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()p
