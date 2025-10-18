import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Joy


class Intake(Node):

    def __init__(self):
        super().__init__('intake')
        self.create_subscription(Bool, 'has_projectile', self.has_projectile_cb, 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        self.pos_pub = self.create_publisher(Float64, 'intake_position_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64, 'intake_velocity_controller/commands', 10)

        self.has_projectile = False
        self.shooter_ready = False
        self.joy_pressed = False

        self.declare_parameter('intake_active', 1.0)
        self.declare_parameter('intake_idle', 0.0)
        self.declare_parameter('servo_open', 1.0)
        self.declare_parameter('servo_closed', 0.0)
        self.declare_parameter('button_index', 5)

        self.intake_active = self.get_parameter('intake_active').value
        self.intake_idle = self.get_parameter('intake_idle').value
        self.servo_open = self.get_parameter('servo_open').value
        self.servo_closed = self.get_parameter('servo_closed').value
        self.button_index = self.get_parameter('button_index').value

        self.create_timer(0.02, self.update)  # 50 Hz loop

    def has_projectile_cb(self, msg: Bool):
        self.has_projectile = msg.data

    def shooter_ready_cb(self, msg: Bool):
        self.shooter_ready = msg.data

    def joy_cb(self, msg: Joy):
        if len(msg.buttons) > self.button_index:
            self.joy_pressed = msg.buttons[self.button_index] == 1
        else:
            self.joy_pressed = False

    def update(self):
        pos_cmd = Float64()
        vel_cmd = Float64()

        if self.shooter_ready and not self.has_projectile and self.joy_pressed:
            pos_cmd.data = self.servo_open
            vel_cmd.data = self.intake_active
        else:
            pos_cmd.data = self.servo_closed
            vel_cmd.data = self.intake_idle

        self.pos_pub.publish(pos_cmd)
        self.vel_pub.publish(vel_cmd)

        self.get_logger().debug(
            f"ready={self.shooter_ready} has_proj={self.has_projectile} "
            f"joy={self.joy_pressed} pos={pos_cmd.data} vel={vel_cmd.data}"
        )

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Intake()
    rclpy.spin(Intake)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
