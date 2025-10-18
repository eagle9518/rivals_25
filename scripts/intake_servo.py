import rcpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Joy


class IntakeServo(Node):

    def __init__(self):
        super().__init__('intake_servo')
        self.create_subscription(Bool, 'has_projectile', self.has_projectile_cb, 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        self.pos_pub = self.create_publisher(Float64, 'intake_position_controller/commands', 10)

        self.has_projectile = False
        self.joy_pressed = False

        self.declare_parameter('intake_servo_button', 5)
        self.declare_parameter('intake_servo_out_position', 0)
        self.declare_parameter('intake_servo_in_position', 180)

        self.servo_out = self.get_parameter('intake_servo_out_position').value
        self.servo_in = self.get_parameter('intake_servo_in_position').value
        self.button_index = self.get_parameter('intake_servo_button').value

        self.create_timer(0.02, self.update)  # 50 Hz loop

    def has_projectile_cb(self, msg: Bool):
        self.has_projectile = msg.data

    def joy_cb(self, msg: Joy):
        if len(msg.buttons) > self.button_index:
            self.joy_pressed = msg.buttons[self.button_index] == 1
        else:
            self.joy_pressed = False

    def update(self):
        pos_cmd = Float64()

        if not self.has_projectile and self.joy_pressed:
            pos_cmd.data = self.servo_out
        else:
            pos_cmd.data = self.servo_in

        self.pos_pub.publish(pos_cmd)

        self.get_logger().debug(
            f"{self.get_name()}: {pos_cmd.data}"
        )

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = IntakeServo()
    rclpy.spin(IntakeServo)
    minimal_publisher.destroy_node()
    rclpy.shutdown()l
