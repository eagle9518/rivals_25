import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class BeamBreak(Node):

    # TODO Magic numbers should be in a config.py file ideally
    def __init__(self):
        super().__init__('BeamBreak')
        self.publisher_ = self.create_publisher(Bool, 'has_projectile', 10)

        timer_period = 0.010
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.has_projectile = False
        self.intake_triggered = False


    # TODO: Read break beam sensor
    def read_intake_sensor(self):
        return False

    # TODO: Read break beam sensor
    def read_shooter_sensor(self):
        return False

    # TODO Simple break beam logic, if false positives are frequent implement
    #      time-based system instaed 
    def timer_callback(self):
        intake_triggered = read_intake_sensor()
        shooter_triggered = read_shooter_sensor()

        if intake_triggered:
            self.has_projectile = True
            self.intake_triggered = True

        elif self.intake_triggered and shooter_triggered:
            self.has_projectile = False
            self.intake_triggered = False       

        self.publisher_.publish(self.has_projectile)
        self.get_logger().info(f"{self.node_name}: has_projectile={self.has_projectile}")

def main(args=None):
    rclpy.init(args=args)
    node = BeamBreak()
    rclpy.spin(node)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
