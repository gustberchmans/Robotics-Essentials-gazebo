import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize linear velocity
        self.linear_velocity = 0.5  # Starting linear velocity

    def timer_callback(self):
        msg = Twist()
        # Update the linear velocity to increase over time
        self.linear_velocity += 0.1  # Increment the linear velocity
        msg.linear.x = self.linear_velocity
        msg.angular.z = 0.5  # Keep angular velocity constant

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
