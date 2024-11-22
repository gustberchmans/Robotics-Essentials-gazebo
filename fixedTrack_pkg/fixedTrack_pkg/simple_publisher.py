import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import random

class Path(Node):

    def __init__(self):
        super().__init__('path')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 1  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)




    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:20])
        self.laser_frontRight = min(msg.ranges[340:359])

    def move(self, linear_velocity, angular_velocity, duration):
        # Log the orientation (angular_velocity)
        self.get_logger().info(f'Moving with angular velocity: {angular_velocity}')

        self.cmd.linear.x = linear_velocity
        self.cmd.angular.z = angular_velocity

        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(self.cmd)
            time.sleep(0.01)  # sleep for 10ms

        # Stop the robot
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


    def motion(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    
        # Drive 2 meter forward (assuming max speed is 0.2 m/s)
        self.move(0.2, 0.0, 12.0)

        # Turn 90 degrees to the left (assuming max angular speed is 0.2 rad/s)
        self.move(0.0, 0.2, 7.85)  # Pi/4 rad

        # Drive 30 cm forward
        self.move(0.2, 0.0, 5)

        # Turn 90 degrees to the left
        self.move(0.0, 0.2, 7.85)  # Pi/4 rad

        # Drive 1 meter forward
        self.move(0.2, 0.0, 12)

        # Turn 90 degrees to the right
        self.move(0.0, -0.2, 7.85)  # -Pi/4 rad

        # Drive 30 cm forward
        self.move(0.2, 0.0, 5)

        # Turn 90 degrees to the right
        self.move(0.0, -0.2, 7.85)  # -Pi/4 rad

def main(args=None):
    rclpy.init(args=args)
    path = Path()
    rclpy.spin(path)
    path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()