import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Subpub(Node):

    def __init__(self):
        super().__init__('subpub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timer_period = 0.5
        self.laser_ranges = []  # Initialize an empty list to hold LIDAR data
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        # Store the entire range data from the LIDAR
        self.laser_ranges = msg.ranges

    def motion(self):
        # Check the length of the laser_ranges to avoid index errors
        if not self.laser_ranges:
            return

        # Define indices for the frontal area and side areas
        front_indices = range(340, 360)  # Front range (0° to 20°)
        left_indices = range(270, 340)   # Left range (90° to 180°)
        right_indices = range(0, 90)      # Right range (270° to 360°)

        # Calculate the minimum distances in the defined ranges
        front_distance = min(self.laser_ranges[i] for i in front_indices if self.laser_ranges[i] > 0)
        left_distance = min(self.laser_ranges[i] for i in left_indices if self.laser_ranges[i] > 0)
        right_distance = min(self.laser_ranges[i] for i in right_indices if self.laser_ranges[i] > 0)

        self.get_logger().info('Front: %s, Left: %s, Right: %s' % 
                                (front_distance, left_distance, right_distance))
        
        # Logic for moving and turning
        if front_distance > 0.6:  # If clear in front
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            # If something is too close in front, decide to turn
            if left_distance > right_distance:
                # More space on the left, turn right
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.2  # Negative for right turn
            else:
                # More space on the right, turn left
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2  # Positive for left turn

        # Publish the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    subpub = Subpub()       
    rclpy.spin(subpub)
    subpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()