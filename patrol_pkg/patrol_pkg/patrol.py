import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        
        # Keep track of previous side distances to determine if they are decreasing
        self.prev_left_distance = float('inf')
        self.prev_right_distance = float('inf')
        
        # Variable to control turning behavior
        self.turning = False

    def laser_callback(self, msg):
        # Store the entire range data from the LIDAR
        self.laser_ranges = msg.ranges

    def motion(self):
        # Check if we have received LIDAR data
        if not self.laser_ranges:
            return

        # Define indices for the frontal area and side areas
        front_indices = list(range(330, 360)) + list(range(0, 31))
        left_indices = range(270, 340)   # Left range (90° to 180°)
        right_indices = range(0, 90)      # Right range (270° to 360°)

        # Calculate the minimum distances in the defined ranges
        front_distance = min(self.laser_ranges[i] for i in front_indices if self.laser_ranges[i] > 0)
        left_distance = min(self.laser_ranges[i] for i in left_indices if self.laser_ranges[i] > 0)
        right_distance = min(self.laser_ranges[i] for i in right_indices if self.laser_ranges[i] > 0)

        self.get_logger().info(f'Front: {front_distance}, Left: {left_distance}, Right: {right_distance}')
        
        # Logic for moving and turning
        if front_distance > 0.6:  # If clear in front, move forward
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
            self.turning = False  # Stop turning when moving forward
        else:
            # Check if we are turning or need to start turning
            if left_distance > right_distance and not self.turning:
                # More space on the left, turn right
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.2  # Negative for right turn
                self.turning = True
            elif right_distance > left_distance and not self.turning:
                # More space on the right, turn left
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2  # Positive for left turn
                self.turning = True
            elif self.turning:
                # Keep turning while the distances are decreasing
                if left_distance <= self.prev_left_distance or right_distance <= self.prev_right_distance:
                    # Continue turning if the distance to the obstacles are decreasing
                    self.cmd.linear.x = 0.0
                    if left_distance > right_distance:
                        self.cmd.angular.z = -0.2  # Right turn
                    else:
                        self.cmd.angular.z = 0.2   # Left turn
                else:
                    # If the distance is increasing, stop turning and go forward
                    self.cmd.linear.x = 0.2
                    self.cmd.angular.z = 0.0
                    self.turning = False  # Stop turning when it's clear

        # Update previous distances for comparison
        self.prev_left_distance = left_distance
        self.prev_right_distance = right_distance

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
