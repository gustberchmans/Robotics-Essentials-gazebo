import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.gesture_subscriber = self.create_subscription(String, 'gesture_command', self.gesture_callback, 10)

        # Initialize laser data and gesture
        self.laser_forward = float('inf')
        self.current_gesture = "stop"
        self.cmd = Twist()  # Initialize the cmd object

    def laser_callback(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359] if len(msg.ranges) > 359 else float('inf')

    def gesture_callback(self, msg):
        # Update the current gesture
        self.current_gesture = msg.data

    def motion(self):
        # Log the laser and gesture status
        self.get_logger().info(
            f'Laser forward: {self.laser_forward}, Current Gesture: {self.current_gesture}'
        )

        # Handle motion logic based on detected gestures
        if self.current_gesture == "move_forward" and self.laser_forward > 0.5:
            self.cmd.linear.x = 0.2  # Move forward
            self.cmd.angular.z = 0.0
            self.get_logger().info("Moving forward")
        elif self.current_gesture == "move_backward" and self.laser_forward > 0.5:
            self.cmd.linear.x = -0.2  # Move backward
            self.cmd.angular.z = 0.0
            self.get_logger().info("Moving backward")
        elif self.current_gesture == "turn_left":
            self.cmd.linear.x = 0.0  # Stop forward motion
            self.cmd.angular.z = 0.5  # Turn left
            self.get_logger().info("Turning left")
        elif self.current_gesture == "turn_right":
            self.cmd.linear.x = 0.0  # Stop forward motion
            self.cmd.angular.z = -0.5  # Turn right
            self.get_logger().info("Turning right")
        elif self.current_gesture == "stop":
            self.cmd.linear.x = 0.0  # Stop moving
            self.cmd.angular.z = 0.0  # Stop turning
            self.get_logger().info("Stopping")
        else:
            self.cmd.linear.x = 0.0  # Stop moving
            self.cmd.angular.z = 0.0  # Stop turning
            self.get_logger().info("No valid gesture detected, stopping")

        # Publish the command to the robot
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    try:
        # Use a timer to repeatedly call motion and handle gestures
        timer = node.create_timer(0.1, node.motion)  # Run motion every 0.1 seconds
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
