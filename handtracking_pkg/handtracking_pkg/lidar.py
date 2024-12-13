import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import re

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.gesture_subscriber = self.create_subscription(String, 'gesture_command', self.gesture_callback, 10)

        # Initialize laser data and gesture
        self.laser_forward = float('inf')
        self.current_gesture = "stop"
        self.current_speed = 0.1  # Default speed
        self.cmd = Twist()  # Initialize the cmd object

    def laser_callback(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[0] if len(msg.ranges) > 0 else float('inf')

    def gesture_callback(self, msg):
        # Parse the gesture message to extract the gesture and speed
        gesture_message = msg.data
        # Extract gesture and speed using regular expression
        match = re.match(r"(move_forward|move_backward|turn_left|turn_right|stop).*speed:\s*(\d*\.\d+|\d+)", gesture_message)
        
        if match:
            self.current_gesture = match.group(1)
            self.current_speed = float(match.group(2))
        else:
            self.current_gesture = "stop"
            self.current_speed = 0.1  # Default speed if no match

    def motion(self):
        # Log the laser and gesture status
        self.get_logger().info(
            f'Laser forward: {self.laser_forward}, Current Gesture: {self.current_gesture}, Speed: {self.current_speed}'
        )

        # Handle motion logic based on detected gestures and speed
        if self.current_gesture == "move_forward" and self.laser_forward > 0.5:
            self.cmd.linear.x = self.current_speed  # Use dynamic speed
            self.cmd.angular.z = 0.0
            self.get_logger().info(f"Moving forward with speed {self.current_speed}")
        elif self.current_gesture == "move_backward" and self.laser_forward > 0.5:
            self.cmd.linear.x = -self.current_speed  # Move backward
            self.cmd.angular.z = 0.0
            self.get_logger().info(f"Moving backward with speed {self.current_speed}")
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
