import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

class Odom(Node):

    def __init__(self):
        super().__init__('odom')
        # Create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create the subscriber object
        self.odomSubscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # Timer period (0.5 seconds)
        self.timer_period = 0.5
        
        # Motion tracking variables
        self.starting_position_x = None
        self.starting_position_y = None
        self.starting_yaw = None
        self.current_yaw = None
        self.distance_traveled = 0.0
        
        # Command queue
        self.commands = [
            {"type": "go_forward", "value": 2.6},  # Move forward 2.6 meters
            {"type": "turn", "angle": math.radians(45)},  # Turn 90 degrees (π/2 radians)
            {"type": "go_forward", "value": 1},    # Move forward 1 meter
            {"type": "turn", "angle": math.radians(-45)},  # Turn -90 degrees (-π/2 radians)
            {"type": "go_forward", "value": 5.5},
            {"type": "turn", "angle": math.radians(85)},  # Turn 180 degrees (π radians)
            {"type": "go_forward", "value": 4},
            {"type": "turn", "angle": math.radians(90)},  # Turn 180 degrees (π radians)
            {"type": "go_forward", "value": 5.4},        
            {"type": "turn", "angle": math.radians(85)},  
            {"type": "go_forward", "value": 3},  
            {"type": "turn", "angle": math.radians(85)}, 
            {"type": "turn", "angle": math.radians(85)}, 
            {"type": "go_forward", "value": 3},  
        ]
        self.current_command = None
        self.command_index = 0

        # Create Twist message
        self.cmd = Twist()
        
        # Create a timer for periodic motion updates
        self.timer = self.create_timer(self.timer_period, self.execute_command)

    def odom_callback(self, msg):
        # Update position
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        # Track the starting position and yaw for distance/angle calculation
        if self.starting_position_x is None:
            self.starting_position_x = self.position_x
            self.starting_position_y = self.position_y
            self.starting_yaw = self.current_yaw

        # Calculate distance traveled
        self.distance_traveled = ((self.position_x - self.starting_position_x) ** 2 + 
                                  (self.position_y - self.starting_position_y) ** 2) ** 0.5

    def execute_command(self):
        # If all commands are completed, stop the robot
        if self.command_index >= len(self.commands):
            self.get_logger().info("All commands completed. Stopping robot.")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        # Get the current command
        if self.current_command is None:
            self.current_command = self.commands[self.command_index]
            self.starting_position_x = None  # Reset starting position for distance calculation
            self.starting_yaw = self.current_yaw  # Reset starting yaw

        # Execute the current command
        command_type = self.current_command["type"]

        if command_type == "go_forward":
            self.go_forward(self.current_command["value"])
        elif command_type == "turn":
            self.turn(self.current_command["angle"])

    def go_forward(self, distance):
        self.get_logger().info(f"Moving forward: {self.distance_traveled:.2f}/{distance} meters")
        
        if self.distance_traveled < distance:
            self.cmd.linear.x = 0.2  # Set forward speed
            self.cmd.angular.z = 0.0
        else:
            # Stop and move to the next command
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.next_command()

        # Publish the motion command
        self.publisher_.publish(self.cmd)

    def turn(self, target_angle):
        # Calculate the angle difference
        angle_diff = self.normalize_angle(self.current_yaw - self.starting_yaw)
        self.get_logger().info(f"Turning: {math.degrees(angle_diff):.2f}/{math.degrees(target_angle):.2f} degrees")
        
        if abs(angle_diff) < abs(target_angle):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3 if target_angle > 0 else -0.3  # Set turning speed
        else:
            # Stop and move to the next command
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.next_command()

        # Publish the motion command
        self.publisher_.publish(self.cmd)

    def next_command(self):
        # Move to the next command in the queue
        self.current_command = None
        self.command_index += 1
        self.get_logger().info("Moving to the next command.")

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def normalize_angle(angle):
        # Normalize an angle to the range [-π, π]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    odom = Odom()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
