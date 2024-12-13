import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2
import mediapipe as mp
from cv_bridge import CvBridge

class GestureControlRobot(Node):

    def __init__(self):
        super().__init__('gesture_control_robot')
        # ROS2 Publisher and Subscriber
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Timer for periodic tasks
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.motion)

        # Variables
        self.laser_forward = 0
        self.cmd = Twist()
        self.gesture_detected = False
        self.thumbs_down_detected = False
        self.turn_left_detected = False
        self.turn_right_detected = False

        # MediaPipe Hand Detector
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Start Video Capture
        self.cap = cv2.VideoCapture(0)  # Use your webcam

    def laser_callback(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359]

    def detect_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            return False

        # Convert the frame to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        # Reset gesture detection
        self.gesture_detected = False
        self.thumbs_down_detected = False
        self.turn_left_detected = False
        self.turn_right_detected = False

        # Initialize default positions
        thumb_tip_pos = (0, 0)
        wrist_pos = (0, 0)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks on the hand
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Extract landmark positions
                landmarks = hand_landmarks.landmark
                thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
                index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                wrist = landmarks[self.mp_hands.HandLandmark.WRIST]

                # Calculate positions in image space
                h, w, _ = frame.shape
                thumb_tip_pos = (int(thumb_tip.x * w), int(thumb_tip.y * h))
                wrist_pos = (int(wrist.x * w), int(wrist.y * h))

                # Determine thumb orientation
                thumb_above_wrist = thumb_tip.y < wrist.y
                thumb_below_wrist = thumb_tip.y > wrist.y
                thumb_far_left = thumb_tip.x < wrist.x - 0.1
                thumb_far_right = thumb_tip.x > wrist.x + 0.1

                # Compute thumb angles (Optional)
                thumb_angle = abs(thumb_ip.x - thumb_tip.x) / abs(thumb_ip.y - thumb_tip.y + 1e-5)

                # Gesture detection
                if thumb_above_wrist and thumb_angle < 1.0:  # Adjust angle threshold as needed
                    self.gesture_detected = True  # Thumbs-up
                elif thumb_below_wrist and thumb_angle < 1.0:
                    self.thumbs_down_detected = True  # Thumbs-down
                elif thumb_far_left and thumb_angle > 1.0:
                    self.turn_right_detected = True  # Thumb pointing left
                elif thumb_far_right and thumb_angle > 1.0:
                    self.turn_left_detected = True  # Thumb pointing right

        # Debugging
        self.get_logger().info(
            f'Thumb ({thumb_tip_pos[0]}, {thumb_tip_pos[1]}), Wrist ({wrist_pos[0]}, {wrist_pos[1]}), '
            f'Thumbs up: {self.gesture_detected}, Thumbs down: {self.thumbs_down_detected}, '
            f'Turn left: {self.turn_left_detected}, Turn right: {self.turn_right_detected}'
        )

        # Display the frame with debugging
        cv2.circle(frame, thumb_tip_pos, 5, (0, 255, 0), -1)  # Green for thumb tip
        cv2.circle(frame, wrist_pos, 5, (255, 0, 0), -1)  # Blue for wrist
        cv2.imshow("Hand Gesture Debug", frame)
        cv2.waitKey(1)

    def motion(self):
        # Detect gesture
        self.detect_gesture()

        # Log the laser and gesture status
        self.get_logger().info(
            f'Laser forward: {self.laser_forward}, Thumbs up: {self.gesture_detected}, Thumbs down: {self.thumbs_down_detected}, '
            f'Turn left: {self.turn_left_detected}, Turn right: {self.turn_right_detected}'
        )

        # Move logic
        if self.gesture_detected and self.laser_forward > 0.5:
            # Move forward
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        elif self.thumbs_down_detected and self.laser_forward > 0.5:
            # Move backward
            self.cmd.linear.x = -0.2
            self.cmd.angular.z = 0.0
        elif self.turn_left_detected:
            # Turn left
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5
        elif self.turn_right_detected:
            # Turn right
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.5
        else:
            # Stop
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # Publish the command
        self.publisher_.publish(self.cmd)

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    robot = GestureControlRobot()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.cleanup()
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
