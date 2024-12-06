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

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks on the hand
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Check for thumbs-up gesture
                landmarks = hand_landmarks.landmark
                thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP].y
                index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y

                # Thumbs-up if thumb tip is lower than index finger tip in the y-axis
                if thumb_tip < index_tip:
                    self.gesture_detected = True
                    break

        # Display the frame (Optional)
        cv2.imshow("Hand Gesture", frame)
        cv2.waitKey(1)

    def motion(self):
        # Detect gesture
        self.detect_gesture()

        # Log the laser and gesture status
        self.get_logger().info(f'Laser forward: {self.laser_forward}, Gesture detected: {self.gesture_detected}')

        # Move logic
        if self.gesture_detected and self.laser_forward > 0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
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
