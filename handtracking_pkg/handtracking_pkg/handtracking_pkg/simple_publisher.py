import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # New import
import cv2
import mediapipe as mp
import time

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(String, 'gesture_command', 10)  # Publishes gestures
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.detect_gesture)

        # MediaPipe Hand Detector
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

    def detect_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        gesture = "stop"  # Default gesture
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Extract landmarks
                landmarks = hand_landmarks.landmark
                thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
                index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                wrist = landmarks[self.mp_hands.HandLandmark.WRIST]

                # Gesture detection logic
                thumb_angle = abs(thumb_tip.x - index_tip.x) / abs(thumb_tip.y - index_tip.y + 1e-5)
                thumb_above_wrist = thumb_tip.y < wrist.y

                if thumb_above_wrist and thumb_angle < 1.0:
                    gesture = "move_forward"
                elif not thumb_above_wrist and thumb_angle < 1.0:
                    gesture = "move_backward"
                elif thumb_tip.x < wrist.x - 0.1:
                    gesture = "turn_right"
                elif thumb_tip.x > wrist.x + 0.1:
                    gesture = "turn_left"

        # Publish the gesture
        self.publisher_.publish(String(data=gesture))
        self.get_logger().info(f'Published Gesture: {gesture}')

        # Display frame (optional)
        cv2.imshow("Hand Gesture", frame)
        cv2.waitKey(1)

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    
    time.sleep(0.05)
    
    rclpy.init(args=args)
    node = GesturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
