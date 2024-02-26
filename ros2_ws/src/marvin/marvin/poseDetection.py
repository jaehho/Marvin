import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
from geometry_msgs.msg import Point

class PoseDetectionPublisher(Node):
    def __init__(self):
        super().__init__('pose_detection_publisher')
        self.publisher_ = self.create_publisher(PoseLandmark, 'pose_landmarks', 10)
        self.init_matplotlib()

    def init_matplotlib(self):
        """Initialize Matplotlib for 3D plotting."""
        plt.ion()  # Turn on interactive mode to update plots in real-time
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        # Set the labels of the 3D plot
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def draw_matplotlib_landmarks(self, landmarks, connections):
        """Plot landmarks and connections in the Matplotlib 3D subplot."""
        self.ax.clear()
        self.ax.set_xlim3d(-0.5, 0.5)
        self.ax.set_ylim3d(-0.5, 0.5)
        self.ax.set_zlim3d(-0.5, 0.5)
        
        if landmarks:
            xs, ys, zs = self.extract_coordinates(landmarks)
            self.ax.scatter(xs, ys, zs, c='blue', marker='o')
            if connections:
                for start_idx, end_idx in connections:
                    self.ax.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], 
                                 [zs[start_idx], zs[end_idx]], 'ro-')
            self.ax.view_init(elev=10, azim=10)
            plt.pause(0.1)  # Pause to update the plot

    @staticmethod
    def extract_coordinates(landmarks):
        """Extract x, y, z coordinates from landmarks."""
        ys = [landmark.x for landmark in landmarks.landmark]
        zs = [-landmark.y for landmark in landmarks.landmark]  # Invert y for display
        xs = [landmark.z for landmark in landmarks.landmark]
        return xs, ys, zs

    def run_pose_detection(self):
        """Main loop for pose detection and visualization."""
        with mp.solutions.pose.Pose(
            static_image_mode=False, model_complexity=2, smooth_landmarks=True,
            enable_segmentation=False, min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:

            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise IOError("Cannot open webcam")

            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image = self.process_image(image, pose)
                cv2.imshow('MediaPipe Pose', image)
                if cv2.waitKey(5) & 0xFF == 27:  # Exit loop if 'ESC' is pressed
                    break

            cap.release()
            cv2.destroyAllWindows()  # Clean up OpenCV windows
            plt.ioff()  # Turn off interactive mode for Matplotlib
            plt.close(self.fig)  # Close the Matplotlib figure

    def process_image(self, image, pose):
        """Process each image frame to detect and display pose landmarks."""
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False  # Optimize performance by making image read-only
        results = pose.process(image)  # Apply pose detection

        # Prepare the image for displaying
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(
                image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))
            # Update the 3D plot with new landmark data
            self.draw_matplotlib_landmarks(results.pose_world_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
            # Publish the landmarks to a ROS topic
            self.plot_landmarks_and_publish(results.pose_world_landmarks)
        return image

    def plot_landmarks_and_publish(self, landmarks):
        """Prepare and publish pose landmarks as ROS messages."""
        landmarks_labels = {
            11: "left_shoulder", 12: "right_shoulder", 13: "left_elbow", 14: "right_elbow",
            15: "left_wrist", 16: "right_wrist", 17: "left_pinky", 18: "right_pinky",
            19: "left_index", 20: "right_index", 21: "left_thumb", 22: "right_thumb",
            23: "left_hip", 24: "right_hip",
        }
        pose_landmark_msg = PoseLandmark()
        pose_landmark_msg.label = []
        pose_landmark_msg.point = []

        # Fill the message with landmarks data
        for idx, landmark in enumerate(landmarks.landmark):
            if idx in landmarks_labels:
                label = landmarks_labels[idx]
                pose_landmark_msg.label.append(label)
                
                point = Point(x=landmark.x, y=landmark.y, z=landmark.z)
                pose_landmark_msg.point.append(point)

        # Publish the landmarks to the ROS topic
        self.publisher_.publish(pose_landmark_msg)
        self.get_logger().info(f'Publishing: {pose_landmark_msg}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS client library
    node = PoseDetectionPublisher()  # Create the ROS node
    node.run_pose_detection()  # Run the main loop for pose detection
    node.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()