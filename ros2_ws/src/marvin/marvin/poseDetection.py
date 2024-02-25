import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import PoseLandmark
from geometry_msgs.msg import Point


class PoseDetectionPublisher(Node):

    def __init__(self):
        super().__init__('pose_detection_publisher')
        self.publisher_ = self.create_publisher(PoseLandmark, 'pose_landmarks', 10)
        
        plt.ion()  # Interactive mode on for Matplotlib.
        global fig
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')

        def matplot_landmarks(landmarks, connections):
            ax.clear()
            if landmarks:
                # Use World Landmark coordinates directly
                ys = [landmark.x for landmark in landmarks.landmark] # Switch axes to rotate view
                zs = [-landmark.y for landmark in landmarks.landmark] 
                xs = [landmark.z for landmark in landmarks.landmark]

                ax.scatter(xs, ys, zs, c='blue', marker='o')
                if connections:
                    for connection in connections:
                        start_idx, end_idx = connection
                        ax.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], [zs[start_idx], zs[end_idx]], 'ro-')

                # Adjust the limits and labels for a better 3D visualization
                ax.set_xlim3d(-0.5, 0.5)
                ax.set_ylim3d(-0.5, 0.5)
                ax.set_zlim3d(-0.5, 0.5)
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

            # Rotate the view
            elevation = 10  # change this value to adjust the up/down rotation
            azimuth = 10    # change this value to adjust the left/right rotation
            ax.view_init(elev=elevation, azim=azimuth)

            plt.pause(0.1)

        with mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=2,
            smooth_landmarks=True,
            enable_segmentation=False,
            smooth_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:

            mp_drawing = mp.solutions.drawing_utils

            # Attempt to capture video from webcam.
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise IOError("Cannot open webcam")

            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False # Prevent image processing from writing to the image, also saves time/memory
                results = pose.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.pose_landmarks:
                    # Draw the NORMALIZED landmarks on the image(NOT world landmarks for webcam overlay)
                    mp_drawing.draw_landmarks(
                        image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                        connection_drawing_spec=mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))

                    matplot_landmarks(results.pose_world_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # Use world landmarks for 3D graph view
                    self.plot_landmarks_and_publish(results.pose_world_landmarks)

                cv2.imshow('MediaPipe Pose', image)
                if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
                    break

            cap.release()
            cv2.destroyAllWindows()
            plt.ioff()
            plt.close(fig)

    def plot_landmarks_and_publish(self, landmarks):
        # Define a dictionary mapping landmark indices to their labels for landmarks 11 through 24
        landmarks_labels = {
            11: "left_shoulder",
            12: "right_shoulder",
            13: "left_elbow",
            14: "right_elbow",
            15: "left_wrist",
            16: "right_wrist",
            17: "left_pinky",
            18: "right_pinky",
            19: "left_index",
            20: "right_index",
            21: "left_thumb",
            22: "right_thumb",
            23: "left_hip",
            24: "right_hip",
        }
        
        # Prepare a PoseLandmark message
        pose_landmark_msg = PoseLandmark()
        pose_landmark_msg.label = []
        pose_landmark_msg.point = []

        if landmarks:
            for idx, landmark in enumerate(landmarks.landmark):
                # Append label if it exists in the dictionary
                if idx in landmarks_labels:
                    # Get the label from the dictionary
                    label = landmarks_labels[idx]
                    pose_landmark_msg.label.append(label)
                    
                    # Create a Point message for the landmark
                    point = Point()
                    point.x = landmark.x
                    point.y = landmark.y
                    point.z = landmark.z
                    pose_landmark_msg.point.append(point)

            # Publish the message
            self.publisher_.publish(pose_landmark_msg)
            self.get_logger().info(f'Publishing: {pose_landmark_msg}')


def main(args=None):
    rclpy.init(args=args)

    pose_detection_publisher = PoseDetectionPublisher()

    rclpy.spin(pose_detection_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()