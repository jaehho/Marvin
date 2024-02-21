import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

def plot_landmarks(landmarks, connections):
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

    plt.pause(0.001)

# Initialize MediaPipe Pose with a context manager to ensure resources are released.
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

    plt.ion()  # Interactive mode on for Matplotlib.
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False # Prevent image processing from writing to the image, also saves time/memory
        results = pose.process(image)

        if results.pose_world_landmarks:
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

            # Filter the landmarks_data to only include selected landmarks with labels
            landmarks_data = [
                {
                    'label': landmarks_labels.get(index),
                    'x': landmark.x, 
                    'y': landmark.y, 
                    'z': landmark.z
                } 
                for index, landmark in enumerate(results.pose_world_landmarks.landmark) 
                if index in landmarks_labels
            ]
            global json_data
            json_data = json.dumps(landmarks_data, indent=2)


        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            # Draw the NORMALIZED landmarks on the webcam(NOT world landmarks for webcam overlay)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))

            plot_landmarks(results.pose_world_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # Use world landmarks for 3D graph view

        cv2.imshow('MediaPipe Pose', image)
        if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
            break

    cap.release()
    cv2.destroyAllWindows()
    plt.ioff()
    plt.close(fig)