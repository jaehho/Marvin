import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

    def plot_landmarks(landmarks, connections):
        ax.clear()
        if landmarks:
            xs = [landmark.x for landmark in landmarks.landmark]
            zs = [-landmark.y for landmark in landmarks.landmark]  # Invert y for z
            ys = [landmark.z for landmark in landmarks.landmark]

            ax.scatter(xs, ys, zs, c='blue', marker='o')
            if connections:
                for connection in connections:
                    start_idx, end_idx = connection
                    ax.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], [zs[start_idx], zs[end_idx]], 'ro-')

            ax.set_xlim3d(-1, 1)
            ax.set_ylim3d(-1, 1)
            ax.set_zlim3d(-1, 1)
            ax.set_xlabel('X')
            ax.set_ylabel('Z')
            ax.set_zlabel('Y')
        plt.pause(0.001)

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = pose.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))

            plot_landmarks(results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)

        cv2.imshow('MediaPipe Pose', image)
        if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
            break

    cap.release()
    cv2.destroyAllWindows()
    plt.ioff()
    plt.close(fig)