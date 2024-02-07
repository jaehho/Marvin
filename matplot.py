import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize MediaPipe Pose.
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False,
                    model_complexity=1,
                    smooth_landmarks=True,
                    enable_segmentation=False,
                    smooth_segmentation=True,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Capture video from the webcam.
cap = cv2.VideoCapture(0)

# Initialize Matplotlib figure for 3D plotting.
plt.ion()  # Enable interactive mode.
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Function to update the 3D plot with landmarks and connections, flipped 90 degrees around the x-axis.
def plot_landmarks(landmarks, connections, ax):
    ax.clear()  # Clear previous landmarks to update the plot.
    if landmarks:
        # Adjust landmark coordinates for 90-degree flip around the x-axis.
        xs = [landmark.x for landmark in landmarks.landmark]
        zs = [-landmark.y for landmark in landmarks.landmark]  # Invert y for z to simulate rotation
        ys = [landmark.z for landmark in landmarks.landmark]  # Use original z as new y
        
        # Plot landmarks with adjusted coordinates.
        ax.scatter(xs, ys, zs, c='blue', marker='o')
        
        # Plot connections with adjusted coordinates.
        if connections:
            for connection in connections:
                start_idx, end_idx = connection
                ax.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], [zs[start_idx], zs[end_idx]], 'ro-')
        
        # Adjust axes limits and labels for the flipped visualization.
        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)  # Adjusted to accommodate the flipped coordinates
        ax.set_zlim3d(-1, 1)
        ax.set_xlabel('X')
        ax.set_ylabel('Z')  # Y and Z labels are swapped
        ax.set_zlabel('Y')
    plt.pause(0.001)  # Pause to update the plot.

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
        # Draw the pose annotation on the image.
        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
            connection_drawing_spec=mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))
        
        # Update the plot.
        plot_landmarks(results.pose_landmarks, mp_pose.POSE_CONNECTIONS, ax)

    # Display the resulting frame
    cv2.imshow('MediaPipe Pose', image)

    if cv2.waitKey(5) & 0xFF == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
plt.ioff()  # Turn off interactive mode.
plt.close(fig)  # Close the figure.

#updated