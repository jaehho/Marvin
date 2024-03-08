# Project Marvin

## Overview

Project Marvin is an advanced robotic system built using ROS 2. It integrates real-time pose detection with the control of a manipulator arm, visualizing the pose detection results in RViz. This system captures human pose landmarks using a camera, processes these landmarks to determine specific joint angles (e.g., shoulder angles), and maps these angles to the joints of a robotic manipulator arm.

### Components

- **Custom Interfaces (`PoseLandmark.msg`)**: Defines a ROS 2 message format for publishing pose landmarks detected from the video feed.
- **Pose Detection Node (`poseDetection.py`)**: Captures video, detects human pose landmarks using MediaPipe, and publishes them as `PoseLandmark` messages.
- **Shoulder Angle Joint Node (`shoulderAngleJoint.py`)**: Subscribes to `PoseLandmark` messages, calculates shoulder joint angles, and publishes `JointState` messages.
- **Robot Description (`urdf/open_manipulator.urdf.xacro`)**: Provides the URDF model for the manipulator arm.
- **RViz Visualization (`launch/full.launch.py`)**: Launches the system, including the pose detection node, the shoulder angle joint node, and RViz configured to visualize the robot model and pose landmarks.

## Prerequisites

- ROS 2 (Humble)
- Python 3.8 or later
- OpenCV
- MediaPipe
- matplotlib

Ensure ROS 2 and the necessary dependencies are installed on your system.

## Installation

1. Clone this repository to your local machine:

   ```bash
   git clone https://github.com/GitJaehoCho/Marvin.git
   ```

2. Install the required Python libraries:

   ```bash
   pip3 install opencv-python
   pip3 install mediapipe
   pip3 install matplotlib
   ```

3. Build the workspace

   ```bash
   cd ros2_ws/
   colcon build
   ```

## Running Project Marvin

1. **Source the Workspace**: Source the workspace to utilize the package executables

   ```bash
   source install/setup.bash
   ```

2. **Launch the System**: Use the provided launch file to start all nodes and RViz:
   ```bash
   ros2 launch marvin full.launch.py
   ```
This command launches the pose detection node, the shoulder angle joint node, and RViz for visualization.

**Pose Detection**: Stand in front of the camera to have your pose detected. The system captures video frames, detects human pose landmarks, and visualizes them in RViz.

**Shoulder Angle Mapping**: The system calculates shoulder joint angles from the detected pose landmarks and maps these angles to the robotic manipulator's joints, simulating the upper body movements.

**RViz Visualization**: Observe the manipulator arm in RViz, which moves according to the detected human shoulder angles.

## Onboarding for Developers

To contribute to Project Marvin, you'll need a basic understanding of ROS 2, experience with Python, and familiarity with pose detection technologies.

- **ROS 2**: Get familiar with ROS 2 concepts, including nodes, messages, and launch files.
- **Python Development**: Ensure you're comfortable with Python, especially in the context of ROS 2 nodes and OpenCV for image processing.
- **Pose Detection**: Understand how MediaPipe is used for pose detection and how to process its output.

## Project Structure

- `launch/full.launch.py`: Launch file to start the system.
- `poseDetection.py`: Node for detecting human pose.
- `shoulderAngleJoint.py`: Node for processing pose landmarks and controlling the manipulator.

## Contributing

We welcome contributions to improve the project's accuracy, efficiency, and usability. Please follow the standard pull request process for your contributions.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---