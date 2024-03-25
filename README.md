# Project Marvin

## Overview

Project Marvin is an advanced robotic system built using ROS 2 Humble, visualizing real-time robotic manipulation and pose detection in Rviz2 and Gazebo Fortress. It captures human pose landmarks using a camera, processes these landmarks to determine specific joint angles (e.g., shoulder and elbow angles), and maps these angles to the joints of a robotic manipulator arm, specifically designed to mimic human arm movements. The project integrates Mediapipe for pose detection and OpenCV for image processing, operating on an Ubuntu 22.04 system.

### Components

- **Custom Interfaces (`PoseLandmark.msg`)**: Defines a ROS 2 message format for publishing pose landmarks detected from the video feed.
- **Pose Detection Node (`poseDetection.py`)**: Captures video, detects human pose landmarks using MediaPipe, and publishes them as `PoseLandmark` messages.
- **Manipulator Arm Control Nodes**: Includes nodes like `shoulderFlexion.py` and `shoulderAdduction.py` that subscribe to `PoseLandmark` messages, calculate joint angles, and publish `JointState` messages to control the manipulator arm.
- **Robot Description (`marvin.urdf`)**: Provides the URDF model for the Marvin manipulator arm, which is visualized in RViz and can be simulated in Gazebo.
- **RViz and Gazebo Launch Configuration (`launch/full.launch.py`)**: Launches the system including pose detection nodes, manipulator arm control nodes, RViz for visualization, and Gazebo for simulation.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8 or later
- OpenCV
- MediaPipe
- Matplotlib

Ensure ROS 2 and the necessary dependencies are installed on your system.

## Installation

1. Clone this repository to your local machine:

   ```bash
   git clone https://github.com/jaehho/Marvin.git
   ```

2. Navigate to the project directory and install the required Python libraries:

   ```bash
   pip3 install opencv-python
   pip3 install mediapipe
   pip3 install matplotlib
   ```

3. Build the ROS 2 workspace:

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

This will start the pose detection node, manipulator arm control nodes, and launch RViz and Gazebo for visualization and simulation, respectively.

## Contributing

Contributions to Project Marvin are welcome. Please follow the standard GitHub pull request process to submit your contributions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
