# Marvin

The Marvin project aims to create a real-time system for tracking human pose movements using MediaPipe and mirroring these movements with two robotic arms. It integrates cutting-edge pose estimation technologies with robotic manipulation, providing a seamless interface for human-robot interaction.

## Overview

The core of this project is built upon the integration of several technologies:
- **MediaPipe**: For real-time, accurate pose estimation.
- **OpenCV**: For video capture and processing.
- **ROS2**: As the operating system for processing pose information and controlling the Open-Manipulator-X robotic arms.

The end goal is to enable the robotic arms to mimic human movements in real-time, which can have applications in teleoperation, education, and surgical robotics.

## Hardware

- Open-Manipulator-X x2
- PC running Ubuntu 22.04 or Raspberry Pi 4

## Software

- Ubuntu 22.04
- Python 3.10
- ROS2 Humble
- OpenCV
- MediaPipe
- Gazebo Fortress
- Rviz2

Ensure software is up to date and respective dependencies installed.

## Installation

Clone this repository to your local machine:

```bash
git clone https://github.com/GitJaehoCho/Marvin.git
```

Install the required Python libraries:

```bash
pip3 install opencv-python
pip3 install mediapipe
pip3 install matplotlib
```

## Usage

1. **Start the Pose Estimation Script**:

   Run the script to begin pose tracking and visualization:

   ```bash
   python main.py
   ```

   This will open three windows: one displaying the video feed with pose landmarks overlaid, another showing the 3D visualization and lastly the JSON data of selected landmarks.

2. **ROS2 Integration**:

   (in development)

## Contributing

We welcome contributions to improve the project's accuracy, efficiency, and usability. Please follow the standard pull request process for your contributions.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---
