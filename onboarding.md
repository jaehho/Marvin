# Onboarding

## Prerequisites

1. Install ROS2 Humble

    <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

2. Install Colcon

    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

## Setup

1. Clone repository

    ```bash
    git clone https://github.com/jaehho/Marvin.git
    ```

2. Build Packages

    ```bash
    source /opt/ros/humble/setup.bash
    cd ros2_ws
    colcon build --symlink-install
    ```

3. Install Python modules

    ```bash
    sudo apt install python3-tk python3-pip
    sudo apt-get install x11-xserver-utils # for xrandr
    pip install matplotlib mediapipe opencv-python
    ```

## Usage

1. Source current workspace

    ```bash
    . install/setup.bash
    ```

2. Full Launch

    ```bash
    ros2 launch marvin full.launch.py
    ```

## Troubleshooting

### Cmake Error after colcon build

```bash
CMake Error at CMakeLists.txt:2 (project):
  No CMAKE_CXX_COMPILER could be found.

  Tell CMake where to find the compiler by setting either the environment
  variable "CXX" or the CMake cache entry CMAKE_CXX_COMPILER to the full path
  to the compiler, or to the compiler name if it is in the PATH.
```

**Solution**: Ensure C++ compiler is Installed

```bash
sudo apt install build-essential
```

---

### Missing xacro or rviz2

You may be missing packages if you installed ROS-Base Install (Bare Bones)

**Solution**: Install missing packages

```bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-rviz2
```

---

### Can't create Python virtual environment

```bash
The virtual environment was not created successfully because ensurepip is not
available.  On Debian/Ubuntu systems, you need to install the python3-venv
package using the following command.

    apt install python3.10-venv

You may need to use sudo with that command.  After installing the python3-venv
package, recreate your virtual environment.

Failing command: /home/jaeho/Marvin/ros2_ws/.venv/bin/python3
```

**Solution**: Install `python3.10-venv`

```bash
    sudo apt install python3.10-venv
```
