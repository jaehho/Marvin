# Onboarding Page

**Last Updated:** February 10, 2024

Flash Ubuntu 22.04

- Clone Repository (<https://github.com/GitJaehoCho/Marvin>)

    ```bash
    sudo apt install git
    ```

    Install gh

    ```bash
    type -p curl >/dev/null || (sudo apt update && sudo apt install curl -y)
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
    && sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
    && sudo apt update \
    && sudo apt install gh -y
    ```

    Install/Update Curl

    ```bash
    sudo apt-get install curl
    sudo apt-get update
    sudo apt-get upgrade curl
    ```

    Clone Marvin Repository

    ```bash
    gh repo clone GitJaehoCho/Marvin
    ```

- Install Libraries

    ```bash
    sudo apt-get install python3-pip
    ```

  - Install OpenCV ([OpenCV](https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html)) ([potentially unnecessary](https://pypi.org/project/opencv-python/))

    ```bash
    sudo apt-get install python3-opencv -y
    sudo apt-get install cmake -y
    sudo apt-get install gcc g++ -y
    sudo apt-get install python3-dev python3-numpy -y
    sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev -y
    sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev -y
    sudo apt-get install libgtk-3-dev -y
    sudo apt-get install libpng-dev -y
    sudo apt-get install libjpeg-dev -y
    sudo apt-get install libopenexr-dev -y
    sudo apt-get install libtiff-dev -y
    sudo apt-get install libwebp-dev -y
    
    # Necessary for Mediapipe Framework(maybe unnecessary for Mediapipe Library
    git clone https://github.com/opencv/opencv.git
    cd opencv/
    mkdir build
    cd build/
    cmake ../
    ```

  - Install MediaPipe Framework (Not necessary? - might only need to install Mediapipe library)
    - Install Bazelisk

        [Bazelisk Release](https://github.com/bazelbuild/bazelisk/releases) ([bazelisk-linux-amd64](https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64))

        ```bash
        git clone --depth 1 https://github.com/google/mediapipe.git
        ```

  - Install Mediapipe

    ```bash
    pip3 install mediapipe
    ```

  - Install Matplotlib

    ```bash
    pip3 install -U matplotlib
    ```
