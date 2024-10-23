# Project Marvin

## Overview

## Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/)

## Installation

1. Clone this repository to your local machine:

   ```bash
   git clone https://github.com/jaehho/Marvin.git
   ```

## Running Project Marvin

## Troubleshooting

### Running Docker on Mac

To run a ros2 Jazzy container

```zsh
docker run --platform linux/amd64 --name ros -v /Users/sophiaklymchuk/ros2-ws:/root/ros2-ws -d osrf/ros:jazzy-desktop-full tail -f /dev/null```

To connect to above container

```zsh
docker exec -it ros bash
```

### Docker permission denied

```bash
permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Get "http://%2Fvar%2Frun%2Fdocker.sock/v1.47/containers/json?all=1&filters=%7B%22label%22%3A%7B%22com.docker.compose.config-hash%22%3Atrue%2C%22com.docker.compose.project%3Dmarvin%22%3Atrue%7D%7D": dial unix /var/run/docker.sock: connect: permission denied
```

```bash
sudo usermod -aG docker $USER
```

then logout and login

### Access to USB ports

```bash
sudo usermod -aG dialout $USER
```

then logout and login

## Contributing

Contributions to Project Marvin are welcome. Please follow the standard GitHub pull request process to submit your contributions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
