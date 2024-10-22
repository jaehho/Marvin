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

## Contributing

Contributions to Project Marvin are welcome. Please follow the standard GitHub pull request process to submit your contributions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
