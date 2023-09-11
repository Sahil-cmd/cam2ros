# cam2ros

`cam2ros` is a ROS 2 package that provides a simple way to capture video feed from a webcam or an external camera and publish it as ROS 2 Image messages. This package is written in Python and utilizes OpenCV to interface with the webcam or a connected external camera.

## Table of Contents
- [Features](#features)
- [Dependencies](#dependencies)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Maintainers](#maintainers)

## Features

- Capture video feed from the default webcam device.
- Capture video feed from connected external cameras
- Publish the captured frames as `sensor_msgs/msg/Image` messages.

## Dependencies

- ROS 2 Foxy (might work with other distributions but tested with Foxy).
- OpenCV (Python bindings)
- Python 3.6 or newer
- ROS 2 `sensor_msgs` package

## Getting Started

### Installation

1. Navigate to your ROS 2 workspace's `src` directory:

    ```bash
    cd ~/ros2_ws/src/
    ```

2. Clone the repository:

    ```bash
    git clone [repository_url] cam2ros
    ```

3. Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select cam2ros
    ```

4. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

1. To run the cam node:

    ```bash
    ros2 run cam2ros cam_node
    ```

2. To launch the cam node using the launch file:

    ```bash
    ros2 launch cam2ros cam_launch.py
    ```

## Contributing

Contributions to `cam2ros` are welcomed! Whether it's bug reports, feature requests, or code contributions, all are appreciated. For major changes, please open an issue first to discuss what you'd like to change.

Please see the CONTRIBUTING.md file for guidance on how to contribute to this project.

## License

This project is licensed under the Apache License 2.0.

## Maintainers

- [Sahil Panjwani](mailto:panjwani_sahil@artc.a-star.edu.sg)
