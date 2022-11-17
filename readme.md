[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
# Beginner Tutorial - ENPM808X

## Overview

This repository contains basic publisher/subscriber nodes

## Dependencies/Assumptions

- OS: Ubuntu 20.04
- ROS2: Humble
- Packages:
    - ament
    - CMake
    - colcon
    - rclcpp
    - std_msgs

## Build and Run:
- Source ROS2 Underlayh (depends on how you've installed ROS2)
- Navigate to your < ROS2 Workspace >/src/

### Build:
```
cd src/
git clone https://github.com/aniruddhbalram97/beginner_tutorials.git
cd ..
colcon build --packages-select cpp_pubsub
```

### Run:

### Run the publisher:

```
# Open a terminal and navigate to your ROS2 Workspace
. install/setup.bash
ros2 run cpp_pubsub talker
```

### Run the subscriber:

```
# Open a terminal and navigate to your ROS2 Workspace
. install/setup.bash
ros2 run cpp_pubsub listener
```

## Results
The results are present in the 'results' directory

    - cpplint.txt
    - cppcheck.txt
