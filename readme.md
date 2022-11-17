[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Beginner Tutorial - ENPM808X

## Overview
ROS Package which contains: 
    - Basic Publisher/Subscriber node with string output.
    - Service to add two numbers and modify strings output.

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

### Update the message through the client:

Run the talker and listener nodes on separate terminals and the following command on another terminal
```
ros2 run cpp_pubsub client 2 54
```
Notice change in the talker and listener terminals

### Run both nodes using launch file and modify parameter:
```
cd src/cpp_pubsub/launch
ros2 launch pub_sub_service_launch.yaml freq:=5.0
```
### Results:
The results are present in <your_package>/results folder.

### RQT Console:
Once both talker/listener nodes are running, run the following command:
```
ros2 run rqt_console rqt_console
```
