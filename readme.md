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
. install/setup.bash
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

## Transformation Inspection TF2

### 1) Viewframes to produce a pdf
```
ros2 run tf2_tools view_frames
```
### 2) TF2 Echo
In a terminal run the following when you are publisher is already running:
```
ros2 run tf2_ros tf2_echo world talk
```

## Bag Files

### 1) Recording a bag file:

Run the following command to record a bag for all topics
```
ros2 bag record -a -o saved_bag
```

### 2) Information on bag file:

Run the following command to get information on a bag file
```
ros2 bag info saved_bag
```

### 3) To play the bag file:

Run the following command to play the bag file
```
ros2 bag play saved_bag
```

### 4) Record a bag file using launch file:

```
ros2 launch cpp_pubsub pub_sub_service.launch.yaml freq:=1 record_bag:=true
```

### 5) Running ROS Test

Build the package:
```
colcon build --packages-select cpp_pubsub
```

Run the test:

```
colcon test --event-handlers console_direct+ --packages-select cpp_pubsub
```
