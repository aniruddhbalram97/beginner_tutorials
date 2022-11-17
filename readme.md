## Beginner Tutorial - ENPM808X

### Description

This repository contains basic publisher/subscriber nodes

### Dependencies

- ROS2: 
    - Humble
- Packages:
    - ament
    - CMake
    - colcon
- Minimum Requirement
    - Must have a ROS2 workspace setup before running this package

### Steps to Install 

Navigate to your < ROS2 Workspace >/src/: 
```
# source the underlay ros2 - depends on how you've installed ros
# source the overlay ros2 - depends on how you've setup ros workspace

git clone https://github.com/aniruddhbalram97/beginner_tutorials.git
git checkout ros_pub_sub
# go to base of the workspace and build package
cd ..
colcon build --packages-select cpp_pubsub
. install/setup.bash

```

### Steps to Run:

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

### Results
The results are present in the 'results' directory
    - cpplint.txt
    - cppcheck.txt
