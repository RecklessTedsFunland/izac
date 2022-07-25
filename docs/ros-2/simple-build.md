---
title: Simple Build
date: 22 July 2022
---

## Python and C++ Packages

1. `. /opt/ros/humble/setup.zsh`
1. `mkdir -p ~/ros2_ws/src`
2. `cd ros2_ws/src`
3. Create packages
    - `ros2 pkg create --build-type ament_python my_package`
        - Edit package
    - `ros2 pkg create --build-type ament_cmake custom_interfaces`
        - Edit interface package
        - Interfaces (Msgs/Srvs) have to use CMake
5. `cd ../..` (now in `ros2_ws`)
6. `rosdep install -i --from-path src --rosdistro Humble -y`
7. `colcon build`
8. `. install/setup.bash`
9. `ros2 run my_package some_node`

## Clean Build Artifacts

`rm -fr build install log`

## References

- [Building a ROS2 Package](https://medium.com/@thehummingbird/building-a-ros2-project-part-1-a2c02d6ac3d8)
