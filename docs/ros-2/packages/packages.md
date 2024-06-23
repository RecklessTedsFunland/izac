# ROS2 Packages

## New

```bash
ros2 pkg create --build-type <ament_cmake|ament_python> [--license MIT] [--dependencies ros_pkg ...] -- <pkg_name>
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp
```
