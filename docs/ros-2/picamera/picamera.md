# Pi Camera v2.1

- [ROS2 libcamera node](https://github.com/christianrauch/camera_ros): `sudo apt install ros-$ROS_DISTRO-camera-ros`
  - **WARNING:** doesn't seem to be working for jazzy right now (2024-06-20)
- Also `image_tools`: `ros2 run image_tools cam2image` reads `/dev/video0` and publishes on `/image`
