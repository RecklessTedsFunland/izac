# ROS2 Bag Files

- [ROS2 tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html), bag files now use foxglove's MCAP for storage by default
  - [foxglove blog post](https://foxglove.dev/blog/mcap-as-the-ros2-default-bag-format) about the change 



```bash
ros2 bag record [-o custom_filename] <topic0> [topic1] ...

# example with custom filename and 2 topics
ros2 bag record -o mystuff /turtle1/cmd_vel /turtle1/pose
```
```bash
ros2 bag info <bag_file_name>

# example
ros2 bag info rosbag2_2024_06_22-07_59_40

Files:             rosbag2_2024_06_22-07_59_40_0.mcap
Bag size:          1.3 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          25.31s
Start:             Jun 22 2024 07:59:40.761 (1719057580.761)
End:               Jun 22 2024 08:00:05.792 (1719057605.792)
Messages:          5510
Topic information: Topic: /imu | Type: sensor_msgs/msg/Imu | Count: 2504 | Serialization Format: cdr
                   Topic: /mag | Type: sensor_msgs/msg/MagneticField | Count: 2504 | Serialization Format: cdr
                   Topic: /press | Type: sensor_msgs/msg/FluidPressure | Count: 251 | Serialization Format: cdr
                   Topic: /temp | Type: sensor_msgs/msg/Temperature | Count: 251 | Serialization Format: cdr
Service:           0
Service information:
```
```bash
ros2 bag play <bag_file_name>
```
