# ROS2 Python and Launch Files

![](ros2.png)

## Package Creation

- install packages with: `rosdep install --from-paths src -i -y`
- create a package with: `ros2 pkg create <pkg-name> --dependencies [deps]`

```
dummy_pkg
├── CMakeLists.txt
├── include
│   └── dummy_pkg
├── launch
│   └── dummy_launch.py
├── LICENSE
├── package.xml
├── readme.md
└── src
    └── dummy_node.cpp
```

### Launch File

`ros2 launch <package> <launch.py>`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtf_sensors',
            executable='rtf_imu',
            name='imu'
        ),
        # Node(
        #     package='rtf_urg',
        #     executable='rtf_lidar',
        #     name='lidar',
        #     parameters=[
        #         {'port': '/dev/tty.usbmodem14501'}
        #     ]
        # ),
    ])
```

```python
from launch import LaunchDescription
from launch.substitutions import DeclareLaunchArgument, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        Node(
            package='demo_nodes_cpp',
            node_executable='talker',
            output='screen',
            node_name=[LaunchConfiguration('node_prefix'), 'talker']),
    ])
```

# References

- ros answers: [ROS2 equivalents to ROS1 roscd, rosmsg rossrv...](https://answers.ros.org/question/358573/ros2-equivalents-to-ros1-roscd-rosmsg-rossrv/)
- [ROS2 colcon tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- [launch system for cpp and python](https://index.ros.org/doc/ros2/Tutorials/Launch-system/)
