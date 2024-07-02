![](https://foxglove.dev/images/logo.png)

# Foxglove

## Install and Run

- docs: https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge 
- Connect using website or desktop app to `ws://<hostip>:8765`

```bash
apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Launch File

There are many more parameters available, but these seem to work

```python
Node(
    package="foxglove_bridge",
    executable="foxglove_bridge",
    name="foxglove_bridge",
    parameters=[
        {"port": 8765},
        {"address": "0.0.0.0"},
        {"tls": False},
        {"topic_whitelist": [".*"]},
        {"send_buffer_limit": 10000000},
        {"use_sim_time": False},
        {"num_threads": 0}
    ]
)
```

[XML launch file for reference](https://github.com/foxglove/ros-foxglove-bridge/blob/main/ros2_foxglove_bridge/launch/foxglove_bridge_launch.xml)
