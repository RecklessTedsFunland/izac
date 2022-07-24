---
title: ROS2 Command Cheatsheet
date: 5 Oct 2019
image: "https://index.ros.org/doc/ros2/_images/Nodes-TopicandService.gif"
image-height: "400px"
---

![](ros2.png)

## Commands

Most of the commands have changed for ros2:

- rosbag: 
    - `ros2 bag record <topic1> <topic2>`
    - `ros2 bag play <bag_file>`
    - `ros2 bag info <bag_file>`
- rostopic: 
    - `ros2 topic list`
    - `ros2 topic echo|bw <topic>`
- image_view: `ros2 run rqt_image_view rqt_image_view`
- rviz2: `ros2 run rviz2 rviz2`
- rosmsg: `ros2 interface show std_msgs/Bool` -> `bool data`
- rossrv: `ros2 interface show std_srvs/srv/Trigger` -> 
```
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

- Find out what nodes are running: `ros2 node list`
```
/listener
/talker
```
- See node services, pub/sub, and actions: `ros2 node info <node>`
```
/talker
  Subscribers:

  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

# References

- github: [ros2 cheatsheet](https://github.com/RecklessTedsFunland/ros2_cheats_sheet)
- ros answers: [ROS2 equivalents to ROS1 roscd, rosmsg rossrv...](https://answers.ros.org/question/358573/ros2-equivalents-to-ros1-roscd-rosmsg-rossrv/)
- [ROS2 colcon tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- [launch system for cpp and python](https://index.ros.org/doc/ros2/Tutorials/Launch-system/)
