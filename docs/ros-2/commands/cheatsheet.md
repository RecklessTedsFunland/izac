---
title: ROS2 Command Cheatsheet
date: 5 Oct 2019
---

![](ros2.png)

## Commands

Most of the commands have changed for ros2:

- rosrun:
    - `ros2 run <package_name> <executable_name>`
- ros lifecycle to kill a node:
    - `ros2 lifecycle set <nodename> shutdown`
- rosbag: 
    - `ros2 bag record <topic1> <topic2>`
    - `ros2 bag play <bag_file>`
    - `ros2 bag info <bag_file>`
- rostopic: 
    - `ros2 topic list`
    - `ros2 topic echo|bw|hz|info <topic>`
    - `ros2 topic pub <topic_name> <msg_type> '<args>'`
        - `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
        - `ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
- rosaction:
    - `ros2 action list`
    - `ros2 action send_goal <action_name> <action_type> <values>`
- rosparameters:
    - `ros2 param list`
    - `ros2 param get <node_name> <parameter_name>`
    - `ros2 param set <node_name> <parameter_name> <value>`
    - `ros2 param dump <node_name>`
        - can also be redirected to file: `ros2 param dump /turtlesim > turtlesim.yaml`
    - `ros2 param load <node_name> <parameter_file>`
        - Read-only parameters can only be modified at startup and not afterwards and will give warnings for the "qos_overrides" parameters. Read-only must be set at launch:
            - `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`
- image_view: `ros2 run rqt_image_view rqt_image_view`
- rviz2: `ros2 run rviz2 rviz2`
- rosmsg: `ros2 interface show std_msgs/Bool` -> `bool data`
- rossrv: `ros2 interface show std_srvs/srv/Trigger` -> 
    ```
    ---
    bool success   # indicate successful run of triggered service
    string message # informational, e.g. for error messages
    ```
- rosnode:
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
