---
title: Using Navigation2 in ROS2
date: 19 Jan 2021
---

# Overview of Nav2

- Install
    - `sudo apt install ros-foxy-navigation2`
    - `sudo apt install ros-foxy-nav2-bringup`
- Behavior Trees replace `move_base`
    - XML governs complex behavior
    - Supposed to be more efficient and more human understandable for complex problems than a FSM would be
- Planners and recovery are new ROS actions
    - ported `navfn` (global) planner, Dijkstra or A*
    - ported `dwb` (local) planners
- Lifecycle nodes add stability
    - Built in state machine in the node: unconfigured, inactive, active, initialized
- `lifecycle_manager` controls startup and shutdown of nav2 nodes
- `amcl` map/localizer (particle filter) and `map_server` ported
- `nav2_core` has a plugin interface
- `nav2_bringup` makes it easy to start the nav2 stack with good defaults ... good reference

## Sensor Fusion

Still learning ...

- Install
    - `sudo apt install ros-foxy-robot-localization`
- Frames
    - `earth (ECEF)` -> `map` -> `odom` -> `base_link` -> `sensor(s)`
    - `\map` is the world frame (obviously world-fixed frame)
        - `\odom` is child of `\map` is a world-fixed frame
            - `\base_link` is a child of `\odom` and rigidly attached to robot
- Transforms
    - `\map` -> `\odom` handelled by the `\tf` via a positioning system (localization, mapping, SLAM)
        - Accounts for the robot's global pose
        - Tracks the offset needed to compensate for overall sensor drift as the robot drives around, 
          allowing the robot to more robustly create global path plans
        - Is discontinuous (jumps around)
        - Is published by the AMCL node (or any other global pose estimators!)
    - `\odom` -> `\base_link` handeled by the `odom` message
        - Accounts for the robot's local pose
        - Is best for local obstacle avoidance
        - Is continuous, but drifts over time due to sensor drift
        - Is published by the odometry node (which should be taking into account encoders, IMUs and 
          other sensors like laser scan matching nodes, etc.)
- `robot_localization` (see docs folder)
    - Has EKF and UKF nonlinear estimators
        - `ekf_localization_node` and `ukf_localization_node`
        - `navsat_transform_node` for GPS integration
    - All state estimation nodes track the 15-dimensional state of the vehicle: (X, Y, Z, roll, pitch, yaw, $\dot{X}$, $\dot{Y}$, $\dot{Z}$, $\dot{roll}$, $\dot{pitch}$, $\dot{yaw}$, $\ddot{X}$, $\ddot{Y}$, $\ddot{Z}$)
    - `nav_msgs/Odomety`
        - All pose data is between `frame_id` and the `world_frame` (typically `map` or `odom`)
        - Alll twist data is transformed into `base_link_frame` (typically `base_link`)
    - `sensor_msgs/Imu`
        - expects all messages in ENU, NED not supported (TBD)
- Things to remeber
    - IMU Orientation [[ref](http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html#)] 
        - Orientation: x (forward), y (left), z (up) ![](https://miro.medium.com/max/501/1*MboVHLeGk0e4bVoJdjUdmg.png)
        - **Not all** AHRS software follow this convention 
        - IMU right side up on a flat surface will measure **+** 9.81 m/s/s on z-axis
        - Roll **+** 90 deg (left side up), will measure **+** 9.81 m/s/s on y-axis
        - Pitch **+** 90 deg (front down), will measure **-** 9.81 m/s/s on x-axis

## Behavior Trees

This is a generic overview

- Status: SUCCESS, FAILURE, or RUNNING
- Control Nodes
    - Sequence (->): SUCCESS if all children return SUCCESS
    - Fallback (?): SUCCESS when first child returns SUCCESS
    - Parallel (double arrow): SUCCESS if any child returns SUCCESS
    - Decorator ($\del$): modifies the return state of a child
- Execution Nodes
    - Leaves in the tree
    - Action: executes an action/command
    - Condition: checks for some kind of condition

# References

- [nav2 docs](https://navigation.ros.org/index.html)
- github: [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox)
- github: [`robot_localization`](https://github.com/cra-ros-pkg)
    - [example params for ekf](https://github.com/cra-ros-pkg/robot_localization/blob/kinetic-devel/params/ekf_template.yaml)
- ROSCon 2019: [Navigation 2 Overview](https://vimeo.com/378682188)
- REP-0105: [Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Behavior Tree](https://www.behaviortree.dev)
- [9DoF Razor IMU with ROS (Part I)](https://medium.com/@leofaf/sparkfun-9dof-razor-imu-m0-with-ros-d9e1c5209f9e)
- [Designing AI Agents’ Behaviors with Behavior Trees](https://towardsdatascience.com/designing-ai-agents-behaviors-with-behavior-trees-b28aa1c3cf8a)
