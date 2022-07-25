---
title: ROS2 File Structure
date: 24 July 2022
---

`~/ros2_ws/src/`

- my_robot 
    - This package is a metapackage. A metapackage doesn’t contain anything except a list of dependencies to other packages. You can use a metapackage to make it easier to install multiple related packages at once. Example.
- my_robot_base 
    - This package is used to control the motors of your robot. Example.
- my_robot_bringup 
    - Put the ROS 2 launch files that bring up the robot inside this folder. Example.
- my_robot_description 
    - Contains the URDF and mesh files of your robot. Example.
- my_robot_gazebo 
    - Configuration and launch files for spawning the robot in Gazebo. Example
- my_robot_kinematics 
    - Your forward and inverse kinematics algorithms go here. Example.
- my_robot_localization 
    - Files for localizing in an environment. Example.
- my_robot_manipulation 
    - Contains algorithms for manipulating objects in the environment. Example.
- my_robot_moveit_config 
    - Configuration files using the MoveIt framework. Example.
- my_robot_msgs 
    - Contains custom messages, services, and actions. Example.
- my_robot_navigation 
    - Contains configuration and launch files for the ROS 2 Navigation Stack. Example.
- my_robot_teleop 
    - A node for manually tele-operating a robot using a keyboard, joystick, game console controller, etc. Example.
- my_robot_tests 
    - A package used for system tests. Example.
- my_robot_rviz_plugins 
    - RViz specific plugins go here. Example.
