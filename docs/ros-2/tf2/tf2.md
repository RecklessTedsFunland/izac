---
title: Using tf2
date: 7 Sept 2020
---

- **Publish:** `ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 foo bar`
    - *Note:* This publishes the transform from the parent (foo) to the child (bar), the numbers are X, Y, X, yaw, pitch, roll
- **Receiving:** `ros2 run tf2_ros tf2_echo foo bar`
    - This returns the position, rotation from foo to bar: 
    ```
    At time 0.0
    - Translation: [1.000, 2.000, 3.000]
    - Rotation: in Quaternion [-0.475, -0.076, 0.240, 0.843]
    ```
    
# References

- ROS Tutorial: [Using tf2 with ROS 2](https://index.ros.org/doc/ros2/Tutorials/tf2/)
