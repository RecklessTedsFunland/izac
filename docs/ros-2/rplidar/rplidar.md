![](https://i.pinimg.com/564x/7d/23/b8/7d23b8b828394c9fbc2396f49262c98e.jpg)

# ROS2 and the RPLidar

Ok, the design is a total piece of crap! The white serial/power cable coming out of the
bottom means you can't set it down flat ... what kind of **special idiot** designed this 
sensor?

> **ISSUE:** why does it never stop spinning? I have to disconnect the power
> to shut the damn thing off ... crap!

## Install and Running

Ok, rants aside, it seems to work nicely

- Install drivers: `sudo apt install ros-${ROS_DISTRO}-rplidar-ros`
    - Then: `source /opt/ros/${ROS_DISTRO}/setup.bash`
- Run lidar node to get data: `ros2 run rplidar_ros rplidarNode`
- Run client node to see data: `ros2 run rplidar_ros rplidarNodeClient`
