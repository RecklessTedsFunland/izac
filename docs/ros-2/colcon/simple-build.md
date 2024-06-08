# Simple Build

## Python and C++ Packages

1. `. /opt/ros/humble/setup.bash`
1. `mkdir -p ~/ros2_ws/src`
2. `cd ros2_ws/src`
3. Create packages
    - **Python3:** `ros2 pkg create --build-type ament_python my_package`
        - Edit package
    - **C++:** `ros2 pkg create --build-type ament_cmake custom_interfaces`
        - Edit interface package
        - Interfaces (Msgs/Srvs) have to use CMake
5. `cd ../..` (now in `ros2_ws`)
6. `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y`
7. `colcon build`
8. `. install/setup.bash`
9. `ros2 run my_package some_node`

## Example

```
source /opt/ros/foxy/setup.bash
mkdir -p ros2/src
cd ros2
git clone https://github.com/ros2/examples src/examples
cd src/examples
git checkout $ROS_DISTRO
cd ../..
git clone https://github.com/ros2/example_interfaces.git src/example_interfaces
colcon build --symlink-install
```

> **Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

So now you have a directory structure that looks like:

```
ros2
|-bulid
|-install
| +-setup.bash
|-log
+-src
  |-examples
  +-example_interfaces
```

Now run `colcon test` to make sure all is well.

Now run `source install/setup.bash` (best) or alternatively `source install/local_setup.bash`. The difference is:

- The `local_setup.<ext>` script sets up the environment for all package in the prefix path where 
that script is. It doesn't include any parent workspaces.
- The `setup.<ext>` script on the other hand sources the `local_setup.<ext>` script for *all workspaces*
which were sourced in the environment when this workspace was built. And then it also sources the sibling 
`local_setup.<ext>` script.

Now run a pub/sub, you need some windows (having run `source install/setup.zsh` in them): 

- `ros2 run examples_rclcpp_minimal_publisher publisher_member_function`
- `ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function`
- `ros2 run examples_rclpy_minimal_subscriber subscriber_member_function`

Notice, there is a mixture of cpp (`rclcpp`) and python (`rclpy`) in these ... neat!

## Clean Build Artifacts

`rm -fr build install log`

## Args

- Pass `cmake` any args: `--cmake-args -DCMAKE_PREFIX_PATH=/home/bob/here -DCMAKE_SOMETHING_ELSE`
- Clean first: `--cmake-clean-first`

## References

- Ros2 Tutorial: [Using Colcon to Build Packages](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- rosanswers: [What is the difference between local_setup.bash and setup.bash](https://answers.ros.org/question/292566/what-is-the-difference-between-local_setupbash-and-setupbash/)
- [Building a ROS2 Package](https://medium.com/@thehummingbird/building-a-ros2-project-part-1-a2c02d6ac3d8)
- ROS2 Tutorial: [Creating a Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- ROS2 Tutorial: [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- ROS2 Tutorial: [Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- ROS2 Tutorial: [Implementing custom interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
