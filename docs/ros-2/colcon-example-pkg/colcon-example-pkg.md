---
title: Example Package with Colcon
date: 2 Aug 2020
image: "https://i.pinimg.com/564x/94/22/fc/9422fc82bb5c347961c465c85cc91599.jpg"
image-height: "300px"
---

*Note:* I am using the `zsh` shell instead of `bash`.

Unfortunately the examples are shitty, overly verbose for no reason, leave stuff out and don't make sense.

1. Source the ros setup file from `/opt/ros/.../setup.zsh`
1. Create a workspace (`ros2`) and a source code folder (`ros2/src`)
1. Move into the workspace
1. Clone some example directories into `src`
    1. Make sure to check out the correct distro branch 
1. Build with examples `colcon`, the latest crappy ros build tool

```
source /opt/ros/foxy/setup.zsh
mkdir -p ros2/src
cd ros2
git clone https://github.com/ros2/examples src/examples
cd src/examples
git checkout $ROS_DISTRO
cd ../..
git clone https://github.com/ros2/example_interfaces.git src/example_interfaces
colcon build --symlink-install
```

**Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

So now you have a directory structure that looks like:

```
ros2
|-bulid
|-install
| +-setup.zsh
|-log
+-src
  |-examples
  +-example_interfaces
```

Now run `colcon test` to make sure all is well.

Now run `source install/setup.zsh` (best) or alternatively `source install/local_setup.zsh`. The difference is:

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

## Args

- Pass `cmake` any args: `--cmake-args -DCMAKE_PREFIX_PATH=/home/bob/here -DCMAKE_SOMETHING_ELSE`
- Clean first: `--cmake-clean-first`

## References

- Ros2 Tutorial: [Using Colcon to Build Packages](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- rosanswers: [What is the difference between local_setup.bash and setup.bash](https://answers.ros.org/question/292566/what-is-the-difference-between-local_setupbash-and-setupbash/)
