# ROS2 Packages

## New

```bash
ros2 pkg create --help
usage: ros2 pkg create [-h] [--package-format {2,3}]
                       [--description DESCRIPTION] [--license LICENSE]
                       [--destination-directory DESTINATION_DIRECTORY]
                       [--build-type {cmake,ament_cmake,ament_python}]
                       [--dependencies DEPENDENCIES [DEPENDENCIES ...]]
                       [--maintainer-email MAINTAINER_EMAIL]
                       [--maintainer-name MAINTAINER_NAME]
                       [--node-name NODE_NAME] [--library-name LIBRARY_NAME]
                       package_name

Create a new ROS 2 package

positional arguments:
  package_name          The package name

options:
  -h, --help            show this help message and exit
  --package-format {2,3}, --package_format {2,3}
                        The package.xml format.
  --description DESCRIPTION
                        The description given in the package.xml
  --license LICENSE     The license attached to this package; this can be an
                        arbitrary string, but a LICENSE file will only be
                        generated if it is one of the supported licenses (pass
                        '?' to get a list)
  --destination-directory DESTINATION_DIRECTORY
                        Directory where to create the package directory
  --build-type {cmake,ament_cmake,ament_python}
                        The build type to process the package with
  --dependencies DEPENDENCIES [DEPENDENCIES ...]
                        list of dependencies
  --maintainer-email MAINTAINER_EMAIL
                        email address of the maintainer of this package
  --maintainer-name MAINTAINER_NAME
                        name of the maintainer of this package
  --node-name NODE_NAME
                        name of the empty executable
  --library-name LIBRARY_NAME
                        name of the empty library
```

Summary:

```bash
ros2 pkg create --build-type <ament_cmake|ament_python> [--license MIT] [--dependencies ros_pkg ...] -- <pkg_name>
ros2 pkt create --build-type <ament_cmake|ament_python> ... [--node-name pkg_node] <pkg_name>
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp
```

It is recommended to use one of the ament license identifiers:

- Apache-2.0
- BSL-1.0
- BSD-2.0
- BSD-2-Clause
- BSD-3-Clause
- GPL-3.0-only
- LGPL-3.0-only
- MIT
- MIT-0
