---
title: ROS2 Parameters
date: 31 Aug 2020
---

ROS allows you to set parameters and then change them at run-time without having
to recompile/build your code.

```python
import rclpy
from rclpy.node import Node

class TestParams(Node):

    def __init__(self):
        super().__init__('test_params_rclpy')

        self.declare_parameter('my_str')
        self.declare_parameter('my_int')
        self.declare_parameter('my_double_array')

# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

You can set the values at run-time from the command line:

```
$ ros2 run ros2_tutorials test_params_rclpy --ros-args -p my_str:="Hello world" -p my_int:=5 -p my_double_array:="[4.4, 5.5, 6.6]"
```

## Types

```
NOT_SET = 0
BOOL = 1
INTEGER = 2
DOUBLE = 3
STRING = 4
BYTE_ARRAY = 5
BOOL_ARRAY = 6
INTEGER_ARRAY = 7
DOUBLE_ARRAY = 8
STRING_ARRAY = 9
```

## Setup Callback for Parameters

You can also setup a callback function to handle parameter calls at run-time.

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class TestParams(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_str' and param.type_ == Parameter.Type.STRING:
                self.my_str = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('test_params_rclpy')

        self.declare_parameter('my_str', 'default value')
        self.my_str = self.get_parameter('my_str').value
        self.get_logger().info("New value set: %s" % self.my_str)
        self.add_on_set_parameters_callback(self.parameter_callback)
```

## Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tutorials',
            executable='test_params_rclpy',
            parameters=[
                {'my_str': 'Hello world'},
                {'my_int': 5},
                {'my_double_array': [4.4, 5.5, 6.6]}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
```

# YAML Config File

Another way to set parameters is with a YAML config file. Here your node `your_amazing_node` is setup 
with the following parameters:

```yaml
your_amazing_node:
  ros__parameters:
    bool_value: True
    int_number: 5
    float_number: 3.14
    str_text: "Hello Universe"
    bool_array: [True, False, True]
    int_array: [10, 11, 12, 13]
    float_array: [7.5, 400.4]
    str_array: ['Nice', 'more', 'params']
    bytes_array: [0x01, 0xF1, 0xA2]
    nested_param:
      another_int: 7
```

You can also setup multiple nodes in one YAML file:

```yaml
node_1:
  ros__parameters:
    some_text: "abc"

node_2:
  ros__parameters:
    int_number: 27
    float_param: 45.2

node_3:
  ros__parameters:
    int_number: 45
```

A suggested package layout for packages:

```
your_package/
├── config
│   └── params.yaml
├── launch
│   └── test_params.launch.py
├── package.xml
├── ...
```

```
$ ros2 run ros2_tutorials test_yaml_params --ros-args --params-file ~/ros2_ws/src/ros2_tutorials/config/params.yaml
```

## Launch File

You can also apply the config file in a launch file.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ros2_tutorials'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'ros2_tutorials',
        name = 'your_amazing_node',
        executable = 'test_yaml_params',
        parameters = [config]
    )

    ld.add_action(node)
    return ld
```

You can also add the config file to your python package in the `setup.py` file:

```python
...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
...
```

A C++ CMake would be:

```cmake
...
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
...
```

## Namespaces

You can also put your parameters into a namespace like `ns1`:

```yaml
ns1:
  your_amazing_node:
    ros__parameters:
      bool_value: True
      ...
```

Now run the node with:

```
$ ros2 run ros2_tutorials test_yaml_params --ros-args -r __ns:=/ns1 --params-file ~/ros2_ws/src/ros2_tutorials/config/params.yaml
```

Or create a YAML file and make the following change for the namespace:

```yaml
...
    node=Node(
        package = 'ros2_tutorials',
        namespace='ns1',
        name = 'your_amazing_node',
        executable = 'test_yaml_params',
        parameters = [config]
    )
...
```

# Did It Work?

```
$ ros2 param dump /your_amazing_node --print
your_amazing_node:
  ros__parameters:
    bool_array:
    - true
    - false
    - true
    bool_value: true
    bytes_array: !!python/object/apply:array.array
    - q
    - - 1
      - 241
      - 162
    float_array: !!python/object/apply:array.array
    - d
    - - 7.5
      - 400.4
    float_number: 3.14
    int_array: !!python/object/apply:array.array
    - q
    - - 10
      - 11
      - 12
      - 13
    int_number: 5
    nested_param:
      another_int: 7
    str_array:
    - Nice
    - more
    - params
    str_text: Hello Universe
    use_sim_time: false
```

# References

- Roboticsbackend: [ROS2 YAML For Parameters](https://roboticsbackend.com/ros2-yaml-params/)
- Roboticsbackend: [rclcpp Params Tutorial – Get and Set ROS2 Params with Cpp](https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/)
- Roboticsbackend: [rclpy Params Tutorial – Get and Set ROS2 Params with Python](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/)
