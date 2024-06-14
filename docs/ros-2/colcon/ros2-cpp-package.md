---
title: ROS2 CPP Package
date: 23 Oct 2020
---

ROS2 rclcpp package:

1. Make workspace: `mkdir –p ws/src`
1. Move to source directory: `cd ws/src`
1. Create package: `ros2 pkg create --build-type ament_cmake cpp <my_cool_pkg>`
1. Install dependancies: `rosdep install -i --from-path src --rosdistro <distro> -y`
1. Build:
   - `colcon build`
   - `colcon build --packages-select <my_cool_pkg>`
1. Source setup file: `. install/setup.bash`
1. Run package: `ros2 run <my_cool_pkg> <binary>`

## Binary CMake Example

```cmake
cmake_minimum_required(VERSION 3.8)
project(<my_cool_package>)

# Default to C17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(<binary> src/main.cpp)
ament_target_dependencies(<binary> rclcpp std_msgs)

# Need this for setup.bash to find the binary
install(TARGETS
  <binary>
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

```cmake
cmake_minimum_required(VERSION 3.5)

project(rclcpp)

find_package(Threads REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcpputils REQUIRED)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
  # implicit sign conversions in rclcpp and gtest.cc, see https://ci.ros2.org/job/ci_osx/9265/.
  # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
  # (from https://github.com/ros2/rclcpp/pull/1188#issuecomment-650229140)
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

set(${PROJECT_NAME}_SRCS
  src/file.cpp
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
  ../src
)

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# specific order: dependents before dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rcl"
  "rcl_yaml_param_parser"
  "rcpputils"
  "rcutils"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

# these should match the dependencies above
ament_export_dependencies(rcl)
ament_export_dependencies(rcpputils)
ament_export_dependencies(rcutils)
ament_export_dependencies(rcl_yaml_param_parser)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()

install(
  DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
)
```

## Package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gci_sensors</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="walchko@users.noreply.github.com">walchko</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

# References

- [Example](https://github.com/ros2/rclcpp/blob/master/rclcpp/CMakeLists.txt)
