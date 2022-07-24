---
title: Github Workflows with ROS2
date: 7 Sept 2020
---

`.github/workflows/workflow.yaml`:

```
name: Test Example

on:
  pull_request:
  push:
    branches:
      - master

jobs:
  build-and-test:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest, macos-latest]
      fail-fast: false
    steps:
    - name: Setup ROS 2
      uses: ros-tooling/setup-ros@0.0.15
    with:
        required-ros-distributions: eloquent

    - name: Run Tests
      uses: ros-tooling/action-ros-ci@0.0.13
      with:
        package-name: example_package

    - name: Upload Logs
      uses: actions/upload-artifact@v1
      with:
        name: colcon-logs
        path: ros_ws/log
      if: always()
```

# References

- github: [setup-ros](https://github.com/ros-tooling/setup-ros)
- Ubuntu: [ROS 2 CI with Github Actions](https://ubuntu.com/blog/ros-2-ci-with-github-actions)
