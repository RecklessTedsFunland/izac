---
title: Awesome ROS Nodes
date: 9 Aug 2020
image: "https://i.pinimg.com/564x/ec/a5/0d/eca50dae0d4b1c3c969da06e208805bd.jpg"
image-height: "300px"
---

## Camera

- [raspicam2_node](https://github.com/christianrauch/raspicam2_node)
- [opencv_cam](https://github.com/christianrauch/opencv_cam)

## Markers

- [apriltag_ros](https://github.com/christianrauch/apriltag_ros)

## LIDAR

- [ydlidar](https://github.com/YDLIDAR/ydlidar_ros2_driver)
    - Install first: [ydlidar sdk](https://github.com/YDLIDAR/YDLidar-SDK)
    - Avoid polluting system with:
        - `cmake -DCMAKE_INSTALL_PREFIX=/home/ubuntu/SDKs -DBUILD_SHARED_LIBS=1 .. && make install` 
        - `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/SDKs/lib`
        - Setup `ld`
            - `sudo echo "/home/ubuntu/SDKs/lib" > /etc/ld.so.conf.d/kevin.conf`
            - `sudo ldconfig`
        - Verify with `ldconfig -v`
        - Install python with (not build dir, see `setup.py` in root dir): `pip install -e .` 
    - Build ROS2 driver with: `colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/home/ubuntu/SDKs`
    - Once the ros driver is built: `source install/setup.bash`
    - **ERROR:** For some reason, the cmake files produced by `ydlidar_sdk` are wrong ... in `/home/ubuntu/SDKs/lib/make/ydlidar_sdk` change to:
        - `SET( YDLIDAR_SDK_LIBRARY_DIRS  /home/ubuntu/SDKs/lib CACHE INTERNAL "YDLIDAR_SDK library directories" FORCE )`
        - Originally the library path `/home/ubuntu/SDKs/lib` was missing, so `${YDLIDAR_SDK_LIBRARY_DIRS}` is empty
    - **ERROR:** python doesn't work (lots of warnings pop up) and cpp is cleaner, but fails also:
```$ ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node 
[INFO] [1597097146.820571890] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Current ROS Driver Version: 1.0.1

YDLidar SDK initializing
YDLidar SDK has been initialized
[YDLIDAR]:SDK Version: 1.0.1
LiDAR successfully connected
Error, cannot retrieve YDLidar health code: ffffffff
get Device Information Error
[CYdLidar::initialize] Error initializing YDLIDAR check status under [/dev/ydlidar] and [230400].
[ERROR] [1597097149.494657227] [ydlidar_ros2_driver_node]: Unknown error

[INFO] [1597097149.499509163] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Now YDLIDAR is stopping .......
```

## IMU

# References

- [Setting library paths](https://www.cyberciti.biz/faq/linux-setting-changing-library-path/)
