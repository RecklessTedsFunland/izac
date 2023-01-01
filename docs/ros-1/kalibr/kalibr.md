# Kalibr

OMG, Kalibr is such a POS. Not only is it only `ROS1`, but it is a very old 
`ROS1` version. The documentation is crap too ... hopefully this is a little 
clearer.

## Overview

I am doing this on macOS using Docker (Ubuntu 20.04 version).

1. Convert data to `ROS1` bag file if necessary with
1. Calibrate the cameras with `kalibr_calibrate_cameras`
1. Calibrate the cameras and imu with `kalibr_calibrate_imu_camera`
1. Calibrate rolling shutter, not sure how useful this is

## References

- [Data layout](https://github.com/ethz-asl/kalibr/wiki/bag-format) for using the bag conversion script
- [Kalibr Github Repo](https://github.com/ethz-asl/kalibr)
- YouTube: [DIY Indoor Autonomous Drone! - Part 2 (Kalibr & Calibration)](https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY)

## Downloads

- Example data:
  - Downloads: https://github.com/ethz-asl/kalibr/wiki/downloads
  - [IMU-CAM.bag](https://drive.google.com/file/d/1dHkfsPBzUbnoyXpmiGxSQiqt4hrPXF2z/view?usp=sharing)
  - [aprilgrid_6x6.yml](https://drive.google.com/file/d/10zw3LvCDvXYyTQje4Gt4vzJMMYOuGsma/view?usp=sharing)
- Get the software
  - Download the zip of the Kalibr repo
  - Or clone the repo, here is the version I tested from 24 Nov 2022, Merge pull request #582:
    ```
    git clone https://github.com/ethz-asl/kalibr.git
    git checkout -b test ee5a57d
    ```
- Save bag and `aprilgrid.yml` to `data` folder in the repo (I made the folder) 

## Build and Run Image

1. `docker build -t kalibr -f Docker_ros1_20_04 .`
1. `docker run -it -v "$PWD/data:/data" kalibr`
    - **WARNING:** Once you set the volume path to `$PWD/data` you cannot
      change it when you run the container again. I *think* if you redo the
      `docker run ...` command you can change the volume location.
    - Check bag file: 
        ```
        root@8b064ce39682:/data# rosbag info imu_april.bag
        path:        imu_april.bag
        version:     2.0
        duration:    1:11s (71s)
        start:       Jul 07 2014 11:43:25.79 (1404733405.79)
        end:         Jul 07 2014 11:44:37.69 (1404733477.69)
        size:        996.5 MB
        messages:    17259
        compression: none [960/960 chunks]
        types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
                     sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
        topics:      /cam0/image_raw    1439 msgs    : sensor_msgs/Image
                     /cam1/image_raw    1439 msgs    : sensor_msgs/Image
                     /imu0             14381 msgs    : sensor_msgs/Imu
        ```
1. Check bag works, run: `roscore`
    - Open new terminal, `docker ps` and get the name of the container
        - `docker exec -it <container_name> bash` will log you in
        - `rosbag play imu_april.bag`
    - Open another terminal
        - `docker exec -it <container_name> bash` will log you in
        - `rostopic echo /imu0`

## IMU-CAM Bag

```
root@8b064ce39682:/catkin_ws# rostopic list
/cam0/image_raw
/cam1/image_raw
/clock
/imu0
/rosout
/rosout_agg
```

## Calibrate

1. `rosrun kalibr kalibr_bagcreater --folder dataset/. --output-bag awsome.bag`
    - Verify it worked with: `rosbag info awesome.bag`
1. `rosrun kalibr kalibr_calibrate_cameras --bag awesome.bag --target ../april_4x5.yml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --dont-show-report`
1. `rosrun kalibr kalibr_calibrate_imu_camera --bag awesome.bag --cam [camchain.yaml] --imu [imu.yaml] --target ../april_4x5.yml`

## Bag Creation 

[Ref](https://github.com/ethz-asl/kalibr/wiki/bag-format)

```
+-- dataset-dir
    +-- cam0
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    +-- cam1
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    \-- imu0.csv
```
IMU csv file:

```
timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
1385030208736607488,0.5,-0.2,-0.1,8.1,-1.9,-3.3
 ...
1386030208736607488,0.5,-0.1,-0.1,8.1,-1.9,-3.3
```

## Generate Custom Target

I had to install: `sudo apt install texlive-base` and `pip3 install pxy`

```
rosrun kalibr kalibr_create_target_pdf --nx 5 --ny 4 --tsize 0.02 --tspace 0.2 --type apriltag
```

Here is the result: [target.pdf](target.pdf)

[`aprilgrid.yml`](aprilgrid.yml)

```yaml
target_type: 'aprilgrid' #gridtype
tagCols: 5               #number of apriltags
tagRows: 4               #number of apriltags
tagSize: 0.020           #size of apriltag, edge to edge [m]
tagSpacing: 0.2          #ratio of space between tags to tagSize
codeOffset: 0            #code offset for the first tag in the aprilboard
```

## Rolling Shutter

This doesn't seem to work ... I get a `[ERROR] [1672599708.485615]: Exception: float division by zero` 
when I run it.

```
rosrun kalibr kalibr_calibrate_rs_cameras --bag awesome.bag --model pinhole-radtan-rs --target ../april_4x5.yml --topic /cam0/image_raw --inverse-feature-variance 1 --frame-rate 30
```