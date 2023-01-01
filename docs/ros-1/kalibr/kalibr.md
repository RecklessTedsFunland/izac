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

## Rolling Shutter Results

This doesn't seem to work ... I get a `[ERROR] [1672599708.485615]: Exception: float division by zero` 
when I run it.

```
rosrun kalibr kalibr_calibrate_rs_cameras --bag awesome.bag --model pinhole-radtan-rs --target ../april_4x5.yml --topic /cam0/image_raw --inverse-feature-variance 1 --frame-rate 30
```

## Camera Calibration Results

So it appears to have worked:

[`awesome-report-cam.pdf`](awesome-report-cam.pdf)

[`awesome-results-cam.txt`](awesome-results-cam.txt)

```
Calibration results 
====================
Camera-system parameters:
cam0 (/cam0/image_raw):
    type: <class 'aslam_cv.libaslam_cv_python.DistortedPinholeCameraGeometry'>
    distortion: [ 0.07536437 -0.18468836  0.00092726  0.00033364] +- [0.00560802 0.01507799 0.00053571 0.00044853]
    projection: [1151.13341587 1150.30902036  648.80503297  365.50538407] +- [0.80018663 0.76459404 0.07637371 0.85366651]
    reprojection error: [0.000121, -0.000036] +- [0.380177, 0.318709]

cam1 (/cam1/image_raw):
    type: <class 'aslam_cv.libaslam_cv_python.DistortedPinholeCameraGeometry'>
    distortion: [ 0.06143751 -0.13747218  0.00153052 -0.00356831] +- [0.00531193 0.01316079 0.00052539 0.00045102]
    projection: [1155.11798085 1154.48430634  645.37252872  365.78508202] +- [0.80123604 0.76772923 0.14196111 0.82729782]
    reprojection error: [-0.000098, -0.000042] +- [0.416945, 0.325086]

baseline T_1_0:
    q: [-0.00087935 -0.00270995 -0.0008824   0.99999555] +- [0.00150475 0.00034032 0.00017118]
    t: [-0.02997144 -0.00027389  0.00036353] +- [0.00010267 0.00006741 0.00030318]



Target configuration
====================

  Type: aprilgrid
  Tags: 
    Rows: 4
    Cols: 5
    Size: 0.02 [m]
    Spacing 0.004 [m]
```

[`awesome-camchain.yaml`](awesome-camchain.yaml)

```yaml
cam0:
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [0.0753643713800888, -0.18468836000112895, 0.0009272632396975177, 0.00033363659205522315]
  distortion_model: radtan
  intrinsics: [1151.1334158718826, 1150.3090203551176, 648.8050329663585, 365.5053840716478]
  resolution: [1280, 720]
  rostopic: /cam0/image_raw
cam1:
  T_cn_cnm1:
  - [0.9999837550791691, -0.0017600256386679483, 0.0054214285495924955, -0.0299714404913448]
  - [0.0017695575766284754, 0.9999968962420779, -0.0017539019917587435, -0.00027389486151521193]
  - [-0.0054183248103174715, 0.0017634670297258209, 0.9999837658383699, 0.0003635283259655295]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [0.06143751113012889, -0.137472177155589, 0.0015305181120103494, -0.003568309204891758]
  distortion_model: radtan
  intrinsics: [1155.1179808462648, 1154.4843063411186, 645.3725287189998, 365.7850820223632]
  resolution: [1280, 720]
  rostopic: /cam1/image_raw
```

## Camera and IMU Results

TBD
