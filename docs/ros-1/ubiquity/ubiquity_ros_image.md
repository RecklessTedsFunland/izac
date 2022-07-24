---
title: Ubiquity ROS SD Images for RPi
date: 21 Sep 2019
---

## Images

Ubiquity Robotics has created SD card images with ROS already installed. You can [download here](https://downloads.ubiquityrobotics.com/pi.html). They are based on Ubuntu 16.04 rather than Raspbian

1. Burn image to SD card
1. In the boot volume, do `touch ssh` to enable ssh connections

The image is setup as follows:

- WiFi is an access point:
    - SSID `ubiquityrobotXXXX` where `XXXX` is part of the MAC address
    - Password: ubuntu
- You can also access through ethernet via: `ssh ubuntu@ubiquityrobot.local` and password ubuntu

## Kill X-Windows

The image boot stupidly to a deskop and consumes a lot of resources, you can turn this off with: `sudo service lightdm stop`

## ROS is Already Running

The image has a bunch of nodes already setup and running when you boot:

```
ubuntu@ubiquityrobot:~$ rostopic list
/battery_state
/cmd_vel
/diagnostics
/joint_states
/joy
/left_error
/motor_node/parameter_descriptions
/motor_node/parameter_updates
/right_error
/rosout
/rosout_agg
/statistics
/tf
/tf2_web_republisher/cancel
/tf2_web_republisher/feedback
/tf2_web_republisher/goal
/tf2_web_republisher/result
/tf2_web_republisher/status
/tf_static
```

According to [this post](https://forum.ubiquityrobotics.com/t/changing-start-up-nodes-in-pi-image/55), these can be modified here: `/etc/systemd/system/roscore.service`
and `/etc/systemd/system/magni-base.service`,

`roscore.service` launches `roscore magni-base.service` starts the core nodes for Magni.

If you look in `magni-base.service`, you see that it launches the script at `/usr/sbin/magni-base`. The relevant part of this script is line 32 `roslaunch magni_demos teleop.launch &` this launches the launch file `teleop.launch` from the magni_demos package.

If you already have a launch file for the nodes you should make a copy of both `/etc/systemd/system/magni-base.service` `/usr/sbin/magni-base` in their respective directories, naming them to whatever is relevant to your robot.

Change your version of the script in `/usr/sbin/` to launch the file you want, and your version of the service file in `/etc/systemd/system/` to point to your script.

Then run `sudo systemctl disable magni-base` and `sudo systemctl enable {YOUR SERVICE}`. More details on `roslaunch` can be found [here](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)

## Install Node

Use their official packages:

```
# Using Ubuntu
curl -sL https://deb.nodesource.com/setup_11.x | sudo -E bash -
sudo apt-get install -y nodejs
```

## Mote Fixes

So these are already in [mote](https://github.com/MomsFriendlyRobotCompany/mote) in the ros folder.

- change default password
- change hostname
- turn off default robot launch file
- kill x windows

run sshd 1022
iptables -I INPUT -p tcp --dport 1022 -j ACCEPT

# Nodes

- [PiCamera](https://github.com/UbiquityRobotics/raspicam_node)
- [fiducials](http://wiki.ros.org/fiducials): uses the aruco markers to allow the robot to understand its orientation
