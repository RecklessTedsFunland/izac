---
title: ROS Setup Using Vanilla Ubuntu Image
date: 22 Sept 2019
---

So the [ros wiki](http://wiki.ros.org/Installation/Ubuntu) shows that
melodic packages are built for Ubuntu and armhf (RPi) automatically.
Unfortunately, there are not official raspbian packages, so if you 
want easy, you have to use Ubuntu.

- Ubuntu server images are [here](https://wiki.ubuntu.com/ARM/RaspberryPi) built
for the RPi 2/3/3+ (no 4 yet).
    - These appear to have user ubuntu and password ubuntu

Install ubuntu mate (18.04 at the time of this writing) from [raspberrypi.org](https://www.raspberrypi.org/downloads/).
Unfortunately, they do not allow headless install because they are
stupid.

1. download and burn image to SD card
1. insert SD card, hookup keyboard/mouse/monitor, and power on
    1. Hey Ubuntu Mate idiots, you do realize that IoT are often headless ... right?
    Thank goodness you don't support Arduino, otherwise we would have to do the same
    ... thanks a lot for extra complexity ... you suck! Oh, this also doesn't make
    anything safer or more secure!
1. Then following the ros ubuntu install directions, use a script like below and
run: `sudo ./install_ros.sh`
    ```
    #!/bin/bash
    # http://wiki.ros.org/melodic/Installation/Ubuntu

    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    apt-get update
    # apt-get install ros-melodic-desktop-full -y
    # apt-get install ros-melodic-desktop -y
    apt-get install ros-melodic-ros-base -y
    rosdep init
    rosdep update
    #echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    #source ~/.bashrc
    apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
    ```


## Services

Here are some example systemd scripts examples I found online.

```
$ cat /etc/systemd/system/roscore.service 
[Unit]
After=NetworkManager.service time-sync.target
[Service]
Type=forking
User=ubuntu
# Start roscore as a fork and then wait for the tcp port to be opened
# ----------------------------------------------------------------
# Source all the environment variables, start roscore in a fork
# Since the service type is forking, systemd doesn't mark it as
# 'started' until the original process exits, so we have the 
# non-forked shell wait until it can connect to the tcp opened by
# roscore, and then exit, preventing conflicts with dependant services
ExecStart=/bin/sh -c ". /opt/ros/kinetic/setup.sh; . /etc/ubiquity/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done"
[Install]
WantedBy=multi-user.target
```

```
$ cat /etc/systemd/system/magni-base.service 
[Unit]
Requires=roscore.service
PartOf=roscore.service
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=ubuntu
ExecStart=/usr/sbin/magni-base
[Install]
WantedBy=multi-user.target
```

```
$ cat /usr/sbin/magni-base 
#!/bin/bash

function log() {
  logger -s -p user.$1 ${@:2}
}

log info "magni-base: Using workspace setup file /home/ubuntu/catkin_ws/devel/setup.bash"
source /home/ubuntu/catkin_ws/devel/setup.bash

log_path="/tmp"
if [[ ! -d $log_path ]]; then
  CREATED_LOGDIR=true
  trap 'CREATED_LOGDIR=false' ERR
    log warn "magni-base: The log directory you specified \"$log_path\" does not exist. Attempting to create."
    mkdir -p $log_path 2>/dev/null
    chown ubuntu:ubuntu $log_path 2>/dev/null
    chmod ug+wr $log_path 2>/dev/null
  trap - ERR
  # if log_path could not be created, default to tmp
  if [[ $CREATED_LOGDIR == false ]]; then
    log warn "magni-base: The log directory you specified \"$log_path\" cannot be created. Defaulting to \"/tmp\"!"
    log_path="/tmp"
  fi
fi

source /etc/ubiquity/env.sh
log info "magni-base: Launching ROS_HOSTNAME=$ROS_HOSTNAME, ROS_IP=$ROS_IP, ROS_MASTER_URI=$ROS_MASTER_URI, ROS_LOG_DIR=$log_path"

# Punch it.
export ROS_HOME=$(echo ~ubuntu)/.ros
export ROS_LOG_DIR=$log_path
roslaunch magni_demos teleop.launch &
PID=$!

log info "magni-base: Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
echo "$PID" > $log_path/magni-base.pid
wait "$PID"
```
