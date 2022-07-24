---
title: Simple Publisher and Subscriber in Python
date: 14 Sep 2019
---

Note: I am using docker on Ubuntu with ros melodic

Note: it seems really easy to get kicked out of the container
and dumped into the host env if a command fails.

I really hate catkin, the old cmake way was better! catkin imposes too many
requirements (like how your folder structure is setup) that makes it a pain
in the ass. Unfortunately, ros should be a lightweight library, but instead
it is setup as the center of the universe.

## Setup

**Window 1**
1. `docker pull ros`
1. `docker run -it ros` will drop you into a command line on the container
    1. `. ros_entrypoint.sh` to setup your env
1. `roscore`

**Window 2**
1. Open another window
1. `docker ps -l` to find the name of the container we started above
1. `docker exec -it nostalgic_morse bash` to connect to the container
    1. `. ros_entrypoint.sh` to setup your env
1. Create a packege with `roscreate-pkg pstest rospy roscpp std_msgs`
    1. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/pstest:/root` lets ros know where your work is
1. Put the `talker.py` and the `listener.py` in the `pstest/src` folder
    1. Make sure both are executable with `chmod a+x *.py`
1. Build everyting with `rosmake pstest`
1. Start the talker with `rosrun pstest talker.py`

**Window 3**
1. Open another window
1. `docker exec -it nostalgic_morse bash` to connect to the container
    1. `. ros_entrypoint.sh`
    1. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/pstest:/root`
1. `rosrun pstest listener.py`
