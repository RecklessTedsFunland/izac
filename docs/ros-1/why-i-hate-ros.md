---
title: Why I Hate ROS
date: 29 Sept 2019
---

There is a large community that uses ROS and lots of good stuff out there, but
I honestly don't know why.

Why I hate ROS:

- catkin
    - ros should really be a simple library you use cmake to include into
    programs. Instead there are many commands to just build the project
        - workspaces: too complex, your packages go into a `src` directory, but then
        they themselves have a `src` directory ... "turtles all the way down!"
        - catkin_make: should just be `make`, why do I need to source a `setup.bash`
        file just to make things work and then call `catkin_make` several leves
        above my project?
- python
    - ros1 uses python2.7 still ... why? I have moved on to 3.0 and don't want
    to go back!
- OpenCV
    - try updating to a modern version ... I dare you!
- Core
    - to do the simplest thing, you have to publish a ton of tf frames and messages
    ... why?
    - really easy to end up with a ton of nodes for a simple robot
