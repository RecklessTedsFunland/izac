---
title: How to use Someone Else's Catkin Package
date: 22 Sept 2019
---

When using someone else's catkin package, here is what you do:

1. `mkdir -p catkin_ws/src`
1. `cd catkin_ws/src`
1. Get repo: `git clone https://github.com/repo.git`
1. Back to workspace: `cd ..`
1. Install needed dependencies: `rosdep install --from-paths src -i -y`
1. `catkin_make`
1. `source devel/setup.bash`

So basically, we created a workspace, downloaded the code, installed depences, and sourced
the setup file.

I really hate how complex catkin is!!!

# References

- [answers.ros.org](https://answers.ros.org/question/230798/sourcing-exsisting-ros-project-running-someone-elses-project/)
