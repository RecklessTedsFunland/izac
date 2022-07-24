---
title: Gazebo and Python Scripting
date: 19 Jan 2021
---

ROS1 only?

- `pck_gazebo_pkgs`
    - generators:
        - `model_factory`
        - `model_group_generator`
        - `world_generator`
        - `assets_manager`
        - `engines_manager`
        - `constraints_manager`
    - simulations
        - physics, models, lights, links, etc
    - parsers
        - `sdf2urdf`, `urdf2sdf`, xacro, jinja, etc
    - task_manager
        - proxy to talk to Gazebo to start/stop/etc


# References

- [ROSCon 2019 Macau: `pcg_gazebo_pkgs`: A Python library for scripting and rapid-prototyping of simulated Gazebo models and worlds](https://vimeo.com/378683294)
