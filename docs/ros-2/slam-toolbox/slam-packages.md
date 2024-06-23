# SLAM Packages in ROS2

> **NOTE:** Not interested in anything ROS1 anymore

## ROS2

- SLAM toolbox, only real option
- Cartographer

- graph slam
    - sensor matcher, computes relative poses and confidences between candidate scans
    - Pose graph, not sensor specific, so can easily include different sensors
    - loop closure
    - graph optimizer tries to reduce error in graph

- SLAM Toolbox
    - Map large and dyncamic spaces
    - Serializes maps to combine various sessions with no loss of data
        - *Continue Mapping* is refining or exploring new spaces
        - *Lifelong Mapping* is *continue mapping* _plus_ the removal of extra nodes
            - Added heuristics to remove things (changes in the environment) so it runs on embedded systems better
    - Synchronous
        - May lag in large spaces
        - Offline for best performance
        - up to 200,000 square feet
        - can increase this using cloud services and off-board processing
    - Asynchronous, best effort
        - online navigation
        - good for embeded real-time SLAM
    - stack size is used for serialization/deserialization

# References

- [ROSCon 2019 Macau: On Use of SLAM Toolbox](https://vimeo.com/378682207)
