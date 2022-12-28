# ROS 1 Cheatsheet

Setup `ENV`: `source /opt/ros/<version>/setup.bash`

- `rosmsg [show,package,users,md5]`:
  - `rosmsg show Pose`
  - `rosmsg package nav_msgs`
- `rostopic [bw,echo,hz,list,pub,type,find]`:
  - `rostopic pub -r 10 /topic_name std_msgs/String hello`
  - `rostopic echo /topic_name`
  - `rostopic type /topic_name | rosmsg show`
- `rosservice [list,node,call,args,type,uri,find]`:
  - `rosservice call /add_two_ints 1 2`
  - `rosservice type add_two_ints | rossrv show`
- `rosbag [record,play]`:
  - `rosbag record -a` record all
  - `rosbag record topic1 topic2`
  - `rosbag play -a demo.bag` play all
  - `rosbag play -l demo1.bag demo2.bag` play both bags and loop data
- `rosrun tf tf_echo <source_frame> <target_frame>`:
  - `rosrun tf tf_echo /map /odom`
- `roscore` start up the master (default `locahost:11311`)

## References

- [ROScheatsheet](https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf)
