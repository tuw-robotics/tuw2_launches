# tuw2_launches
ROS2 launches


## SLAM
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/max/projects/ros2/gazebo/tuw/src/tuw2_launches/config/slam_toolbox/tuw_mapper_params_online_async.yaml
ros2 run nav2_map_server map_saver_cli -f config/map/cave

```