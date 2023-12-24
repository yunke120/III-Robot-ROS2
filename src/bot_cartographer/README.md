

# bot_cartographer

## Install

```shell
$ sudo apt install ros-humble-cartographer
$ sudo apt install ros-humble-cartographer-ros
$ sudo apt install ros-humble-navigation2 
$ sudo apt install ros-humble-nav2-bringup
```

## Run Demo
shell1



## Save Map
```shell
$ ros2 run nav2_map_server map_saver_cli
```
or
```shell
$ ros2 run nav2_map_server map_saver_cli -f [your_map_name]
```

## Show Map In RVIZ2
shell1
```shell
$ ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=substation.yaml
```
shell2, add map plugin
```shell
$ rviz2
```
shell3
```shell
$ ros2 lifecycle set /map_server configure
$ ros2 lifecycle set /map_server activate
```


```shell
$ ros2 run tf2_ros tf2_echo map odom
$ ros2 run tf2_tools view_frames
```
