# III-Robot-ROS2


## robot_model
```shell
$ colcon build --packages-select robot_model
$ ros2 launch robot_model display.launch.py
```
![robot_model](./asserts/robot_model.png)

## bridge

shell 1
```shell
$ ROS_DOMAIN_ID=0
$ colcon build --packages-select bridge
$ source install/setup.bash
$ ros2 run bridge bridge_demo_widget
```
shell 2
```shell
$ ROS_DOMAIN_ID=0
$ colcon build --packages-select micro_ros_setup
$ source install/setup.bash
$ sudo chmod a+rw /dev/ttyACM0
$ ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0
```
![bridge_demo](./asserts/bridge_demo.png)

## FAQ
 - [FAQ](./FAQ.md)

## Refer
 - [Window SSH Linux](https://elementalgrady.com/posts/ubuntu-2204-enable-ssh/)
