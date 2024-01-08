# NAV2阅读笔记

1. [里程计设置](http://dev.nav2.fishros.com/doc/setup_guides/odom/setup_odom.html#setting-up-odometry-on-your-robot)

    里程计信息可以从各种来源获得，例如 IMU、LIDAR、RADAR、VIO 和车轮编码器。需要注意的一点是，IMU会随着时间而漂移，而车轮编码器会随着行驶距离而漂移，因此它们经常一起使用以抵消彼此的负面特性。
    请记住，Nav2 需要发布 nav_msgs/Odometry 消息和 odom => base_link 转换，这应该是您设置里程计系统时的目标。
2. [传感器设置](http://dev.nav2.fishros.com/doc/setup_guides/sensors/setup_sensors.html#build-run-and-verification)
    障碍物层和体素层可以同时使用 LaserScan 和 PointCloud2 作为其 data_type ，但默认情况下设置为 LaserScan 。下面的代码片段展示了一个同时使用 LaserScan 和 PointCloud2 作为传感器源的例子。这在设置自己的物理机器人时可能特别有用。
    ```yaml
    obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: True
    observation_sources: scan pointcloud
    scan:
        topic: /scan
        data_type: "LaserScan"
    pointcloud:
        topic: /depth_camera/points
        data_type: "PointCloud2"
    ```
