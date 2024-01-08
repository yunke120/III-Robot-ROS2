

## Packages to install
 - sudo apt install ros-humble-nav2-costmap-2d
 - sudo apt install ros-humble-nav2-core
 - sudo apt install ros-humble-nav2-behaviors
 - sudo apt install ros-humble-robot-localization
 - sudo apt install ros-humble-gazebo-ros-pkgs
 - sudo apt install ros-humble-nav2-map-server
 - sudo apt install ros-humble-nav2-bringup

 ## Parameter explanation

**amcl**:

这一部分配置的是自适应蒙特卡洛定位（AMCL）节点的参数：

    use_sim_time: 是否使用仿真时间。
    alpha1, alpha2, alpha3, alpha4, alpha5: AMCL 算法的参数。
    base_frame_id: 机器人底部的坐标系名称。
    beam_skip_distance, beam_skip_error_threshold, beam_skip_threshold, do_beamskip: 雷达束跳跃算法的参数。
    global_frame_id: 全局地图坐标系名称。
    laser_likelihood_max_dist: 激光测量的最大距离。
    laser_max_range: 激光传感器的最大测距范围。
    laser_min_range: 激光传感器的最小测距范围。
    laser_model_type: 激光模型的类型。
    max_beams: 最大激光束数。
    max_particles: 粒子滤波器的最大粒子数。
    min_particles: 粒子滤波器的最小粒子数。
    odom_frame_id: 里程计坐标系名称。
    pf_err, pf_z: 粒子滤波器的参数。
    recovery_alpha_fast, recovery_alpha_slow: 用于恢复行为的参数。
    resample_interval: 重采样间隔。
    robot_model_type: 机器人模型的类型。
    save_pose_rate: 保存位姿的频率。
    sigma_hit: 激光击中模型的参数。
    tf_broadcast: 是否广播变换。
    transform_tolerance: 变换的容忍度。
    update_min_a, update_min_d: 位置和姿态更新的最小阈值。
    z_hit, z_max, z_rand, z_short: 用于激光模型的参数。
    scan_topic: 激光扫描话题。

**bt_navigator**:

这一部分配置的是导航行为树（Behavior Tree）的参数：

    use_sim_time: 是否使用仿真时间。
    global_frame, robot_base_frame, odom_topic: 全局坐标系、机器人底部坐标系、里程计话题。
    default_bt_xml_filename: 默认的行为树 XML 文件。
    bt_loop_duration, default_server_timeout: 行为树循环间隔和默认服务器超时。
    enable_groot_monitoring, groot_zmq_publisher_port, groot_zmq_server_port: groot 监控的相关参数。
    plugin_lib_names: 行为树插件的名称列表。

**controller_server**:

这是用于配置控制器服务器（controller_server）的ROS 2 参数。控制器服务器负责实现机器人的运动控制，以达到导航目标。

    use_sim_time: 是否使用仿真时间。如果设置为 True，控制器服务器将使用仿真时间，否则将使用实际时间。

    controller_frequency: 控制器执行的频率，以赫兹（Hz）为单位。

    min_x_velocity_threshold, min_y_velocity_threshold, min_theta_velocity_threshold: 控制器考虑的最小线性和角速度阈值。

    failure_tolerance: 失败容差，用于确定是否视为控制失败的阈值。

    progress_checker_plugin: 进度检查器插件的名称，用于检查机器人是否在规定的时间内取得进展。

    goal_checker_plugins: 用于检查是否达到目标的插件列表。

    controller_plugins: 使用的控制器插件的列表，这里是 "FollowPath"。

    progress_checker: 进度检查器的配置参数。

        plugin: 进度检查器插件的类型，这里是 "nav2_controller::SimpleProgressChecker"。

        required_movement_radius: 要求机器人在规定时间内移动的最小半径。

        movement_time_allowance: 允许的移动时间。

    general_goal_checker: 通用目标检查器的配置参数。

        stateful: 是否是有状态的。

        plugin: 目标检查器插件的类型，这里是 "nav2_controller::SimpleGoalChecker"。

        xy_goal_tolerance: 在xy平面上的目标容忍度。

        yaw_goal_tolerance: 在方向上的目标容忍度。

    FollowPath: 使用 "FollowPath" 控制器插件的配置参数。

        plugin: 使用的控制器插件类型，这里是 "dwb_core::DWBLocalPlanner"。

        debug_trajectory_details: 是否启用调试轨迹详细信息。

        一系列控制器的速度、加速度、时间等参数。

        critics: 轨迹评估中使用的评估标准，包括 "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"。

        每个评估标准的具体参数配置，例如 BaseObstacle.scale, PathAlign.scale 等。


**controller_server_rclcpp_node**

这是用于配置控制器服务器节点（controller_server_rclcpp_node）的ROS 2 参数。控制器服务器负责接收导航目标并执行相应的控制策略，使机器人达到目标位置。

**local_costmap**

这是ROS 2中用于配置本地成本地图（local costmap）的参数。本地成本地图是导航系统中的关键组件，用于表示机器人周围环境的成本信息，以便规划路径避开障碍物。以下是各个参数的解释：

    update_frequency: 更新本地成本地图的频率。

    publish_frequency: 发布本地成本地图的频率。

    global_frame: 全局坐标系的名称。

    robot_base_frame: 机器人底部坐标系的名称。

    use_sim_time: 是否使用仿真时间。

    rolling_window: 是否使用滚动窗口模式。在滚动窗口模式下，只有在机器人周围的区域内计算成本。

    width, height: 本地成本地图的宽度和高度。

    resolution: 本地成本地图的分辨率。

    robot_radius: 机器人的半径，用于计算和考虑机器人的安全空间。

    plugins: 本地成本地图使用的插件列表，包括障碍物层、体素层和膨胀层。

下面是各个插件的配置：

    inflation_layer: 膨胀层的配置参数。
        inflation_radius: 膨胀半径，表示障碍物周围多远需要考虑膨胀。
        cost_scaling_factor: 膨胀层的成本缩放因子。

    obstacle_layer: 障碍物层的配置参数。
        enabled: 是否启用障碍物层。
        observation_sources: 观测源列表。
        scan: 障碍物层使用的激光扫描参数。
            topic: 激光扫描话题。
            max_obstacle_height: 障碍物的最大高度。
            clearing: 是否清除障碍物。
            marking: 是否标记障碍物。
            data_type: 激光数据类型。

    voxel_layer: 体素层的配置参数。
        enabled: 是否启用体素层。
        publish_voxel_map: 是否发布体素地图。
        origin_z, z_resolution, z_voxels: 体素层的高度信息参数。
        max_obstacle_height, mark_threshold: 障碍物相关的参数。
        raytrace_max_range, raytrace_min_range, obstacle_max_range, obstacle_min_range: 光线追踪和障碍物参数。

    static_layer: 静态层的配置参数。
        map_subscribe_transient_local: 是否订阅瞬时本地地图。
        always_send_full_costmap: 是否始终发送完整的成本地图。

    local_costmap_client: 本地成本地图的客户端参数，可能包括用于接收和处理本地成本地图的其他模块。

    local_costmap_rclcpp_node: 本地成本地图的ROS节点的参数，包括是否使用仿真时间。

**global_costmap**

这是ROS 2中用于配置全局成本地图（global costmap）的参数。全局成本地图用于规划机器人在整个环境中的路径，包括全局障碍物信息。以下是各个参数的解释：

    update_frequency: 更新全局成本地图的频率。

    publish_frequency: 发布全局成本地图的频率。

    global_frame: 全局坐标系的名称。

    robot_base_frame: 机器人底部坐标系的名称。

    use_sim_time: 是否使用仿真时间。

    robot_radius: 机器人的半径，用于计算和考虑机器人的安全空间。

    resolution: 全局成本地图的分辨率。

    track_unknown_space: 是否跟踪未知空间。

    plugins: 全局成本地图使用的插件列表，包括静态层、障碍物层、体素层和膨胀层。

下面是各个插件的配置：

    obstacle_layer: 障碍物层的配置参数。
        enabled: 是否启用障碍物层。
        observation_sources: 观测源列表。
        scan: 障碍物层使用的激光扫描参数。
            topic: 激光扫描话题。
            max_obstacle_height: 障碍物的最大高度。
            clearing: 是否清除障碍物。
            marking: 是否标记障碍物。
            data_type: 激光数据类型。

    voxel_layer: 体素层的配置参数。
        enabled: 是否启用体素层。
        publish_voxel_map: 是否发布体素地图。
        origin_z, z_resolution, z_voxels: 体素层的高度信息参数。
        max_obstacle_height, mark_threshold: 障碍物相关的参数。
        raytrace_max_range, raytrace_min_range, obstacle_max_range, obstacle_min_range: 光线追踪和障碍物参数。

    static_layer: 静态层的配置参数。
        plugin: 使用的插件类型。
        map_subscribe_transient_local: 是否订阅瞬时本地地图。

    inflation_layer: 膨胀层的配置参数。
        plugin: 使用的插件类型。
        cost_scaling_factor: 膨胀层的成本缩放因子。
        inflation_radius: 膨胀半径。

    global_costmap_client: 全局成本地图的客户端参数，可能包括用于接收和处理全局成本地图的其他模块。

    global_costmap_rclcpp_node: 全局成本地图的ROS节点的参数，包括是否使用仿真时间。

**map_server**
这是用于配置地图服务器（map_server）的ROS 2 参数。地图服务器负责加载和发布地图，供导航系统使用。以下是参数的解释：

    use_sim_time: 是否使用仿真时间。如果设置为 true，地图服务器将使用仿真时间，否则将使用实际时间。

    yaml_filename: 地图的 YAML 文件名。这是地图的配置文件，其中包含地图的信息，包括分辨率、原点、障碍物信息等。在这里，文件名为 "map.yaml"，表示地图的配置信息存储在名为 "map.yaml" 的文件中。

**map_saver**

这是用于配置地图保存器（map_saver）的ROS 2 参数。地图保存器允许将地图保存为文件。以下是各个参数的解释：

    use_sim_time: 是否使用仿真时间。如果设置为 true，地图保存器将使用仿真时间，否则将使用实际时间。

    save_map_timeout: 保存地图的超时时间（以毫秒为单位）。如果地图保存操作超过这个时间，将被中断。

    free_thresh_default: 地图中被认为是自由空间的阈值。该值表示地图中像素值小于此阈值的区域被认为是自由空间。

    occupied_thresh_default: 地图中被认为是占用空间的阈值。该值表示地图中像素值大于此阈值的区域被认为是占用空间。

    map_subscribe_transient_local: 是否订阅瞬时本地地图。如果设置为 true，地图保存器将订阅瞬时本地地图，以确保保存的地图是最新的。

**planner_server**

这是用于配置路径规划器服务器（planner_server）的ROS 2 参数。路径规划器负责计算机器人在环境中的路径。以下是各个参数的解释：

    expected_planner_frequency: 期望的路径规划频率，即规划器每秒尝试规划的次数。

    use_sim_time: 是否使用仿真时间。如果设置为 true，路径规划器服务器将使用仿真时间，否则将使用实际时间。

    planner_plugins: 使用的路径规划插件的列表。在这里，使用了一个名为 "GridBased" 的插件。

下面是 "GridBased" 插件的配置：

    plugin: 插件的类型，这里是 "nav2_navfn_planner/NavfnPlanner"，表示使用 Navfn 路径规划器。

    tolerance: 路径规划器的容差，表示规划的路径可以偏离目标点的最大距离。

    use_astar: 是否使用 A* 路径规划算法。在这里设置为 false，表示使用 Navfn 路径规划器，而不是 A*。

    allow_unknown: 是否允许规划器通过未知空间。如果设置为 true，规划器可以通过地图中的未知区域，这在地图尚未完全探测的情况下可能是有用的。

**planner_server_rclcpp_node**

这是用于配置路径规划器服务器的ROS节点（planner_server_rclcpp_node）的ROS 2 参数。

**recoveries_server**

这是用于配置恢复行为服务器（recoveries_server）的ROS 2 参数。恢复行为用于处理导航系统中可能发生的问题，例如机器人陷入困境或遇到无法规遍的区域。以下是各个参数的解释：

    costmap_topic: 用于恢复行为的成本地图话题。

    footprint_topic: 发布机器人足迹的话题。

    cycle_frequency: 恢复行为循环执行的频率。

    recovery_plugins: 使用的恢复插件的列表。在这里，使用了 "spin"、"backup" 和 "wait" 三个恢复插件。
        spin: 旋转恢复插件的配置。
        backup: 后退恢复插件的配置。
        wait: 等待恢复插件的配置。

    global_frame: 全局坐标系的名称。

    robot_base_frame: 机器人底座坐标系的名称。

    transform_timeout: 获取变换信息的超时时间。

    use_sim_time: 是否使用仿真时间。如果设置为 true，恢复行为服务器将使用仿真时间，否则将使用实际时间。

    simulate_ahead_time: 模拟提前时间，即在执行恢复行为之前模拟的时间。

    max_rotational_vel: 最大旋转速度。

    min_rotational_vel: 最小旋转速度。

    rotational_acc_lim: 旋转加速度限制。

**robot_state_publisher**

这是用于配置机器人状态发布器（robot_state_publisher）的ROS 2 参数。机器人状态发布器负责发布机器人的运动状态信息，例如关节角度和机器人坐标系之间的关系。

**waypoint_follower**

这是用于配置航点跟随器（waypoint_follower）的ROS 2 参数。航点跟随器用于引导机器人按照预定的路径或航线移动。以下是参数的解释：

    loop_rate: 航点跟随器循环执行的频率，以赫兹（Hz）为单位。

    stop_on_failure: 如果设置为 true，在遇到失败情况时航点跟随器将停止。

    waypoint_task_executor_plugin: 航点任务执行器插件的名称。在这里，使用了名为 "wait_at_waypoint" 的插件。

        wait_at_waypoint: "wait_at_waypoint" 插件的配置。

            plugin: 插件的类型，这里是 "nav2_waypoint_follower::WaitAtWaypoint"，表示使用等待航点插件。

            enabled: 是否启用该插件。

            waypoint_pause_duration: 在航点上等待的时间（以毫秒为单位）。