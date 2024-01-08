include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",            -- 地图坐标系的名称
  tracking_frame = "base_footprint",    -- 用于跟踪的坐标系的名称 如果使用gazebo使用base_footprint，而不是imu_link
  published_frame = "odom",             -- 发布的坐标系的名称
  odom_frame = "odom",          -- 这个odom是cartographer提供的，需要与机器人本身的odom区分开
  provide_odom_frame =  false, -- 不发布从odom_frame 到 published_frame 的 TF 转换
  publish_frame_projected_to_2d = true,   -- 是否发布二维地图
  -- publish_tracked_pose=true,
  use_odometry = true,          -- 是否使用odom数据
  use_nav_sat = false,          -- 是否使用卫星
  use_landmarks = false,        -- 是否使用地标
  num_laser_scans = 1,          -- 激光扫描的数量
  num_multi_echo_laser_scans = 0,       -- 多回波激光扫描的数量
  num_subdivisions_per_laser_scan = 1,    -- 每个激光扫描的细分数
  num_point_clouds = 0,                   -- 点云数据的数量
  lookup_transform_timeout_sec = 0.5,      -- 查找坐标系变换的超时时间
  submap_publish_period_sec = 0.3,        -- 发布子地图的时间间隔
  pose_publish_period_sec = 0.03, -- 发布机器人位姿的时间间隔
  trajectory_publish_period_sec = 30e-3,    -- 发布轨迹的时间间隔
  rangefinder_sampling_ratio = 1.,          -- 
  odometry_sampling_ratio = 1.,             -- 
  fixed_frame_pose_sampling_ratio = 1.,     -- 用于子样本生成的采样比率
  imu_sampling_ratio = 1.,                  -- 
  landmarks_sampling_ratio = 1.,            -- 
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 指定地图构建器是否使用二维轨迹构建器

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 累积的激光数据数量，用于构建子地图
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35     -- 每个子地图包含的激光数据数量
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 11.0               -- 激光传感器的最小和最大测距范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.    -- 当激光传感器遇到无法测量的地方时，沿着射线添加的虚拟测量的长度
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 是否使用IMU数据进行轨迹构建
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 -- 在线相关扫描匹配的线性搜索窗口大小
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10. -- 在线相关扫描匹配的平移和旋转权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2

POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- Huber损失函数的尺度，用于优化问题的鲁棒性
POSE_GRAPH.optimize_every_n_nodes = 5            -- 每优化多少个节点执行一次位姿图优化 35   0:关闭全局SLAM 尽量小
POSE_GRAPH.constraint_builder.min_score = 0.65    -- 约束建立的最小分数，用于滤除低置信度的约束

return options
