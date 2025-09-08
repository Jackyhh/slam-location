# 安装步骤

1. 使用 `catkin_make` 编译该 package。注意：必须先安装 `livox_ros_driver2` 包（livox 雷达的 ROS 驱动），才能进行编译。
2. 运行 `launch` 目录下的 `run_livox_location.launch` 启动定位算法。
3. 运行 `launch` 目录下的 `run_livox_location_gps.launch` 启动带 RTK 辅助的定位算法。

## 发布的与定位消息有关的话题

- `/agv_pose`：发布 `PoseStamped` 消息
- `/agv_gps_pos`：发布 `NavSatFix` 消息
- `/agv_gps_heading`：发布 `Float64` 消息，单位为角度

## 发布与算法运行状况有关的话题

- `/loc_info`：发布包含算法运行状况的字符串，格式为 JSON

# 算法参数配置

在 `config/livox_config_outdoor.yaml` 文件中，以下是需要重点注意的几个参数：

- `map_filename`：地图文件名
- `lidar_topic`：激光雷达话题
- `imu_topic`：IMU 话题
- `rtk_topic`：RTK GPS 话题
- `rtk_imu_topic`：RTK IMU 话题

### 激光雷达到基座坐标系的偏移和旋转（单位：米和弧度）

- `lidar2base_x`：X 轴偏移
- `lidar2base_y`：Y 轴偏移
- `lidar2base_z`：Z 轴偏移
- `lidar2base_roll`：Roll 角度
- `lidar2base_pitch`：Pitch 角度
- `lidar2base_yaw`：Yaw 角度

### 其他参数

- `thread_num`：线程数，根据当前 CPU 核数决定
- `min_range`：最小范围
- `max_range`：最大范围（一般室内为 20 米，室外为 50 米）
- `filter_leaf_size`：滤波体素大小（室内环境一般为 0.8 米，室外为 1.2 米。如果运算时间过长，可以适当调大该数值，最大可到 1.8 米）
- `limit_angle_min`：最小角度
- `limit_angle_max`：最大角度

