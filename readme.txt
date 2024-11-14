安装步骤：
1. catkin_make编译该package。注意要先安装livox_ros_driver2包（livox雷达的ros驱动），才能通过编译。
2. 运行launch目录下的run_livox_location.launch 即可启动定位算法。
3. 运行launch目录下的run_livox_location_gps.launch 即可启动带RTK辅助的定位算法。


发布的与定位消息有关的话题为：/agv_pose （为PoseStamped消息）
发布的与定位消息有关的带GPS的话题为： /agv_gps_pos （为NavSatFix消息） 和 /agv_gps_heading (为Float64消息，单位为角度)
发布与算法运行状况有关的话题为：/loc_info 为字符串下以json格式的结构。



算法参数配置：
在config/livox_config_outdoor.yaml文件里，需要重点注意的几个参数如下：

map_filename: 地图文件名
lidar_topic: 激光雷达话题
imu_topic: IMU话题

rtk_topic: RTK GPS话题
rtk_imu_topic: RTK IMU话题

激光雷达到基座坐标系的偏移和旋转，单位为米和弧度
lidar2base_x: 
lidar2base_y: 
lidar2base_z: 
lidar2base_roll: 
lidar2base_pitch: 
lidar2base_yaw:


thread_num: 线程数,该参数根据当前CPU核数来决定

min_range: 最小范围
max_range: 最大范围，一般室内20米，室外50米
filter_leaf_size: 滤波体素大小，室内环境一般0.8米，室外为1.2米，如果运算时间过长，可以适当调大该数值，最大可到1.8米。
limit_angle_min: 最小角度
limit_angle_max: 最大角度