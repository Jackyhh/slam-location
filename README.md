# Installation Steps

1. Compile the package using `catkin_make`. Note: You must first install the `livox_ros_driver2` package (ROS driver for Livox LiDAR) before compiling.
2. Run `run_livox_location.launch` in the `launch` directory to start the localization algorithm.
3. Run `run_livox_location_gps.launch` in the `launch` directory to start the localization algorithm with RTK assistance.

## Published Topics Related to Localization Messages

- `/agv_pose`: Publishes `PoseStamped` messages
- `/agv_gps_pos`: Publishes `NavSatFix` messages
- `/agv_gps_heading`: Publishes `Float64` messages, in degrees

## Published Topics Related to Algorithm Operation Status

- `/loc_info`: Publishes strings containing algorithm operation status in JSON format

# Algorithm Parameter Configuration

In the `config/livox_config_outdoor.yaml` file, the following parameters require special attention:

- `map_filename`: Map file name
- `lidar_topic`: LiDAR topic
- `imu_topic`: IMU topic
- `rtk_topic`: RTK GPS topic
- `rtk_imu_topic`: RTK IMU topic

### LiDAR to Base Coordinate System Offset and Rotation (units: meters and radians)

- `lidar2base_x`: X-axis offset
- `lidar2base_y`: Y-axis offset
- `lidar2base_z`: Z-axis offset
- `lidar2base_roll`: Roll angle
- `lidar2base_pitch`: Pitch angle
- `lidar2base_yaw`: Yaw angle

### Other Parameters

- `thread_num`: Number of threads, determined by the number of CPU cores
- `min_range`: Minimum range
- `max_range`: Maximum range (typically 20 meters for indoor, 50 meters for outdoor)
- `filter_leaf_size`: Filter voxel size (typically 0.8 meters for indoor environments, 1.2 meters for outdoor. If computation time is too long, this value can be increased appropriately, up to a maximum of 1.8 meters)
- `limit_angle_min`: Minimum angle
- `limit_angle_max`: Maximum angle