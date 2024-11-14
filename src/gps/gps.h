#ifndef _GPS_H_
#define _GPS_H_

#include "utility.h"



//gps数据
struct nav_data_type{
    double timestamp;
    double longitude;
    double latitude;
    double altitude;
    double heading;
};

struct gps_data_type{
    bool is_init = false;
    nav_data_type start_gps;
    Eigen::Vector3d start_loc;

    double offset2base_x;
    double offset2base_y;

    double map_heading;

    gps_data_type(){
        is_init = false;

        offset2base_x = 0.0;
        offset2base_y = 0.0;
    }

    void update_data(nav_data_type gps_data, Eigen::Vector3d loc_pose, bool calib_flag){
        start_gps = gps_data;
        start_loc = loc_pose;

        map_heading = normalize_angle(start_gps.heading - start_loc(2));

        // nav_data_type gps_device_coordi = loc2gps(start_loc);
        // start_gps.latitude = gps_device_coordi.latitude;
        // start_gps.longitude = gps_device_coordi.longitude;


        double s_yaw = sin(start_loc(2));
        double c_yaw = cos(start_loc(2));


        if(calib_flag){
            start_loc(0) = c_yaw * offset2base_x - s_yaw * offset2base_y + loc_pose(0);
            start_loc(1) = s_yaw * offset2base_x + c_yaw * offset2base_y + loc_pose(1);
        }
        
        is_init = true;
    }

    Eigen::Vector3d gps2loc(nav_data_type gps_data){
        Eigen::Vector3d res;

        double x_enu = earth_radius * cos(gps_data.latitude * Deg2Rad) * sin((gps_data.longitude - start_gps.longitude) * Deg2Rad); 
        double y_enu = earth_radius * sin((gps_data.latitude - start_gps.latitude) * Deg2Rad);

        double s_th = sin(map_heading);
        double c_th = cos(map_heading);

        double rtk_x = c_th * x_enu + s_th * y_enu + start_loc(0);
        double rtk_y = -s_th * x_enu + c_th * y_enu + start_loc(1);
        res(2) = normalize_angle(gps_data.heading - map_heading);
    
        double s_yaw = sin(res(2));
        double c_yaw = cos(res(2));

        double to_base_x = -offset2base_x;
        double to_base_y = -offset2base_y;

        res(0) = c_yaw * to_base_x - s_yaw * to_base_y + rtk_x;
        res(1) = s_yaw * to_base_x + c_yaw * to_base_y + rtk_y;

        return res;
    }

    nav_data_type loc2gps(Eigen::Vector3d loc_pose){
        double s_yaw = sin(loc_pose(2));
        double c_yaw = cos(loc_pose(2));

        double to_base_x = -offset2base_x;
        double to_base_y = -offset2base_y;

        double rtk_x = loc_pose(0) - c_yaw * to_base_x + s_yaw * to_base_y;
        double rtk_y = loc_pose(1) - s_yaw * to_base_x - c_yaw * to_base_y;

        double s_th = sin(map_heading);
        double c_th = cos(map_heading);

        double dx = rtk_x - start_loc(0);
        double dy = rtk_y - start_loc(1);

        double x_enu = c_th * dx - s_th * dy;
        double y_enu = s_th * dx + c_th * dy;

        nav_data_type res;

        res.latitude = asin(y_enu / earth_radius) / Deg2Rad + start_gps.latitude;
        res.longitude = asin(x_enu /  (earth_radius * cos(res.latitude * Deg2Rad))) / Deg2Rad + start_gps.longitude;
        res.altitude = start_gps.altitude;
        return res;
    }
 

};




class gps{
public:
    std::string mapfile_name_; //地图文件名

    uint8_t is_init_{0x00}; //是否已经初始化
    bool has_origin_{false}; //是否已经初始化了原点
    uint8_t signal_quality_{0}; //信号质量 

    std::string origin_info_file_; //原点信息文件

    gps_data_type gps_data_;
    nav_data_type gps_data_last_; //上一个gps数据

    bool aktiv_gps_loc_{false}; //是否使用gps定位

    ros::Subscriber gps_sub_;
    ros::Subscriber rtk_imu_sub_;    
    
    gps(std::string package_path, std::string config_file_path, ros::NodeHandle &nh); //构造函数

    void write_gps_origin(nav_data_type gps_origin, Eigen::Vector3d loc_pose); //写入原点信息
    void read_gps_origin(void); //读取原点信息

    std::string get_signal_quality(void); //获取信号质量

    void set_gps_start_pos(Eigen::Vector3d loc_pose); //设置起始位置
    bool get_gps_start_pos_init(void);

    Eigen::Vector3d gps_reloc(bool &is_ok); //gps重定位
    bool check_reloc(Eigen::Vector3d loc_pose, Eigen::Vector3d& gps_loc_pose); //检查是否需要重定位

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg);

    //heading 回调函数
    void rtk_imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msgs);
     //发布gps定位
    void pub_gps_loc(ros::Publisher gps_pub, ros::Publisher heading_pub, pose_type pose);

};



#endif