#include "gps.h"

gps::gps(std::string package_path, std::string config_file_path, ros::NodeHandle &nh){

    origin_info_file_ = package_path + "/config/gps_origin_info.yaml"; //gps信息文件

    YAML::Node config = YAML::LoadFile(config_file_path);
    mapfile_name_ = config["map_filename"].as<std::string>();

    std::string rtk_topic = config["rtk_topic"].as<std::string>();
    std::string rtk_imu_topic = config["rtk_imu_topic"].as<std::string>();
    
    gps_sub_ = nh.subscribe(rtk_topic, 1, &gps::gps_cb, this); // GPS订阅
    rtk_imu_sub_ = nh.subscribe(rtk_imu_topic, 1, &gps::rtk_imu_cb, this); // RTK IMU订阅

    gps_data_.offset2base_x =  config["lidar2rtk_x"].as<double>();
    gps_data_.offset2base_y =  config["lidar2rtk_y"].as<double>();


    read_gps_origin();
}

void gps::write_gps_origin(nav_data_type gps_origin, Eigen::Vector3d loc_pose){
    YAML::Node config; //创建一个YAML::Node对象

    config["map_file_name"] = mapfile_name_;

    config["gps_origin"]["longitude"] = gps_origin.longitude;
    config["gps_origin"]["latitude"] = gps_origin.latitude;
    config["gps_origin"]["altitude"] = gps_origin.altitude;
    config["gps_origin"]["heading"] = gps_origin.heading;

    config["loc_origin"]["px"] = loc_pose(0);
    config["loc_origin"]["py"] = loc_pose(1);
    config["loc_origin"]["yaw"] = loc_pose(2);


    std::ofstream fout(origin_info_file_); //保存config为yaml文件
    fout << config;
    fout.close();

    std::cout << "GPS原点写入成功" << std::endl;
    read_gps_origin();
}

void gps::read_gps_origin(void){
    YAML::Node config;
    config = YAML::LoadFile(origin_info_file_); //读取yaml文件

    std::string map_name_in_info = config["map_file_name"].as<std::string>();

    if(mapfile_name_ != map_name_in_info){
        has_origin_ = false;
        return;
    }
    else{
        has_origin_ = true;
    }

 
    if(!has_origin_) return;

    //定位的原点对应的经纬度
    nav_data_type gps_origin;
    //定位的原点对应的odom坐标
    Eigen::Vector3d loc_origin;

    gps_origin.longitude = config["gps_origin"]["longitude"].as<double>();
    gps_origin.latitude = config["gps_origin"]["latitude"].as<double>();
    gps_origin.altitude = config["gps_origin"]["altitude"].as<double>();
    gps_origin.heading = config["gps_origin"]["heading"].as<double>();

    loc_origin(0) = config["loc_origin"]["px"].as<double>();
    loc_origin(1) = config["loc_origin"]["py"].as<double>();
    loc_origin(2) = config["loc_origin"]["yaw"].as<double>();


    gps_data_.update_data(gps_origin, loc_origin, false);
}

//获取信号质量
std::string gps::get_signal_quality(void){
    if(signal_quality_ == 2) return "good";
    else if(signal_quality_ == 1) return "normal";
    else return "bad";
} 

//使用GPS重定位
Eigen::Vector3d gps::gps_reloc(bool &is_ok){
    Eigen::Vector3d pose;
    is_ok = true; //默认gps定位成功
    if(!has_origin_){ //没有定位原点
        is_ok = false;
        return pose;
    }

    if(signal_quality_ < 2){ //信号质量差
        is_ok = false;
        return pose;
    }

    pose = gps_data_.gps2loc(gps_data_last_); //gps转loc坐标
    return pose; 
}

//检查是否需要重定位
bool gps::check_reloc(Eigen::Vector3d loc_pose, Eigen::Vector3d& gps_loc_pose){
    if(!has_origin_){return false;}
    if(signal_quality_ < 2){return false;}
    
    gps_loc_pose = gps_data_.gps2loc(gps_data_last_);
    double dx = loc_pose(0) - gps_loc_pose(0);
    double dy = loc_pose(1) - gps_loc_pose(1);
    double dist = std::sqrt(dx * dx + dy * dy);

    double dth = normalize_angle(loc_pose(2) - gps_loc_pose(2));

    //std::cout <<"gps check reloc : dx: "<<dx<<" dy: "<<dy<< " dyaw: "<< dth / Deg2Rad<< std::endl;


    double dist_thr, dth_thr;
    if(aktiv_gps_loc_){
        dist_thr = 3.0;
        dth_thr = 10.0 * Deg2Rad;
    }
    else{
        dist_thr = 4.0;
        dth_thr = 20.0 * Deg2Rad;
    }


    if(dist > dist_thr || dth > dth_thr){
        std::cout <<"aktiv_gps_loc_ : "<<std::endl;
        aktiv_gps_loc_ = true;

        return true;
    }

    std::cout <<"deaktiv_gps_loc_ : "<<std::endl;

    aktiv_gps_loc_ = false;
    return false;
} 


void gps::set_gps_start_pos(Eigen::Vector3d loc_pose){//设置gps起始位置
    if(is_init_ != 0x03) return;

    if(gps_data_.is_init == true) return;


    gps_data_.update_data(gps_data_last_, loc_pose, true);

    write_gps_origin(gps_data_.start_gps, gps_data_.start_loc);
    return;
}

bool gps::get_gps_start_pos_init(void){//获取gps起始位置是否初始化
    bool res = true;

    if(gps_data_.is_init == false || is_init_ != 0x03) 
        res = false;

    return res;
}


void gps::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg){//GPS回调函数
    gps_data_last_.timestamp = gpsMsg->header.stamp.toSec();

    //信号质量
    if(gpsMsg->status.status == 0) signal_quality_ = 0;
    else if(gpsMsg->status.status >= 16 && gpsMsg->status.status <= 33) signal_quality_ = 1;
    else signal_quality_ = 2;


    gps_data_last_.latitude = gpsMsg->latitude;
    gps_data_last_.longitude = gpsMsg->longitude;
    gps_data_last_.altitude = gpsMsg->altitude;

    is_init_ |= 0x01;
} 


//heading 回调函数
void gps::rtk_imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msgs){
    if(signal_quality_ < 2) return;


    double qx = imu_msgs->orientation.x;
    double qy = imu_msgs->orientation.y;
    double qz = imu_msgs->orientation.z;
    double qw = imu_msgs->orientation.w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

    double origin_heading = atan2(siny_cosp, cosy_cosp);
    gps_data_last_.heading = normalize_angle(origin_heading + 90.0 * Deg2Rad);

    //ROS_INFO("origin heading : %lf", origin_heading / Deg2Rad);

    //ROS_INFO("heading : %lf", gps_data_last_.heading / Deg2Rad);


    is_init_ |= 0x02;
}

//发布gps定位
void gps::pub_gps_loc(ros::Publisher gps_pub, ros::Publisher heading_pub, pose_type pose) {
    if(has_origin_ == false) return;

    sensor_msgs::NavSatFix nav_msgs;

    nav_msgs.header.frame_id = "agv_rtk_pose";
    nav_msgs.header.stamp = ros::Time().fromSec(pose.timestamp);

    Eigen::Vector3d loc_pose;
    loc_pose(0) = pose.pos(0);
    loc_pose(1) = pose.pos(1);
    loc_pose(2) = pose.orient(2);

    nav_data_type gps_coordi = gps_data_.loc2gps(loc_pose);
    nav_msgs.latitude = gps_coordi.latitude;
    nav_msgs.longitude = gps_coordi.longitude;
    nav_msgs.altitude = gps_coordi.altitude;

    std_msgs::Float64 heading_msgs;
    heading_msgs.data = normalize_angle(normalize_angle(pose.orient(2) + gps_data_.map_heading) - 90.0 * Deg2Rad) / Deg2Rad;

    gps_pub.publish(nav_msgs);
    heading_pub.publish(heading_msgs);

}