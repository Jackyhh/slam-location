#include "livox_process.h"

ros::Publisher pub_cloud;
ros::Subscriber cloud_sub;


std::unique_ptr<cloud_process> pl_process_obj;


//激光雷达回调函数
void lidar_cb(const livox_ros_driver2::CustomMsg::ConstPtr &cloud_msg){
    pcl::PointCloud<PointType>::Ptr scan_cloud(new pcl::PointCloud<PointType>);

    scan_cloud = pl_process_obj->cloud_handler(cloud_msg);

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*scan_cloud, tempCloud);
    tempCloud.header.stamp = cloud_msg->header.stamp;
    tempCloud.header.frame_id = "laser_link";
    pub_cloud.publish(tempCloud);
}



//
int main(int argc, char** argv){
    ros::init(argc, argv, "livox_cloud_process");

    ROS_INFO("\033[1;32m---->\033[0m cloud process for livox Started.");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string package_name;
    nh_private.param<std::string>("package_name", package_name, "none");

    std::string config_file_name;
    nh_private.param<std::string>("config_file", config_file_name, "none");


    std::string pkg_path = ros::package::getPath(package_name);//获取当前包的路径地址
    std::string yaml_path= pkg_path+"/config/" + config_file_name;
    std::cout<< "Loc preprocess: yaml file: " <<yaml_path<<std::endl;

    YAML::Node config; //读取yaml文件
    try{
        config = YAML::LoadFile(yaml_path);
    } 
    catch(YAML::BadFile &e) {
        std::cout<<" Loc preprocess:  read config file error!"<<std::endl;
        return -1;
    }

    //初始化点云处理对象
    double filter_leaf_size = config["filter_leaf_size"].as<double>();
    pl_process_obj.reset(new cloud_process(filter_leaf_size));

    pl_process_obj->min_range_ = config["min_range"].as<double>();
    pl_process_obj->max_range_ = config["max_range"].as<double>();
    pl_process_obj->min_angle_ = config["limit_angle_min"].as<double>() * Deg2Rad;
    pl_process_obj->max_angle_ = config["limit_angle_max"].as<double>() * Deg2Rad;


    Eigen::Vector3f base2link_rpy;
    base2link_rpy(0) = config["lidar2base_roll"].as<float>();
    base2link_rpy(1) = config["lidar2base_pitch"].as<float>();
    base2link_rpy(2) = config["lidar2base_yaw"].as<float>();
    pl_process_obj->base2link_rot = Exp(base2link_rpy);


    //初始化订阅，发布器
    std::string lidar_topic = config["lidar_topic"].as<std::string>();
    std::cout<< " lidar_topic: " <<lidar_topic<<std::endl;
    cloud_sub = nh.subscribe(lidar_topic, 1, lidar_cb); // 激光点云订阅


    std::string cloud_topic = config["deskew_lidar_topic"].as<std::string>();;
    std::cout<< " cloud_topic: " <<cloud_topic<<std::endl;
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> (cloud_topic, 1);

    ros::spin();

    return 0;
}