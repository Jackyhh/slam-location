#include "p2pl_icp.h"
#include "livox_ros_driver2/CustomMsg.h"

pcl::PointCloud<PointType>::Ptr cloud_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg){

    int plsize = msg->point_num;
    pcl::PointCloud<PointType>::Ptr pl_full(new pcl::PointCloud<PointType>);
    pl_full->clear();

    PointType pt;
    for(uint i=1; i<plsize; i++){
        if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00){
            pt.x = msg->points[i].x;
            pt.y = msg->points[i].y;
            pt.z = msg->points[i].z;
            pt.intensity = msg->points[i].reflectivity;

            pl_full->points.push_back(pt);
        }
    }

    pl_full->width = pl_full->points.size();
    pl_full->height = 1;

    return pl_full;  
}

//发布结果点云
void publish_cloud(ros::Publisher &pub, pcl::PointCloud<PointType>::Ptr &cloud, Eigen::Vector3f pos, Eigen::Vector3f rpy){
    int n_size = cloud->size();

    pcl::PointCloud<PointType>::Ptr cloud_in_world(new pcl::PointCloud<PointType>);
    cloud_in_world->clear();

    Eigen::Matrix3f R = Exp(rpy);

    for(int i=0; i<n_size; i++){
        PointType pt = cloud->points[i];
        PointType pt_in_world;

        pt_in_world.x = R(0,0)*pt.x + R(0,1)*pt.y + R(0,2)*pt.z + pos(0);
        pt_in_world.y = R(1,0)*pt.x + R(1,1)*pt.y + R(1,2)*pt.z + pos(1);
        pt_in_world.z = R(2,0)*pt.x + R(2,1)*pt.y + R(2,2)*pt.z + pos(2);
        pt_in_world.intensity = pt.intensity;

        cloud_in_world->points.push_back(pt_in_world);
    }

    cloud_in_world->width = cloud_in_world->points.size();
    cloud_in_world->height = 1;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_in_world, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    pub.publish(cloud_msg);
}


class lidar_calib{   
public:
    ros::NodeHandle nh_; //ros节点对象
    ros::NodeHandle nh_private_; //私人节点对象

    pcl::PointCloud<PointType>::Ptr map_cloud; //地图点云

    ros::Subscriber main_lidar_sub_;
    ros::Subscriber sub1_lidar_sub_;
    ros::Subscriber sub2_lidar_sub_;

    ros::Publisher map_pub_; //发布地图
    ros::Publisher sub1_lidar_pub_; //发布地图
    ros::Publisher sub2_lidar_pub_; //发布地图

    std::unique_ptr<scan2map3d> localizier3d_;

    pose_type sub1_lidar_pose_; //初始位姿
    pose_type sub2_lidar_pose_; //初始位姿

    pcl::PointCloud<PointType>::Ptr sub1_lidar_cloud_;
    pcl::PointCloud<PointType>::Ptr sub2_lidar_cloud_;

    double lidar_timestamp_{0.0};

    bool finish_{false};

    std::string config_path_;

    lidar_calib(ros::NodeHandle nh, ros::NodeHandle nh_private){
        nh_ = nh;
        nh_private_ = nh_private;
        YAML::Node config;

        std::string package_name;
        nh_private_.param<std::string>("package_name", package_name, "none");

        std::string pkg_path = ros::package::getPath(package_name);//获取当前包的路径地址
        config_path_ = pkg_path+"/config/lidar_calib_config.yaml";
        std::cout<< "Loc: yaml file: " <<config_path_<<std::endl;



        //读取yaml数据
        try{
            config = YAML::LoadFile(config_path_);
        } 
        catch(YAML::BadFile &e) {
            std::cout<<" Loc :  read config file error!"<<std::endl;
            return;
        }

        map_cloud.reset(new pcl::PointCloud<PointType>());
        map_cloud->clear();

        pcl::PointCloud<PointType>::Ptr sub1_lidar_cloud_(new pcl::PointCloud<PointType>);
        sub1_lidar_cloud_->clear();

        pcl::PointCloud<PointType>::Ptr sub2_lidar_cloud_(new pcl::PointCloud<PointType>);
        sub2_lidar_cloud_->clear();


        YAML::Node pose_cfg;

        pose_cfg = config["sub1_lidar_pose"];
        for (int i = 0; i < pose_cfg.size(); ++i) {
            if(i < 3) sub1_lidar_pose_.pos(i) = pose_cfg[i].as<double>();
            else if(i >= 3 && i < 6) sub1_lidar_pose_.orient(i - 3) = pose_cfg[i].as<double>() * Deg2Rad;
        }

        pose_cfg = config["sub2_lidar_pose"];
        for (int i = 0; i < pose_cfg.size(); ++i) {
            if(i < 3) sub2_lidar_pose_.pos(i) = pose_cfg[i].as<double>();
            else if(i >= 3 && i < 6) sub2_lidar_pose_.orient(i - 3) = pose_cfg[i].as<double>() * Deg2Rad;
        }

        //初始化订阅，发布器
        std::string lidar_main_topic = config["main_lidar_topic"].as<std::string>();
        main_lidar_sub_ = nh_.subscribe(lidar_main_topic, 1, &lidar_calib::main_lidar_cb, this); // 激光点云订阅
        
        std::string lidar_sub1_topic = config["sub1_lidar_topic"].as<std::string>();
        sub1_lidar_sub_ = nh_.subscribe(lidar_sub1_topic, 1, &lidar_calib::sub1_lidar_cb, this); // 激光点云订阅

        std::string lidar_sub2_topic = config["sub2_lidar_topic"].as<std::string>();
        sub2_lidar_sub_ = nh_.subscribe(lidar_sub2_topic, 1, &lidar_calib::sub2_lidar_cb, this); // 激光点云订阅
    

        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 1); // 地图点云发布
        sub1_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sub1_lidar_cloud", 1); // sub1_lidar点云发布
        sub2_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sub2_lidar_cloud", 1); // sub2_lidar点云发布

    }

    void main_lidar_cb(const livox_ros_driver2::CustomMsg::ConstPtr &cloud_msg){
        if(finish_ ) return;

        pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
        cloud_in = cloud_handler(cloud_msg);

        *map_cloud += *cloud_in; //构建点云地图

        if(lidar_timestamp_ < 0.1){
            lidar_timestamp_ = ros::Time::now().toSec();
            return;
        }

        double time2start = ros::Time::now().toSec() - lidar_timestamp_;
        if(time2start > 5.0){
            pcl::VoxelGrid<PointType> down_size_filter_; //下采样
            down_size_filter_.setLeafSize(0.2, 0.2, 0.2);

            pcl::PointCloud<PointType>::Ptr downsample_cloud(new pcl::PointCloud<PointType>);
            downsample_cloud->clear();

            down_size_filter_.setInputCloud(map_cloud);
            down_size_filter_.filter(*downsample_cloud);

            localizier3d_.reset(new scan2map3d(downsample_cloud, config_path_));
            localizier3d_->setNormalMode();

            sub1_lidar_pose_ = localizier3d_->location(sub1_lidar_pose_, sub1_lidar_cloud_);
            sub2_lidar_pose_ = localizier3d_->location(sub2_lidar_pose_, sub2_lidar_cloud_);

            std::cout << "sub1_lidar_pos: " << sub1_lidar_pose_.pos << std::endl;
            std::cout << "sub1_lidar_rpy: " << sub1_lidar_pose_.orient << std::endl;
            std::cout << "============================================="  << std::endl;   
            std::cout << "sub2_lidar_pos: " << sub2_lidar_pose_.pos << std::endl;
            std::cout << "sub2_lidar_rpy: " << sub2_lidar_pose_.orient << std::endl;

            finish_ = true;

            Eigen::Vector3f main_lidar_pos(0.0f, 0.0f, 0.0f);
            Eigen::Vector3f main_lidar_rpy(0.0f, 0.0f, 0.0f);


            publish_cloud(map_pub_, map_cloud, main_lidar_pos, main_lidar_rpy);
            publish_cloud(sub1_lidar_pub_, sub1_lidar_cloud_, sub1_lidar_pose_.pos, sub1_lidar_pose_.orient);
            publish_cloud(sub2_lidar_pub_, sub2_lidar_cloud_, sub2_lidar_pose_.pos, sub2_lidar_pose_.orient);

            ros::shutdown();
        }

    }

    void sub1_lidar_cb(const livox_ros_driver2::CustomMsg::ConstPtr &cloud_msg){
        if(finish_ ) return;
        sub1_lidar_cloud_ = cloud_handler(cloud_msg);
    }

    void sub2_lidar_cb(const livox_ros_driver2::CustomMsg::ConstPtr &cloud_msg){
        if(finish_ ) return;
        sub2_lidar_cloud_ = cloud_handler(cloud_msg);
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv, "lidar calib");

    ROS_INFO("\033[1;32m---->\033[0m Lidar Calib Started.");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    lidar_calib calib_obj(nh, nh_private);
    ros::spin();

    return 0;
}