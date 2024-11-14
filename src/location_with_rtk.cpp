#include "p2pl_icp.h"
#include "eskf.h"
#include "gps.h"


#define AKTIV_CODE           23450.0

static bool use_safety = true;

struct runtime_type{
    float run_time;
    float max_run_time;
    float aktiv_code;
    int cnt;
    std::string runtime_filename;
    bool file_exist;
};


bool check_imu_data(Eigen::VectorXd imu_data){
    bool is_ok = true;
    if(std::abs(imu_data(0)) > 10000.0) is_ok = false;
    if(std::abs(imu_data(1)) > 10000.0) is_ok = false;
    if(std::abs(imu_data(2)) > 10000.0) is_ok = false;
    if(std::abs(imu_data(3)) > 10000.0) is_ok = false;
    if(std::abs(imu_data(4)) > 10000.0) is_ok = false;
    if(std::abs(imu_data(5)) > 10000.0) is_ok = false;

    return is_ok;
}

class location{    
public:
    ros::NodeHandle nh_; //ros节点对象
    ros::NodeHandle nh_private_; //私人节点对象

    std::string map_file_path; //地图路径

    pcl::PointCloud<PointType>::Ptr map_cloud; //地图点云

    pose_type act_pose; //当前位姿
    pose_type init_pose_; //初始位姿
    pose_type lidar2base_pose; //激光雷达到baselink相对位姿

    sensor_msgs::PointCloud2 pubmap_msg; //作为消息播放的地图
    ros::Publisher pc_map_pub_; //发布地图
    ros::Publisher pose_pub_; //发布当前位姿
    ros::Publisher test_info_pub_; //发布定位指标状态
    ros::Publisher gps_pub_; //发布GPS坐标数据
    ros::Publisher heading_pub_; //发布航向角数据

    ros::Subscriber cloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber init_pose_sub_;

    tf::TransformBroadcaster tf_broadcaster; //通用tf广播器

    pcl::VoxelGrid<PointType> map_voxelgrid_filter_; //地图点云体素滤波器

    std::unique_ptr<scan2map3d> localizier3d;
    std::unique_ptr<ErrorStateKalmanFilter> eskf; //误差状态卡尔曼滤波
    std::unique_ptr<gps> gps_obj; //GPS对象

    // imu input buffer
    std::mutex imu_data_mutex_;
    std::vector<sensor_msgs::ImuConstPtr> imu_buffer_;

    sensor_msgs::PointCloud2::ConstPtr cloud_msg_;
    bool rece_flag{false};

    uint8_t req_reloc_{2};
    int count_maplost_{0};

    int normal_reloc_try_{0};


    bool enable_test_info_{false};

    bool lock_system_{false};

    runtime_type runtime_controller;

    bool use_imu_{true}; //是否使用IMU融合？
    bool use_reloc_{false};//是否使用重定位？

    float reloc_threshold_; //重定位的匹配度阈值

    double store_pose_timestamp_;//保存位置的时间戳

    double imu_scale_; //imu_scale

    Eigen::Matrix3f base2link_rot_; //base2link的旋转矩阵

    std::string pkg_path_; //包路径
    
    location(ros::NodeHandle nh, ros::NodeHandle nh_private){
        nh_ = nh;
        YAML::Node config;

        double voxel_leaf_size_;

        
        nh_private_ = nh_private;

        std::string config_file_name;
        nh_private_.param<std::string>("config_file", config_file_name, "none");
        std::cout<< "Loc: config file: " <<config_file_name<<std::endl;

        std::string package_name;
        nh_private_.param<std::string>("package_name", package_name, "none");

        //是否使用imu
        nh_private_.param<bool>("use_imu", use_imu_, true);
        std::cout << "use imu: " << use_imu_ <<std::endl;
        //是否使用重定位
        nh_private_.param<bool>("use_reloc", use_reloc_, false);

        pkg_path_ = ros::package::getPath(package_name);//获取当前包的路径地址
        std::string yaml_path= pkg_path_+"/config/" + config_file_name;
        std::cout<< "Loc: yaml file: " <<yaml_path<<std::endl;

        //读取yaml数据
        try{
            config = YAML::LoadFile(yaml_path);
        } 
        catch(YAML::BadFile &e) {
            std::cout<<" Loc :  read config file error!"<<std::endl;
            return;
        }

        pc_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 10, true);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("agv_pose", 10, true);
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("agv_gps_pos", 1, true);
        heading_pub_ = nh_.advertise<std_msgs::Float64>("agv_gps_heading", 1, true);

        test_info_pub_ = nh_.advertise<std_msgs::String>("loc_info", 10, true);


        map_file_path = config["map_filename"].as<std::string>(); //地图路径
        map_file_path = pkg_path_ + "/map/" + map_file_path;
        //nh.param<std::string>("loc_map_path", map_file_path, "none");
        std::cout << "loc: map_path: " << map_file_path << std::endl;


        pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);
        
        //读取地图
        if (pcl::io::loadPCDFile(map_file_path.c_str(), *map_cloud) == -1) {
            std::cout << " Loc :  load map failed " << map_file_path << std::endl;
            return;
        }

        //发布地图
        pcl::toROSMsg(*map_cloud, pubmap_msg);
        if (pubmap_msg.width != 0) {
            pubmap_msg.header.frame_id = "map";
            pc_map_pub_.publish(pubmap_msg);
            std::cout << " Loc :  map has be published"  << std::endl;
        }
        else{
            std::cout << " Loc :  can not publish map"  << std::endl;
            return;
        }

        localizier3d.reset(new scan2map3d(map_cloud, yaml_path));
        localizier3d->setNormalMode();

        //初始化位姿
        //读取pose record数据
        YAML::Node pose_record_config;
        try{
            pose_record_config = YAML::LoadFile(pkg_path_ + "/src/pose_record.yaml");
        } 
        catch(YAML::BadFile &e) {
            std::cout<<" Loc :  read pose record file error!"<<std::endl;
            return;
        }
        init_pose_.pos(0) = pose_record_config["px"].as<float>();
        init_pose_.pos(1) = pose_record_config["py"].as<float>();
        init_pose_.pos(2) = pose_record_config["pz"].as<float>();
        init_pose_.orient(0) = pose_record_config["roll"].as<float>();
        init_pose_.orient(1) = pose_record_config["pitch"].as<float>();
        init_pose_.orient(2) = pose_record_config["yaw"].as<float>();

        std::cout << "init pos: "<< init_pose_.pos << std::endl;

        update_q(init_pose_);
        act_pose = init_pose_;
        act_pose.vel << 0.0, 0.0, 0.0;

        //初始化激光雷达到baselink的转置矩阵
        lidar2base_pose.pos(0) = config["lidar2base_x"].as<float>();
        lidar2base_pose.pos(1) = config["lidar2base_y"].as<float>();
        lidar2base_pose.pos(2) = config["lidar2base_z"].as<float>();
        lidar2base_pose.orient(0) = config["lidar2base_roll"].as<float>();
        lidar2base_pose.orient(1) = config["lidar2base_pitch"].as<float>();
        lidar2base_pose.orient(2) = config["lidar2base_yaw"].as<float>();
        update_q(lidar2base_pose);


        base2link_rot_ = Exp(lidar2base_pose.orient); //激光雷达到baselink的旋转矩阵


        reloc_threshold_ = config["reloc_threshold"].as<float>();

        //是否使用IMU
        imu_buffer_.clear();

        //初始化ukf
        std::string eskf_cfg_name;
        nh_private.param<std::string>("eskf_cfg_file", eskf_cfg_name, "eskf_cfg.yaml");
        eskf.reset(new ErrorStateKalmanFilter(pkg_path_ + "/config/" + eskf_cfg_name));
        
        //初始化GPS
        gps_obj.reset(new gps(pkg_path_, yaml_path, nh_));

        enable_test_info_ = config["enable_pub_test_info"].as<bool>();

        //初始化订阅，发布器
        std::string lidar_topic = config["deskew_lidar_topic"].as<std::string>();
        cloud_sub_ = nh_.subscribe(lidar_topic, 1, &location::robosense_cb, this); // 激光点云订阅
        
        std::string cmd_topic = config["cmd_topic"].as<std::string>();
        cmd_sub_ = nh_.subscribe(cmd_topic, 1, &location::command_cb, this); // 手动重定位订阅

        if(use_imu_){
            std::string imu_topic = config["imu_topic"].as<std::string>();
            imu_sub_ = nh_.subscribe(imu_topic, 20, &location::imu_cb, this); // IMU订阅
        }

        imu_scale_ = config["imu_scale"].as<double>();

        init_pose_sub_ = nh_.subscribe("/initialpose", 1, &location::initpose_cb, this); // 手动重定位订阅

        runtime_controller.runtime_filename = pkg_path_ +"/config/loc_data.bin";
        readBinFile(runtime_controller.runtime_filename);

        store_pose_timestamp_ = ros::Time::now().toSec();
    }

    //IMU反馈
    void imu_cb(const sensor_msgs::Imu::ConstPtr &imu_msg){
        if(use_imu_ == false) return;
        
        std::lock_guard<std::mutex> lock(imu_data_mutex_);
        imu_buffer_.push_back(imu_msg);
    }

    
    
    //激光点云反馈
    void robosense_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg){
        if(use_safety){
            if(lock_system_){
                ROS_WARN("erlaubnis is abgelaufen, bitte verlangert es!!!");
                return;
            }
        }

        cloud_msg_ = cloud_msg;
        rece_flag = true;
    }

    //初始位姿反馈
    void initpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg){
        ROS_INFO("get hand init pose!"); 
        act_pose.pos(0) = pose_msg->pose.pose.position.x;
        act_pose.pos(1) = pose_msg->pose.pose.position.y;
        act_pose.pos(2) = 0.0;


        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * pose_msg->pose.pose.orientation.w * pose_msg->pose.pose.orientation.z;
        double cosy_cosp = 1.0 - 2.0 * pose_msg->pose.pose.orientation.z * pose_msg->pose.pose.orientation.z;

        act_pose.orient(0) = 0.0;
        act_pose.orient(1) = 0.0;
        act_pose.orient(2) = atan2(siny_cosp, cosy_cosp);
        

        act_pose.vel << 0.0, 0.0, 0.0;


        req_reloc_ = 2;
        store_pose_timestamp_ = ros::Time::now().toSec();
    }



    //状态命令反馈
    void command_cb(const std_msgs::String::ConstPtr& msg){
        
        std::cout << "cmd: " << msg->data.c_str() << std::endl;

        if(msg->data == "shutdown"){
            ros::shutdown();
        }
        else if(msg->data == "test_reloc"){
            ROS_INFO("test reloc");
            act_pose.pos(0) = act_pose.pos(0) + 5.0;
            act_pose.pos(1) = 4.0;
        }
    }





    //读取bin文件
    void readBinFile(std::string filename){
        float *data = new float[3];

        std::ifstream inF(filename, std::ios::binary);
        if(inF.is_open() == false){
            runtime_controller.file_exist = false;
            lock_system_ = true;
        }
        else{
            runtime_controller.file_exist = true;
        } 

        inF.read((char *)data, sizeof(float)*3);
        inF.close();

        runtime_controller.run_time = data[0];
        runtime_controller.max_run_time = data[1];
        runtime_controller.aktiv_code = data[2];

        if(std::abs(runtime_controller.aktiv_code - AKTIV_CODE) > 1.0)
            lock_system_ = true;
    }

    //写bin文件
    void writeBinFile(std::string filename){
        FILE *fp = fopen(filename.c_str(), "w");

        float data[3];
        data[0] = runtime_controller.run_time;
        data[1] = runtime_controller.max_run_time;
        data[2] = AKTIV_CODE;

        fwrite(data, sizeof(float), sizeof(data), fp);
        fclose(fp);
    }


    //发布tf格式的位姿
    void publish_tf(pose_type pose){
        geometry_msgs::TransformStamped transform_lidar_stamped;
        transform_lidar_stamped.header.frame_id = "map";
        transform_lidar_stamped.child_frame_id = "laser_link";
        transform_lidar_stamped.header.stamp = ros::Time().fromSec(pose.timestamp);

        transform_lidar_stamped.transform.translation.x = static_cast<double>(pose.pos(0));
        transform_lidar_stamped.transform.translation.y = static_cast<double>(pose.pos(1));
        transform_lidar_stamped.transform.translation.z = static_cast<double>(pose.pos(2));

        transform_lidar_stamped.transform.rotation.x = static_cast<double>(pose.q[1]);
        transform_lidar_stamped.transform.rotation.y = static_cast<double>(pose.q[2]);
        transform_lidar_stamped.transform.rotation.z = static_cast<double>(pose.q[3]);
        transform_lidar_stamped.transform.rotation.w = static_cast<double>(pose.q[0]);

        tf_broadcaster.sendTransform(transform_lidar_stamped);



        geometry_msgs::TransformStamped transform_base_stamped;
        transform_base_stamped.header.frame_id = "laser_link";
        transform_base_stamped.child_frame_id = "base_link";
        transform_base_stamped.header.stamp = ros::Time().fromSec(pose.timestamp);;

        transform_base_stamped.transform.translation.x = static_cast<double>(lidar2base_pose.pos(0));
        transform_base_stamped.transform.translation.y = static_cast<double>(lidar2base_pose.pos(1));
        transform_base_stamped.transform.translation.z = static_cast<double>(lidar2base_pose.pos(2));

        transform_base_stamped.transform.rotation.x = static_cast<double>(lidar2base_pose.q[1]);
        transform_base_stamped.transform.rotation.y = static_cast<double>(lidar2base_pose.q[2]);
        transform_base_stamped.transform.rotation.z = static_cast<double>(lidar2base_pose.q[3]);
        transform_base_stamped.transform.rotation.w = static_cast<double>(lidar2base_pose.q[0]);

        tf_broadcaster.sendTransform(transform_base_stamped);
    }


    //发布当前位置
    void publish_pose(pose_type pose){
        geometry_msgs::PoseStamped pose_stamped;
        if(req_reloc_ > 0){
            if(count_maplost_ >= 2){pose_stamped.header.frame_id = "lost_map";}
            else{
                pose_stamped.header.frame_id = "location";
                count_maplost_++;
            }
        }
        else{
            pose_stamped.header.frame_id = "location";
            count_maplost_ = 0;
        }

        float s_yaw = sin(pose.orient(2));
        float c_yaw = cos(pose.orient(2));


        pose_type base_pose = pose;
        base_pose.pos(0) = c_yaw * lidar2base_pose.pos(0) - s_yaw * lidar2base_pose.pos(1) + pose.pos(0);
        base_pose.pos(1) = s_yaw * lidar2base_pose.pos(0) + c_yaw * lidar2base_pose.pos(1) + pose.pos(1);
        base_pose.orient += lidar2base_pose.orient;
        update_q(base_pose);

        pose_stamped.header.stamp = ros::Time().fromSec(pose.timestamp);;

        pose_stamped.pose.position.x = static_cast<double>(base_pose.pos(0));
        pose_stamped.pose.position.y = static_cast<double>(base_pose.pos(1));
        pose_stamped.pose.position.z = static_cast<double>(base_pose.pos(2));

        pose_stamped.pose.orientation.x = static_cast<double>(base_pose.q[1]);
        pose_stamped.pose.orientation.y = static_cast<double>(base_pose.q[2]);
        pose_stamped.pose.orientation.z = static_cast<double>(base_pose.q[3]);
        pose_stamped.pose.orientation.w = static_cast<double>(base_pose.q[0]);

        pose_pub_.publish(pose_stamped);
    }

    //卡尔曼滤波的预测
    void imu_predict(double scan_timestamp){
        if(use_imu_ == false) return;


        std::lock_guard<std::mutex> lock(imu_data_mutex_); //写保护
        auto imu_iter = imu_buffer_.begin();


        for(imu_iter; imu_iter != imu_buffer_.end(); imu_iter++) {
            double imu_timestamp = (*imu_iter)->header.stamp.toSec();

            if(scan_timestamp < imu_timestamp){
                break;
            } 
                
            Eigen::VectorXd imu_data_pre = Eigen::VectorXd::Zero(6);
            imu_data_pre(0) = (*imu_iter)->linear_acceleration.x * imu_scale_;
            imu_data_pre(1) = (*imu_iter)->linear_acceleration.y * imu_scale_;
            imu_data_pre(2) = (*imu_iter)->linear_acceleration.z * imu_scale_;
            imu_data_pre(3) = (*imu_iter)->angular_velocity.x;
            imu_data_pre(4) = (*imu_iter)->angular_velocity.y;
            imu_data_pre(5) = (*imu_iter)->angular_velocity.z;


            Eigen::VectorXd imu_data = Eigen::VectorXd::Zero(6);
            imu_data(0) = base2link_rot_(0,0) * imu_data_pre(0) + base2link_rot_(0,1) * imu_data_pre(1) + base2link_rot_(0,2) * imu_data_pre(2);
            imu_data(1) = base2link_rot_(1,0) * imu_data_pre(0) + base2link_rot_(1,1) * imu_data_pre(1) + base2link_rot_(1,2) * imu_data_pre(2);
            imu_data(2) = base2link_rot_(2,0) * imu_data_pre(0) + base2link_rot_(2,1) * imu_data_pre(1) + base2link_rot_(2,2) * imu_data_pre(2);
            imu_data(3) = base2link_rot_(0,0) * imu_data_pre(3) + base2link_rot_(0,1) * imu_data_pre(4) + base2link_rot_(0,2) * imu_data_pre(5);
            imu_data(4) = base2link_rot_(1,0) * imu_data_pre(3) + base2link_rot_(1,1) * imu_data_pre(4) + base2link_rot_(1,2) * imu_data_pre(5);
            imu_data(5) = base2link_rot_(2,0) * imu_data_pre(3) + base2link_rot_(2,1) * imu_data_pre(4) + base2link_rot_(2,2) * imu_data_pre(5);
    

            if(check_imu_data(imu_data)){
                eskf->predict(imu_data, imu_timestamp); //ieskf 预测
                eskf->getPose(act_pose);
            }     
        }
        imu_buffer_.erase(imu_buffer_.begin(), imu_iter);
    }


    void runtime_loop(void){
        ros::Rate loop_rate(0.1);
        runtime_controller.cnt = 0;

        YAML::Node pose_record_node; //创建一个YAML::Node对象
        pose_record_node["px"] = act_pose.pos(0);
        pose_record_node["py"] = act_pose.pos(1);
        pose_record_node["pz"] = act_pose.pos(2);
        pose_record_node["roll"] = act_pose.orient(0);
        pose_record_node["pitch"] = act_pose.orient(1);
        pose_record_node["yaw"] = act_pose.orient(2);

        std::string record_filename = pkg_path_ + "/src/pose_record.yaml";

        while(ros::ok()){
            //保存当前位置
            if((ros::Time::now().toSec() - store_pose_timestamp_) > 2.0){
                pose_record_node["px"] = act_pose.pos(0);
                pose_record_node["py"] = act_pose.pos(1);
                pose_record_node["pz"] = act_pose.pos(2);
                pose_record_node["roll"] = act_pose.orient(0);
                pose_record_node["pitch"] = act_pose.orient(1);
                pose_record_node["yaw"] = act_pose.orient(2);

                std::ofstream fout(record_filename); //保存config为yaml文件
                fout << pose_record_node;
                fout.close();

                store_pose_timestamp_ = ros::Time::now().toSec();
            }


            if(runtime_controller.file_exist){
                runtime_controller.run_time += 1.0/360.0;
                if(runtime_controller.run_time > runtime_controller.max_run_time){
                    lock_system_ = true;
                    break;
                }
                runtime_controller.cnt++;
                if(runtime_controller.cnt > 30){
                    writeBinFile(runtime_controller.runtime_filename);
                    runtime_controller.cnt = 0;
                }
            }
            loop_rate.sleep();
        }
        ROS_WARN(" LOC :  quit runtime loop");
    }


    //循环工作
    void loop(void){
        //激光雷达的数据
        pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);


        Eigen::Matrix4f world_pose_matrix;

        ros::Rate loop_rate(25);
        while(ros::ok()){
            if(rece_flag){//接收到点云
                pcl::fromROSMsg(*cloud_msg_, *cloud_in);

                if(req_reloc_ > 0){ //重定位
                    std::cout << " Loc :  begin to relocation !!! "<< std::endl;

                    bool is_gps_ok = false;
                    Eigen::Vector3d gps_pose;
                    if(req_reloc_ == 2 && gps_obj->get_gps_start_pos_init() == true){
                        gps_pose = gps_obj->gps_reloc(is_gps_ok);
                        if(is_gps_ok){
                            act_pose.pos(0) = gps_pose(0);
                            act_pose.pos(1) = gps_pose(1);
                            act_pose.orient(2) = gps_pose(2);
                            req_reloc_ = 0;
                        }
                        else{req_reloc_ = 1;} 
                    }
                    else{req_reloc_ = 1;}

                    if(req_reloc_ == 1){
                        localizier3d->setRelocMode();
                        act_pose = localizier3d->location(act_pose, cloud_in);
                        localizier3d->setNormalMode();

                        if(localizier3d->fitness_ > (reloc_threshold_ * 1.2)){
                            req_reloc_ = 0;
                        } 
                    }
                    
                    //IMU预积分，这里重置一下
                    if(use_imu_){ 
                        eskf->setMean(act_pose.pos.cast<double>(), act_pose.orient.cast<double>(), act_pose.vel.cast<double>());
                        eskf->reset();
                        eskf->correct(act_pose);
                    }
                }
                else{
                    //imu 预积分
                    imu_predict(cloud_msg_->header.stamp.toSec());
                    
                    //输入当前点云进行定位
                    act_pose = localizier3d->location(act_pose, cloud_in);
                                        
                    //如果使用IMU预积分，做correct过程
                    if(use_imu_){ 
                        eskf->correct(act_pose);
                    }


                    Eigen::Vector3d gps_pose;
                    gps_pose(0) = act_pose.pos(0);
                    gps_pose(1) = act_pose.pos(1);
                    gps_pose(2) = act_pose.orient(2);
                    //是否需要初始化GPS开机信息
                    if(gps_obj->get_gps_start_pos_init() == false){
                        gps_obj->set_gps_start_pos(gps_pose);
                    }


                    //判断是否需要重定位
                    Eigen::Vector3d gps_out_pose;
                    if(localizier3d->fitness_ < reloc_threshold_){
                        req_reloc_ = 2;
                    }
                    else if(gps_obj->check_reloc(gps_pose, gps_out_pose) == true){
                        act_pose.pos(0) = gps_out_pose(0);
                        act_pose.pos(1) = gps_out_pose(1);
                        act_pose.orient(2) = gps_out_pose(2);

                        req_reloc_ = 2;
                    }


                
                }
                rece_flag = false;
                act_pose.timestamp = cloud_msg_->header.stamp.toSec();

                publish_tf(act_pose); //发布tf格式的位姿
                publish_pose(act_pose);

                gps_obj->pub_gps_loc(gps_pub_, heading_pub_, act_pose);

                if(std::isnan(act_pose.pos(0)) || std::isnan(act_pose.pos(1)) || std::isnan(act_pose.pos(2))){
                    ROS_INFO("agv location : restart");
                    ros::shutdown();
                }

                //发布定位相关指标信息
                if(enable_test_info_){
                    std_msgs::String test_str;
                    test_str.data = "{'iteration': " + std::to_string(localizier3d->iter_count_) + 
                                    ",'use_imu': " + std::to_string(use_imu_) + 
                                    ",'fitniss': " + std::to_string(localizier3d->fitness_) + 
                                    ",'cal_during': " + std::to_string(localizier3d->during_time_) + 
                                    ",'is_relocation': " + std::to_string(req_reloc_) + "}";

                    test_info_pub_.publish(test_str);
                }

            }
            loop_rate.sleep();
        }

        ROS_WARN(" LOC :  quit main loop");
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_location_3d_with_rtk");

    ROS_INFO("\033[1;32m---->\033[0m Lidar Location 3d with rtk Started.");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");


    location loc(nh, nh_private);
    std::thread mainthread(&location::loop, &loc);
    std::thread saftythread(&location::runtime_loop, &loc);

    ros::spin();

    mainthread.join();
    saftythread.join();
    

    return 0;
}