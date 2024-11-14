#include "livox_process.h"


cloud_process::cloud_process(double leaf_size){
    down_size_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}



pcl::PointCloud<PointType>::Ptr cloud_process::cloud_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg){

    int plsize = msg->point_num;
    scan_time_ = msg->header.stamp.toSec();


    pcl::PointCloud<PointType>::Ptr pl_full(new pcl::PointCloud<PointType>);
    pl_full->clear();

    PointType pt;
    for(uint i=1; i<plsize; i++){
        if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00){
            // pt.x = msg->points[i].x;
            // pt.y = msg->points[i].y;
            // pt.z = msg->points[i].z;

            pt.x = base2link_rot(0,0) * msg->points[i].x + base2link_rot(0,1) * msg->points[i].y + base2link_rot(0,2) * msg->points[i].z;
            pt.y = base2link_rot(1,0) * msg->points[i].x + base2link_rot(1,1) * msg->points[i].y + base2link_rot(1,2) * msg->points[i].z;
            pt.z = base2link_rot(2,0) * msg->points[i].x + base2link_rot(2,1) * msg->points[i].y + base2link_rot(2,2) * msg->points[i].z;
            pt.intensity = msg->points[i].reflectivity;



            double dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            double angle = atan2(pt.y, pt.x);

            if(dist < min_range_ || dist > max_range_){
                continue;
            }

            if(angle < min_angle_ || angle > max_angle_){
                continue;
            }

            pl_full->points.push_back(pt);
        }
    }


    pcl::PointCloud<PointType>::Ptr res_cloud(new pcl::PointCloud<PointType>);
    res_cloud->clear();

    down_size_filter_.setInputCloud(pl_full);
    down_size_filter_.filter(*res_cloud);

    res_cloud->width = res_cloud->points.size();
    res_cloud->height = 1;

    return res_cloud;  
}




