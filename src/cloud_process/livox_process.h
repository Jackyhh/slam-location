#ifndef _PROCESS_H_
#define _PROCESS_H_

#include "utility.h"
#include "livox_ros_driver2/CustomMsg.h"


typedef pcl::PointXYZI PointType;


//点云前处理
class cloud_process{
public:

    double scan_time_;
    double min_angle_;
    double max_angle_;
    double min_range_;
    double max_range_;

    Eigen::Matrix3f base2link_rot;

    pcl::VoxelGrid<PointType> down_size_filter_;

    cloud_process(double leaf_size);

    pcl::PointCloud<PointType>::Ptr cloud_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
};








#endif