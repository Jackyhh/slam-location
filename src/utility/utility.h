#ifndef _UTILITY_H_
#define _UTILITY_H_


#include "param.h"

#include "ros/ros.h"
#include "ros/package.h"

#include <atomic>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <chrono>
#include <deque>
#include <random>
#include <math.h>
#include <unordered_map>
#include <functional>


#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZI PointType;

//位姿因子
struct pose_type{
    double timestamp;
    std::string status;

    Eigen::Vector3f pos;
    Eigen::Vector3f orient;
    Eigen::Vector3f vel;
    float q[4];

    Eigen::Matrix3f R;
    Eigen::Vector3f bias_g;
    Eigen::Vector3f bias_a;
    Eigen::Vector3f gravity;

    Eigen::Matrix<float, 15, 15> cov;

    double lidar_this_time;
    double last_update_time;
};




/*rpy加位移转4x4矩阵*/
Eigen::Matrix4f Pose2Matrix(pose_type pose);
pose_type Matrix2Pose(Eigen::Matrix4f mat);
void update_q(pose_type &pose);
Eigen::Quaternion<double> rpy2q(double roll, double pitch, double yaw);
Eigen::Vector3d q2rpy(double qx, double qy, double qz, double qw);
Eigen::Matrix3f vec_to_hat(Eigen::Vector3f omega);
pcl::PointCloud<PointType>::Ptr cloud_transform(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Matrix4f mat);
double normalize_angle(double angle);
float dist(PointType p);
Eigen::Matrix3f SkewSymMatrix(Eigen::Vector3f vec);
Eigen::Matrix3d SkewSymMatrixF64(Eigen::Vector3d vec);
Eigen::Matrix3f Exp(Eigen::Vector3f ang);
Eigen::Matrix3d ExpF64(Eigen::Vector3d ang);
Eigen::Vector3f SO3_LOG(Eigen::Matrix3f R);
Eigen::Vector3d SO3_LOGF64(Eigen::Matrix3d R);
Eigen::Vector3f RotMtoEuler(Eigen::Matrix3f rot);
Eigen::Matrix3f RightJacobianRotionMatrix(Eigen::Vector3f omega);
Eigen::Matrix3f InverseRightJacobianRotionMatrix(Eigen::Vector3f omega);

void print_pose(pose_type pose);


#endif