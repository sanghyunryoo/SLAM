#ifndef FAST_LIO_SAM_SC_QN_UTILITIES_HPP
#define FAST_LIO_SAM_SC_QN_UTILITIES_HPP

///// Common Headers
#include <string>

///// ROS 2 Headers
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

///// PCL Headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

///// Eigen Headers
#include <Eigen/Eigen>

///// GTSAM Headers
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using PointType = pcl::PointXYZI;

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

//////////////////////////////////////////////////////////////////////
///// Conversions
inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
    return gtsam::Pose3(
        gtsam::Rot3(pose_eig_in.block<3, 3>(0, 0)),
        gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
    pose_eig_out.block<3, 3>(0, 0) = gtsam_pose_in.rotation().matrix();
    pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
    pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
    pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
    return pose_eig_out;
}

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                            const std::string &frame_id = "map")
{   
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    Eigen::Quaterniond quat(pose_eig_in.block<3, 3>(0, 0));
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    return pose;
}

inline geometry_msgs::msg::Transform poseEigToROSTf(const Eigen::Matrix4d &pose)
{
    // Extract rotation as a quaternion
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    
    // Create Transform
    geometry_msgs::msg::Transform transform_msg;
    transform_msg.translation.x = pose(0, 3);
    transform_msg.translation.y = pose(1, 3);
    transform_msg.translation.z = pose(2, 3);
    transform_msg.rotation.x = quat.x();
    transform_msg.rotation.y = quat.y();
    transform_msg.rotation.z = quat.z();
    transform_msg.rotation.w = quat.w();

    return transform_msg;
}









template<typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(const pcl::PointCloud<T> &cloud,
                                                 const std::string &frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
}

///// Transformation
template<typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.empty())
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}

#endif
