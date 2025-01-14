#ifndef FAST_LIO_SAM_SC_QN_POSE_PCD_HPP
#define FAST_LIO_SAM_SC_QN_POSE_PCD_HPP

///// Custom Headers
#include "utilities.hpp"

struct PosePcd
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_;
    int idx_;
    bool processed_ = false;

    PosePcd() {}

    PosePcd(const nav_msgs::msg::Odometry &odom_in,
            const sensor_msgs::msg::PointCloud2 &pcd_in,
            const int &idx_in);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in)
{
    // Convert orientation quaternion to rotation matrix
    tf2::Quaternion q(odom_in.pose.pose.orientation.x,
                      odom_in.pose.pose.orientation.y,
                      odom_in.pose.pose.orientation.z,
                      odom_in.pose.pose.orientation.w);

    // Convert tf2::Quaternion to tf2::Matrix3x3
    tf2::Matrix3x3 rot_mat_tf(q);

    // Manually copy tf2::Matrix3x3 to Eigen::Matrix3d
    Eigen::Matrix3d rot_mat_eig;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rot_mat_eig(i, j) = rot_mat_tf[i][j];
        }
    }

    // Use rot_mat_eig for further processing
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;

    // Convert PointCloud2 to PCL PointCloud and transform it
    pcl::PointCloud<PointType> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse()); // Transform point cloud to local frame

    timestamp_ = rclcpp::Time(odom_in.header.stamp).seconds();
    idx_ = idx_in;
}

#endif
