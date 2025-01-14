#ifndef FAST_LIO_SAM_SC_QN_MAIN_H
#define FAST_LIO_SAM_SC_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>

///// ROS
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/writer.hpp>              // For saving map data
#include <tf2/LinearMath/Quaternion.h>         // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>          // For quaternion to Euler conversions
#include <tf2_eigen/tf2_eigen.h>             // For transformations between tf2 and Eigen
#include <tf2_ros/transform_broadcaster.h>     // For broadcasting transforms
#include "std_msgs/msg/string.hpp"             // For String messages
#include <geometry_msgs/msg/pose_stamped.hpp>  // For PoseStamped messages
#include <message_filters/subscriber.h>        // For message filters
#include <message_filters/time_synchronizer.h> // For time synchronization of messages
#include <message_filters/sync_policies/approximate_time.h> // For approximate time policy
#include <sensor_msgs/msg/point_cloud2.hpp>   // For PointCloud2 messages
#include <nav_msgs/msg/odometry.hpp>           // For Odometry message
#include <nav_msgs/msg/path.hpp>                  // For Path message
#include <visualization_msgs/msg/marker.hpp>      // For Marker message

///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
///// coded headers
#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioSamScQn : public rclcpp::Node
{
private:
    ///// basic params
    std::string map_frame_;
    // std::string odom_frame_;
    std::string package_path_;
    std::string seq_name_;
    ///// shared data - odom and pcd
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    ///// graph and values
    bool is_initialized_ = false;
    bool loop_added_flag_ = false;     // for opt
    bool loop_added_flag_vis_ = false; // for vis
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis
    ///// visualize
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::msg::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    ///// results
    bool save_map_pcd_ = false, save_in_kitti_format_ = false;
    ///// ros
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, corrected_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_, corrected_odom_pub_, corrected_current_pcd_pub_, corrected_pcd_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;
    rclcpp::TimerBase::SharedPtr loop_timer_, vis_timer_;
    
    // odom, pcd sync, and save flag subscribers
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_pcd_;
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_;
    
    ///// Loop closure
    std::shared_ptr<LoopClosure> loop_closure_;

public:
    explicit FastLioSamScQn(const rclcpp::NodeOptions &options);
    ~FastLioSamScQn();
    void initializeSubscribers();
    
private:
    // methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::msg::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    // cb
    
    void odomPcdCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &odom_msg,
                         const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &pcd_msg);
    void saveFlagCallback(const std_msgs::msg::String::SharedPtr msg);
    void loopTimerFunc();
    void visTimerFunc();
};


#endif
