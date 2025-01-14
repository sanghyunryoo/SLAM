#include "fast_lio_sam_sc_qn.h"

FastLioSamScQn::FastLioSamScQn(const rclcpp::NodeOptions &options)
    : Node("fast_lio_sam_sc_qn", options) // Inherit from rclcpp::Node
{
    ////// ROS2 params
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    auto &gc = lc_config.gicp_config_;
    auto &qc = lc_config.quatro_config_;

    // Declare and get parameters
    map_frame_ = declare_parameter<std::string>("basic.map_frame", "map");
    // odom_frame_ = declare_parameter<std::string>("basic.odom_frame", "odom");

    loop_update_hz = declare_parameter<double>("basic.loop_update_hz", 1.0);
    vis_hz = declare_parameter<double>("basic.vis_hz", 0.5);
    voxel_res_ = declare_parameter<double>("save_voxel_resolution", 0.3);
    lc_config.voxel_res_ = declare_parameter<double>("quatro_nano_gicp_voxel_resolution", 0.3);

    keyframe_thr_ = declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
    lc_config.num_submap_keyframes_ = declare_parameter<int>("keyframe.num_submap_keyframes", 5);
    lc_config.enable_submap_matching_ = declare_parameter<bool>("keyframe.enable_submap_matching", false);

    lc_config.scancontext_max_correspondence_distance_ = declare_parameter<double>("scancontext_max_correspondence_distance", 35.0);

    gc.nano_thread_number_ = declare_parameter<int>("nano_gicp.thread_number", 0);
    gc.icp_score_thr_ = declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
    gc.nano_correspondences_number_ = declare_parameter<int>("nano_gicp.correspondences_number", 15);
    gc.max_corr_dist_ = declare_parameter<double>("nano_gicp.max_correspondence_distance", 0.01);
    gc.nano_max_iter_ = declare_parameter<int>("nano_gicp.max_iter", 32);
    gc.transformation_epsilon_ = declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
    gc.euclidean_fitness_epsilon_ = declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
    gc.nano_ransac_max_iter_ = declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
    gc.ransac_outlier_rejection_threshold_ = declare_parameter<double>("nano_gicp.ransac.outlier_rejection_threshold", 1.0);
    
    lc_config.enable_quatro_ = declare_parameter<bool>("quatro.enable", false);
    qc.use_optimized_matching_ = declare_parameter<bool>("quatro.optimize_matching", true);
    qc.quatro_distance_threshold_ = declare_parameter<double>("quatro.distance_threshold", 30.0);
    qc.quatro_max_num_corres_ = declare_parameter<int>("quatro.max_correspondences", 200);
    qc.fpfh_normal_radius_ = declare_parameter<double>("quatro.fpfh_normal_radius", 0.3);
    qc.fpfh_radius_ = declare_parameter<double>("quatro.fpfh_radius", 0.5);
    qc.estimat_scale_ = declare_parameter<bool>("quatro.estimating_scale", false);
    qc.noise_bound_ = declare_parameter<double>("quatro.noise_bound", 0.3);
    qc.rot_gnc_factor_ = declare_parameter<double>("quatro.rotation.gnc_factor", 1.4);
    qc.rot_cost_diff_thr_ = declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold", 0.0001);
    qc.quatro_max_iter_ = declare_parameter<int>("quatro.rotation.num_max_iter", 50);

    save_map_pcd_ = declare_parameter<bool>("result.save_map_pcd", false);
    save_in_kitti_format_ = declare_parameter<bool>("result.save_in_kitti_format", false);
    seq_name_ = declare_parameter<std::string>("result.seq_name", "");

    loop_closure_ = std::make_shared<LoopClosure>(lc_config);
    // GTSAM initialization
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);

    // Initialize paths
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    // Publishers
    odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ori_odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/ori_path", 10);
    corrected_odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_odom", 10);
    corrected_path_pub_ = create_publisher<nav_msgs::msg::Path>("/corrected_path", 10);
    corrected_pcd_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map", 10);
    corrected_current_pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_current_pcd", 10);
    loop_detection_pub_ = create_publisher<visualization_msgs::msg::Marker>("/loop_detection", 10);
    realtime_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", 10);
    debug_src_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/src", 10);
    debug_dst_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dst", 10);
    debug_coarse_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/fine_aligned_nano_gicp", 10);

    sub_save_flag_ = create_subscription<std_msgs::msg::String>(
        "/save_dir", 1,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            saveFlagCallback(msg);
        });
    
    // TF
    // map_to_odom_.setIdentity();

    // Timers
    loop_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / loop_update_hz), std::bind(&FastLioSamScQn::loopTimerFunc, this));
    vis_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz), std::bind(&FastLioSamScQn::visTimerFunc, this));
    
    RCLCPP_INFO(get_logger(), "Main class, starting node...");
}   

void FastLioSamScQn::initializeSubscribers()
{
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(shared_from_this(), "/Odometry", rclcpp::QoS(10).get_rmw_qos_profile());
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(shared_from_this(), "/cloud_registered", rclcpp::QoS(10).get_rmw_qos_profile());
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    
    sub_odom_pcd_sync_->registerCallback(
    std::bind(&FastLioSamScQn::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));
   // broadcaster_도 동일
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
}

void FastLioSamScQn::odomPcdCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &odom_msg,
                                     const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &pcd_msg)
{   
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;                              // to calculate delta
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    auto t1 = std::chrono::high_resolution_clock::now();
    {
        //// 1. Realtime pose = last corrected odom * delta (last -> current)
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        
        // Update odom delta and corrected pose
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;

        // Publish map -> odom transform
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = this->get_clock()->now();
        map_to_odom.header.frame_id = map_frame_; // Parent frame
        map_to_odom.child_frame_id = "odom";     // Child frame

        // Calculate map -> odom using corrected pose and current odom delta
        Eigen::Matrix4d map_to_odom_mat = current_frame_.pose_corrected_eig_ * current_frame_.pose_eig_.inverse();
        map_to_odom.transform = poseEigToROSTf(map_to_odom_mat);
        broadcaster_->sendTransform(map_to_odom);

        // Publish odom -> base_footprint transform
        // geometry_msgs::msg::TransformStamped odom_to_base_footprint;
        // odom_to_base_footprint.header.stamp = this->get_clock()->now();
        // odom_to_base_footprint.header.frame_id = "odom";       // Parent frame
        // odom_to_base_footprint.child_frame_id = "base_footprint"; // Child frame

        // // odom -> base_footprint is derived from the current pose in odometry frame
        // Eigen::Matrix4d odom_to_base_footprint_mat = current_frame_.pose_eig_;
        // odom_to_base_footprint.transform = poseEigToROSTf(odom_to_base_footprint_mat);
        // broadcaster_->sendTransform(odom_to_base_footprint);

        // Publish the realtime pose as a PoseStamped message
        realtime_pose_pub_->publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));

    }
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {   
        // others
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        
        // graph
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        // ScanContext
        loop_closure_->updateScancontext(current_frame_.pcd_);
        is_initialized_ = true;
    }
    else
    {
        //// 2. check if keyframe
        auto t2 = std::chrono::high_resolution_clock::now();
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
            // 2-3. if so, add to graph
            auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
            }
            current_keyframe_idx_++;
            // 2-4. if so, update ScanContext
            loop_closure_->updateScancontext(current_frame_.pcd_);

            //// 3. vis
            auto t3 = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }
            
            //// 4. optimize with graph
            auto t4 = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                isam_handler_->update(gtsam_graph_, init_esti_);
                isam_handler_->update();
                if (loop_added_flag_)
                {
                    isam_handler_->update();
                    isam_handler_->update();
                    isam_handler_->update();
                }
                gtsam_graph_.resize(0);
                init_esti_.clear();
            }

            //// 5. handle corrected results
            auto t5 = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                corrected_esti_ = isam_handler_->calculateEstimate();
                last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            // correct poses in keyframes
            if (loop_added_flag_)
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i)
                {
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
            }
            auto t6 = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms",
                     std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3,
                     std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1e3,
                     std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() / 1e3,
                     std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() / 1e3,
                     std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() / 1e3,
                     std::chrono::duration_cast<std::chrono::microseconds>(t6 - t1).count() / 1e3);
        }
    }
    return;
}

void FastLioSamScQn::loopTimerFunc()
{
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_)
    {
        return;
    }
    latest_keyframe.processed_ = true;

    auto t1 = std::chrono::high_resolution_clock::now();
    const int closest_keyframe_idx = loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_);
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("FastLioSamScQn"), "Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    debug_src_pub_->publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    debug_coarse_aligned_pub_->publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

    RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "loop: %.1f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3);
    return;
}

void FastLioSamScQn::visTimerFunc()
{
    if (!is_initialized_)
    {
        return;
    }

    auto tv1 = std::chrono::high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(poseEigToPoseStamped(pose_.matrix(), map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {   
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
        path_pub_->publish(odom_path_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_->publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() > 0) // save time, only once
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() == 0)
    {
        global_map_vis_switch_ = true;
    }
    auto tv2 = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "vis: %.1fms", std::chrono::duration_cast<std::chrono::microseconds>(tv2 - tv1).count() / 1e3);
    return;
}

void FastLioSamScQn::saveFlagCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // Save scans as individual PCD files and poses in KITTI format
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {   
                // Save the point cloud
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);
                
                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_
                              << " " << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
        RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    }
}

FastLioSamScQn::~FastLioSamScQn()
{
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
        RCLCPP_INFO(rclcpp::get_logger("FastLioSamScQn"), "\033[32;1mResult saved in .pcd format!!!\033[0m");
    }
}

void FastLioSamScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::msg::Marker FastLioSamScQn::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::msg::Marker edges;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::msg::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

bool FastLioSamScQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}
