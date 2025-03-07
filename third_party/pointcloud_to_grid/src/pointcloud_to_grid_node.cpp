// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// C++
#include <vector>
#include <memory>
#include <string>
#include <algorithm>

auto global_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

class PointCloudToGrid : public rclcpp::Node
{
public:
    PointCloudToGrid() : Node("pointcloud_to_grid_node"), count_(0)
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("cloud_in_topic", "/corrected_current_pcd");
        this->declare_parameter<std::string>("map_topic", "/map");

        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("local_width", 10.0);
        this->declare_parameter<double>("local_height", 10.0);

        this->declare_parameter<double>("filter_min_z", -1.5);
        this->declare_parameter<double>("filter_max_z", 2.0);
        this->declare_parameter<double>("occupied_threshold", 2.0);

        // 파라미터 로드
        this->get_parameter("cloud_in_topic", cloud_in_topic_);
        this->get_parameter("map_topic", map_topic_);
        
        this->get_parameter("resolution", resolution_);
        this->get_parameter("local_width", local_width_);
        this->get_parameter("local_height", local_height_);

        this->get_parameter("filter_min_z", filter_min_z_);
        this->get_parameter("filter_max_z", filter_max_z_);
        this->get_parameter("occupied_threshold", occupied_threshold_);

        pub_hgrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic_, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cloud_in_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing map to %s", map_topic_.c_str());

        // ✅ 글로벌 맵 초기 크기를 로컬 맵 크기로 설정
        initializeGlobalMap();
    }

private:
    void initializeGlobalMap()
    {
        global_grid_->info.resolution = resolution_;
        global_grid_->info.width = static_cast<uint32_t>(local_width_ / resolution_);
        global_grid_->info.height = static_cast<uint32_t>(local_height_ / resolution_);
        global_grid_->info.origin.position.x = -local_width_ / 2.0;
        global_grid_->info.origin.position.y = -local_height_ / 2.0;
        global_grid_->data.resize(global_grid_->info.width * global_grid_->info.height, -1);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_msg, *cloud);

        // ✅ PassThrough 필터 적용 (Ground 및 너무 높은 포인트 제거)
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_min_z_, filter_max_z_);
        pass.filter(*cloud_filtered);

        // ✅ 로컬 맵 업데이트 (로봇 중심 기준)
        updateLocalMap(cloud_filtered);

        // ✅ 글로벌 맵에 로컬 맵 반영
        mergeLocalToGlobal();

        // ✅ 맵 publish
        global_grid_->header.stamp = this->now();
        global_grid_->header.frame_id = "map";
        pub_hgrid_->publish(*global_grid_);
    }

    void updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered)
    {
        int local_width_cells = static_cast<int>(local_width_ / resolution_);
        int local_height_cells = static_cast<int>(local_height_ / resolution_);

        local_grid_.resize(local_width_cells * local_height_cells, -1); // ✅ 로컬 맵 초기화

        for (const auto& p : cloud_filtered->points)
        {
            int local_x = static_cast<int>((p.x + local_width_ / 2.0) / resolution_);
            int local_y = static_cast<int>((p.y + local_height_ / 2.0) / resolution_);

            if (local_x >= 0 && local_x < local_width_cells && local_y >= 0 && local_y < local_height_cells)
            {
                int index = local_y * local_width_cells + local_x;
                if (p.z > occupied_threshold_ * 0.8)
                    local_grid_[index] = 100; // Occupied
                else if (local_grid_[index] != 100)
                    local_grid_[index] = 0; // Free if not already occupied
            }
        }
    }

    void mergeLocalToGlobal()
    {
        for (size_t i = 0; i < local_grid_.size(); ++i)
        {
            if (local_grid_[i] != -1)
                global_grid_->data[i] = local_grid_[i];
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_hgrid_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;

    std::string cloud_in_topic_;
    std::string map_topic_;
    double resolution_;
    double local_width_;
    double local_height_;
    double filter_min_z_;
    double filter_max_z_;
    double occupied_threshold_;

    std::vector<signed char> local_grid_; // ✅ 로컬 맵 저장
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
