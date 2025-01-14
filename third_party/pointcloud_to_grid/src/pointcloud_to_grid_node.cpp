// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// Custom
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
// C++
#include <vector>
#include <memory>
#include <string>
#include <algorithm>

auto height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
class PointCloudToGrid : public rclcpp::Node
{
public:
  PointCloudToGrid() : Node("pointcloud_to_grid_node"), count_(0)
  {
    this->declare_parameter<std::string>("cloud_in_topic", "nonground");
    this->declare_parameter<std::string>("odom_lio_topic", "/Odometry");
    this->declare_parameter<std::string>("map_topic", "height_grid");
    this->declare_parameter<double>("filter_min_z", -1.5);
    this->declare_parameter<double>("filter_max_z", 2.0);
    this->declare_parameter<double>("occupied_threshold", 2.0);

    this->get_parameter("cloud_in_topic", cloud_in_topic_);
    this->get_parameter("odom_lio_topic", odom_lio_topic_);
    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("filter_min_z", filter_min_z_);
    this->get_parameter("filter_max_z", filter_max_z_);
    this->get_parameter("occupied_threshold", occupied_threshold_);


    pub_hgrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
    sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_in_topic_, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cloud_in_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", odom_lio_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing map to %s", map_topic_.c_str());

    // Initialize height grid
    height_grid->info.resolution = 0.05; // Grid cell size
    height_grid->info.origin.position.x = 0.0;
    height_grid->info.origin.position.y = 0.0;
    height_grid->info.origin.position.z = 0.0;
    height_grid->info.origin.orientation.w = 1.0;
    height_grid->info.width = 200;
    height_grid->info.height = 200;
    height_grid->data.resize(height_grid->info.width * height_grid->info.height, -1);
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*input_msg, *cloud);

      // Apply PassThrough filter to remove ground points and high points
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(filter_min_z_, filter_max_z_);
      pass.filter(*cloud_filtered);

      // Update Occupancy Grid boundaries
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();

      for (const auto &p : cloud_filtered->points)
      {
          min_x = std::min(min_x, static_cast<double>(p.x));
          max_x = std::max(max_x, static_cast<double>(p.x));
          min_y = std::min(min_y, static_cast<double>(p.y));
          max_y = std::max(max_y, static_cast<double>(p.y));
      }

      // Calculate new origin and size if the grid needs to be expanded
      double origin_x = std::min(height_grid->info.origin.position.x, min_x);
      double origin_y = std::min(height_grid->info.origin.position.y, min_y);
      double end_x = std::max(height_grid->info.origin.position.x + height_grid->info.width * height_grid->info.resolution, max_x);
      double end_y = std::max(height_grid->info.origin.position.y + height_grid->info.height * height_grid->info.resolution, max_y);

      uint32_t new_width = static_cast<uint32_t>(std::ceil((end_x - origin_x) / height_grid->info.resolution));
      uint32_t new_height = static_cast<uint32_t>(std::ceil((end_y - origin_y) / height_grid->info.resolution));

      // Create a new data grid and copy existing data
      std::vector<signed char> new_data(new_width * new_height, -1);

      for (uint32_t y = 0; y < height_grid->info.height; ++y)
      {
          for (uint32_t x = 0; x < height_grid->info.width; ++x)
          {
              double global_x = height_grid->info.origin.position.x + x * height_grid->info.resolution;
              double global_y = height_grid->info.origin.position.y + y * height_grid->info.resolution;

              int new_x = static_cast<int>((global_x - origin_x) / height_grid->info.resolution);
              int new_y = static_cast<int>((global_y - origin_y) / height_grid->info.resolution);

              if (new_x >= 0 && new_x < static_cast<int>(new_width) &&
                  new_y >= 0 && new_y < static_cast<int>(new_height))
              {
                  int old_index = y * height_grid->info.width + x;
                  int new_index = new_y * new_width + new_x;
                  if (new_data[new_index] == -1)
                      new_data[new_index] = height_grid->data[old_index];
                  else if (height_grid->data[old_index] == 100)
                      new_data[new_index] = 100; // Maintain occupied state if already marked
              }
          }
      }

      // Update grid properties
      height_grid->info.origin.position.x = origin_x;
      height_grid->info.origin.position.y = origin_y;
      height_grid->info.width = new_width;
      height_grid->info.height = new_height;
      height_grid->data = std::move(new_data);

      // Populate the grid with new point cloud data
      for (const auto &p : cloud_filtered->points)
      {
          int cell_x = static_cast<int>((p.x - height_grid->info.origin.position.x) / height_grid->info.resolution);
          int cell_y = static_cast<int>((p.y - height_grid->info.origin.position.y) / height_grid->info.resolution);

          if (cell_x >= 0 && cell_x < static_cast<int>(height_grid->info.width) &&
              cell_y >= 0 && cell_y < static_cast<int>(height_grid->info.height))
          {
              // Assign value based on occupancy state
              int index = cell_y * height_grid->info.width + cell_x;
              if (p.z > occupied_threshold_ * 0.8)
                  height_grid->data[index] = 100; // Occupied
              else if (height_grid->data[index] != 100)
                  height_grid->data[index] = 0; // Free if not already occupied
          }
      }
      // Publish the updated map
      height_grid->header.stamp = this->now();
      height_grid->header.frame_id = "map";
      pub_hgrid_->publish(*height_grid);
  }

  void odom_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg)
  {
    // Update the grid origin based on odometry data
    RCLCPP_INFO(this->get_logger(), "Received odometry: (%.2f, %.2f)",
                msg->position.x, msg->position.y);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_hgrid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;

  std::string cloud_in_topic_;
  std::string odom_lio_topic_;
  std::string map_topic_;
  double filter_min_z_;
  double filter_max_z_;
  double occupied_threshold_;
  size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
  rclcpp::shutdown();
  return 0;
}
