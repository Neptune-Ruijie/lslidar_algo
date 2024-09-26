#ifndef NODE_SHIP_GAUGE_HPP
#define NODE_SHIP_GAUGE_HPP

#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <neso_interfaces/srv/point_cloud_save.hpp>
#include <memory>
#include <string>
#include <Eigen/Dense>

namespace ship_gauge
{
using namespace std::chrono_literals;
using SrvPointCloudSave = neso_interfaces::srv::PointCloudSave;

class NodeShipGauge : public rclcpp::Node
{
public:
  explicit NodeShipGauge(const std::string &name);

private:
  std::string get_now_string();

  void point_cloud_save_service(
    const std::shared_ptr<SrvPointCloudSave::Request> req,
    std::shared_ptr<SrvPointCloudSave::Response> resp);

  void point_cloud_message_handle(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void print_pc(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

  int load_ply();

  void update_ship_info();

  void get_eulers(Eigen::Matrix4f &transformation_matrix);

  // Class member variables
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_reduced_t_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_reduced_t_minus_1_;
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxel_grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
  rclcpp::Service<SrvPointCloudSave>::SharedPtr point_cloud_save_service_;
  rclcpp::TimerBase::SharedPtr schedule_timer_;
};

}  // namespace ship_gauge

#endif  // NODE_SHIP_GAUGE_HPP
