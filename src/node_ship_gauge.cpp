#include <pcl/point_cloud.h>

#include <memory>
#include <node_ship_gauge.hpp>
#include <ostream>
#include <pcl/impl/point_types.hpp>
#include <point_cloud_registration.hpp>

namespace ship_gauge
{

// 构造函数
NodeShipGauge::NodeShipGauge(std::string const & name) : rclcpp::Node(name)
{
  pc_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pc_reduced_t_minus_1_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pc_reduced_t_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel_grid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
  voxel_grid_->setLeafSize(0.05f, 0.05f, 0.05f);  // 62536 => 10601
  // voxel_grid_->setLeafSize(0.1f, 0.1f, 0.1f); // 62536 => 172
  std::cout << "node start" << std::endl;

  // 订阅点云
  point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cx/lslidar_point_cloud", 10,
    std::bind(&NodeShipGauge::point_cloud_message_handle, this, std::placeholders::_1));

  // 坐标转换服务
  point_cloud_save_service_ = this->create_service<SrvPointCloudSave>(
    "algo/point_cloud_save", std::bind(
                               &NodeShipGauge::point_cloud_save_service, this,
                               std::placeholders::_1, std::placeholders::_2));

  // 创建一个定时器
  schedule_timer_ =
    this->create_wall_timer(5000ms, std::bind(&NodeShipGauge::update_ship_info, this));
}

std::string NodeShipGauge::get_now_string()
{
  // Get current time from system clock
  auto now = std::chrono::system_clock::now();

  // Convert to time_t for easier formatting
  std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);

  // Convert time_t to a string with formatting
  std::tm * tm_now = std::localtime(&time_t_now);
  std::ostringstream ss;
  ss << std::put_time(tm_now, "%Y%m%d%H%M%S");

  return ss.str();
}

void NodeShipGauge::point_cloud_save_service(
  const std::shared_ptr<SrvPointCloudSave::Request> req,
  std::shared_ptr<SrvPointCloudSave::Response> resp)
{
  // 存储
  std::ostringstream ss;
  if (req->action == SrvPointCloudSave::Request::ACTION_SAVE_RAW_POINT_CLOUD) {
    ss << "/home/neptune/data/cloud_" << get_now_string() << ".ply";
    pcl::io::savePLYFileBinary(ss.str(), *pc_);
  } else if (req->action == SrvPointCloudSave::Request::ACTION_SAVE_REDUCED_POINT_CLOUD) {
    ss << "/home/neptune/data/cloud_reduced_" << get_now_string() << ".ply";
    pcl::io::savePLYFileBinary(ss.str(), *pc_reduced_t_);
  }
}

void NodeShipGauge::point_cloud_message_handle(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // std::cout << "pcl_cloud received0." << std::endl;
  pcl::fromROSMsg(*msg, *pc_);
  // std::cout << "origin: Number of points: " << pc_->size() << std::endl;

  // down_sample
  voxel_grid_->setInputCloud(pc_);
  voxel_grid_->filter(*pc_reduced_t_);
  // std::cout << "down sample: Number of points: " << pc_reduced_t_->size() << std::endl;
}

void NodeShipGauge::print_pc(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc)
{
  for (size_t i = 0; i < pc->points.size(); ++i) {
    std::cout << "Point " << i + 1 << ": "
              << "x = " << pc->points[i].x << ", "
              << "y = " << pc->points[i].y << ", "
              << "z = " << pc->points[i].z << std::endl;
  }
}

int NodeShipGauge::load_ply()
{
  auto pc_loaded = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Load the point cloud from the .ply file
  if (pcl::io::loadPLYFile<pcl::PointXYZ>("cloud.ply", *pc_loaded) == -1) {
    PCL_ERROR("Couldn't read file cloud.ply\n");
    return (-1);
  }
  std::cout << "Loaded point cloud with " << pc_loaded->points.size() << " points." << std::endl;

  return 0;
}

void NodeShipGauge::update_ship_info()
{
  if (pc_reduced_t_minus_1_->size() == 0) {
    // pc_reduced_t_minus_1_ = pc_reduced_t_;
    pcl::copyPointCloud(*pc_reduced_t_, *pc_reduced_t_minus_1_);
    std::cout << "pc_reduced_t_minus_1_ size = " << pc_reduced_t_minus_1_->size() << std::endl;
    return;
  } else {
    std::cout << "pc_reduced_t_minus_1_ size = " << pc_reduced_t_minus_1_->size() << std::endl;
  }

  icp_to_angle(pc_reduced_t_minus_1_, pc_reduced_t_);
  ndt_to_angle(pc_reduced_t_minus_1_, pc_reduced_t_);
  // pc_reduced_t_minus_1_ = pc_reduced_t_;
  pcl::copyPointCloud(*pc_reduced_t_, *pc_reduced_t_minus_1_);
}

}  // namespace ship_gauge