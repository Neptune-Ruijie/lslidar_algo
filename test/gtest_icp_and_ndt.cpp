#include <gtest/gtest.h>

#include <iostream>
// #include <node_ship_gauge.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <euler_angle.hpp>
#include <point_cloud_registration.hpp>

using namespace ship_gauge;

void load_ply(std::string & file, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & pc)
{
  // Load the point cloud from the .ply file
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(file, *pc) == -1) {
    std::cout << "file = " << file << " cannot be loaded." << std::endl;
  } else {
    std::cout << "file = " << file << " loaded with size =" << pc->size() << std::endl;
  }
}

void euler_compare(EulerAngle & euler_t_minus_1, EulerAngle & euler_t)
{
  std::cout << "Diff: yaw=" << fabs(euler_t.yaw - euler_t_minus_1.yaw)
            << ", pitch=" << fabs(euler_t.pitch - euler_t_minus_1.pitch)
            << ", roll=" << fabs(euler_t.roll - euler_t_minus_1.roll) << std::endl;
}

TEST(TestICP, icp)
{
  // int argc = 0;
  // char** argv = nullptr;
  // rclcpp::init(argc, argv);
  // auto node_ship_gauge = std::make_shared<ship_gauge::NodeShipGauge>("test");
  // rclcpp::spin(node_ship_gauge);

  auto pc_0 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_3 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_4 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_5 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_6 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pc_7 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::string file_0 = "/home/neptune/data_20240919/cloud_20240919205222.ply";
  std::string file_1 = "/home/neptune/data_20240919/cloud_20240919205316.ply";
  std::string file_2 = "/home/neptune/data_20240919/cloud_20240919205343.ply";
  std::string file_3 = "/home/neptune/data_20240919/cloud_20240919205410.ply";
  std::string file_4 = "/home/neptune/data_20240919/cloud_20240919205428.ply";
  std::string file_5 = "/home/neptune/data_20240919/cloud_20240919205449.ply";
  std::string file_6 = "/home/neptune/data_20240919/cloud_20240919205507.ply";
  std::string file_7 = "/home/neptune/data_20240919/cloud_20240919205522.ply";

  auto euler_0 = EulerAngle(0.0163536f, 0.999025f, -63.687872f);
  auto euler_1 = EulerAngle(-0.189497f, 0.927004, -76.532563f);
  auto euler_2 = EulerAngle(-0.299192f, 0.899492f, -92.777361f);
  auto euler_3 = EulerAngle(-6.258114f, 2.675336f, -92.191372f);
  auto euler_4 = EulerAngle(-18.731876f, 1.337463f, -91.214511f);
  auto euler_5 = EulerAngle(0.100768f, 1.041842f, -35.91094f);
  auto euler_6 = EulerAngle(-0.38524f, -9.335945f, -37.541287f);
  auto euler_7 = EulerAngle(0.063138f, 0.916253f, -43.596009f);

  load_ply(file_0, pc_0);
  load_ply(file_1, pc_1);
  load_ply(file_2, pc_2);
  load_ply(file_3, pc_3);
  load_ply(file_4, pc_4);
  load_ply(file_5, pc_5);
  load_ply(file_6, pc_6);
  load_ply(file_7, pc_7);

  //   EXPECT_EQ(ais_data_handle_->is_align_with_fog(0.0f), false);
  ship_gauge::icp_to_angle(pc_0, pc_1);
    ship_gauge::ndt_to_angle(pc_0, pc_1);
  std::cout << "--- FOG ---" << std::endl;
  euler_compare(euler_0, euler_1);

//   ship_gauge::icp_to_angle(pc_1, pc_2);
//   //   ship_gauge::ndt_to_angle(pc_1, pc_2);
//   std::cout << "--- FOG ---" << std::endl;
//   euler_compare(euler_1, euler_2);

  ship_gauge::icp_to_angle(pc_2, pc_3);
    ship_gauge::ndt_to_angle(pc_2, pc_3);
  std::cout << "--- FOG ---" << std::endl;
  euler_compare(euler_2, euler_3);

//   ship_gauge::icp_to_angle(pc_3, pc_4);
//   //   ship_gauge::ndt_to_angle(pc_3, pc_4);
//   std::cout << "--- FOG ---" << std::endl;
//   euler_compare(euler_3, euler_4);

//   ship_gauge::icp_to_angle(pc_4, pc_5);
//   //   ship_gauge::ndt_to_angle(pc_4, pc_5);
//   std::cout << "--- FOG ---" << std::endl;
//   euler_compare(euler_4, euler_5);

//   ship_gauge::icp_to_angle(pc_5, pc_6);
//   //   ship_gauge::ndt_to_angle(pc_5, pc_6);
//   std::cout << "--- FOG ---" << std::endl;
//   euler_compare(euler_5, euler_6);

//   ship_gauge::icp_to_angle(pc_6, pc_7);
//   //   ship_gauge::ndt_to_angle(pc_6, pc_7);
//   std::cout << "--- FOG ---" << std::endl;
//   euler_compare(euler_6, euler_7);

  //   rclcpp::shutdown();  // Shutdown ROS 2
}
