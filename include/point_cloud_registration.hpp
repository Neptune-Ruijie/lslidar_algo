#ifndef POINT_CLOUD_REGISTRATION_HPP
#define POINT_CLOUD_REGISTRATION_HPP

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <euler_angle.hpp>

namespace ship_gauge
{

EulerAngle get_eulers(Eigen::Matrix4f & transformation_matrix)
{
  // rot_mat and translation
  Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f translation_vector = transformation_matrix.block<3, 1>(0, 3);

  // Now we decompose the rotation matrix into Euler angles (Yaw, Pitch, Roll -> ZYX)
  // Eigen's eulerAngles function returns angles in the specified axis order.
  Eigen::Vector3f euler_angles =
    rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order (Yaw, Pitch, Roll)

  // Convert radians to degrees if needed
  euler_angles = euler_angles * (180.0 / M_PI);

  // std::cout << "Rotation Matrix:" << rotation_matrix << std::endl;
  // std::cout << "Translation: " << translation_vector.transpose() << std::endl;
  // Print the Euler angles
  //   std::cout << "Yaw (Z): " << euler_angles[0] << " degrees" << std::endl;
  //   std::cout << "Pitch (Y): " << euler_angles[1] << " degrees" << std::endl;
  //   std::cout << "Roll (X): " << euler_angles[2] << " degrees" << std::endl;
  //   std::cout << "Diff - 180: yaw=" << fabs(euler_angles[0]) - 180.0f
  //             << ", pitch=" << fabs(euler_angles[1]) - 180.0f
  //             << ", roll=" << fabs(euler_angles[2]) - 180.0f << std::endl;
  //   std::cout << "Diff -  0 : yaw=" << fabs(euler_angles[0]) - 0.0f
  //             << ", pitch=" << fabs(euler_angles[1]) - 0.0f
  //             << ", roll=" << fabs(euler_angles[2]) - 0.0f << std::endl;

  EulerAngle euler_output(0.0f, 0.0f, 0.0f);
  euler_output.roll = euler_angles[2];
  euler_output.pitch = euler_angles[1];
  euler_output.yaw = euler_angles[0];
  return euler_output;
}

void icp_to_angle(
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_t_minus_1,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_t)
{
  if (pc_t_minus_1->empty() || pc_t->empty()) {
    std::cerr << "Source or target cloud is empty!" << std::endl;
    return;
  }
  auto pc_aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // 创建ICP对象并设置参数
  auto icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
  icp->setInputSource(pc_t_minus_1);
  icp->setInputTarget(pc_t);
  icp->align(*pc_aligned);

  // std::cout << "----------- pc_t_minus_1 ------------" << std::endl;
  // print_pc(pc_t_minus_1);
  // std::cout << "----------- pc_t ------------" << std::endl;
  // print_pc(pc_t);
  // std::cout << "----------- pc_aligned ------------" << std::endl;
  // print_pc(pc_aligned);

  Eigen::Matrix4f transformation_matrix = icp->getFinalTransformation();
  std::cout << "--- ICP ---" << std::endl;
  EulerAngle euler_output = get_eulers(transformation_matrix);

  // 输出角度
  std::cout << "Diff: yaw=" << fmin(fabs(fabs(euler_output.yaw) - 180.0f), fabs(euler_output.yaw))
            << ", pitch=" << fmin(fabs(fabs(euler_output.roll) - 180.0f), fabs(euler_output.roll))
            << ", roll=" << fmin(fabs(fabs(euler_output.pitch) - 180.0f), fabs(euler_output.pitch))
            << std::endl;

  // 输出配准结果
  // std::cout << "--- ICP has converged: " << icp->hasConverged() << " with score: " << icp->getFitnessScore() << std::endl;
  // std::cout << "Transformation Matrix:" << std::endl;
  // std::cout << transformation_matrix << std::endl;
}

void ndt_to_angle(
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_t_minus_1,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_t)
{
  if (pc_t_minus_1->empty() || pc_t->empty()) {
    std::cerr << "Source or target cloud is empty!" << std::endl;
    return;
  }

  // 创建NDT对象并设置参数
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setInputSource(pc_t_minus_1);
  ndt.setInputTarget(pc_t);

  // 设置配准参数
  ndt.setMaximumIterations(35);  // Maximum number of iterations
//   ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.01);
  ndt.setResolution(0.01);

  pcl::PointCloud<pcl::PointXYZ> Final;
  ndt.align(Final);

  Eigen::Matrix4f transformation_matrix = ndt.getFinalTransformation();
  std::cout << "--- NDT ---" << std::endl;
  std::cout << "--- NDT has converged: " << ndt.hasConverged() << " with score: " << ndt.getFitnessScore() << std::endl;
  EulerAngle euler_output = get_eulers(transformation_matrix);
  // 输出角度
  std::cout << "Diff: yaw="
            << fmin(fabs(fabs(euler_output.pitch) - 180.0f), fabs(euler_output.pitch))
            << ", pitch=" << fmin(fabs(fabs(euler_output.yaw) - 180.0f), fabs(euler_output.yaw))
            << ", roll=" << fmin(fabs(fabs(euler_output.roll) - 180.0f), fabs(euler_output.roll))
            << std::endl;

  // 输出配准结果
  // std::cout << "--- NDT has converged: " << ndt.hasConverged() << " with score: " << ndt.getFitnessScore() << std::endl;
  // std::cout << "Transformation Matrix:" << std::endl;
  // std::cout << transformation_matrix << std::endl;
}

}  // namespace ship_gauge

#endif  // POINT_CLOUD_REGISTRATION_HPP