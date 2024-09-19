#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>


// inline void PublishMessage<sensor_msgs::msg::PointCloud2>::publishMsg()
// {
//   bool use_raw = false;
//   getInput("use_raw", use_raw);
//   std::string frame_id;
//   auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//   if (use_raw) {
//     getInput("bumper_frame", frame_id);
//     cloud->push_back(pcl::PointXYZ(0.02, -0.05, 0.0));
//     cloud->push_back(pcl::PointXYZ(0.02, 0.05, 0.0));
//   } else {
//     geometry_msgs::msg::PoseStamped current_pose, fill_start, fill_end;
//     std::string global_frame;
//     double transform_tolerance = 0.1;
//     auto tf = config().blackboard->get<tf2_ros::Buffer::SharedPtr>("tf_buffer");
//     getInput("global_frame", global_frame);
//     getInput("robot_base_frame", frame_id);
//     node_->get_parameter_or("transform_tolerance", transform_tolerance, 0.1);
//     nav2_util::getCurrentPose(current_pose, *tf, global_frame, frame_id, transform_tolerance);

//     //正前方
//     geometry_msgs::msg::TransformStamped fill_start_pose, fill_end_pose;
//     fill_start_pose.transform.translation.x = 0.512;
//     fill_start_pose.transform.translation.y = -0.05;
//     fill_start_pose.transform.translation.z = 0.0;
//     fill_start_pose.transform.rotation.x = 0.0;
//     fill_start_pose.transform.rotation.y = 0.0;
//     fill_start_pose.transform.rotation.z = 0.0;
//     fill_start_pose.transform.rotation.w = 1.0;
//     fill_end_pose.transform.translation.x = 0.512;
//     fill_end_pose.transform.translation.y = 0.05;
//     fill_end_pose.transform.translation.z = 0.0;
//     fill_end_pose.transform.rotation.x = 0.0;
//     fill_end_pose.transform.rotation.y = 0.0;
//     fill_end_pose.transform.rotation.z = 0.0;
//     fill_end_pose.transform.rotation.w = 1.0;

//     tf2::doTransform(current_pose, fill_start, fill_start_pose);
//     tf2::doTransform(current_pose, fill_end, fill_end_pose);

//     cloud->push_back(pcl::PointXYZ(
//       fill_start.pose.position.x, fill_start.pose.position.y, fill_start.pose.position.z));
//     cloud->push_back(
//       pcl::PointXYZ(fill_end.pose.position.x, fill_end.pose.position.y, fill_end.pose.position.z));
//   }

//   pcl::toROSMsg(*cloud, *msg);
//   msg->header.frame_id = frame_id;
//   msg->header.stamp = node_->get_clock()->now();
//   pub_message_->publish(std::move(msg));
// }


int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto& point : cloud.points)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0);
        point.y = 1024 * rand() / (RAND_MAX + 1.0);
        point.z = 1024 * rand() / (RAND_MAX + 1.0);
    }

    std::cout << "PCL Version: " << PCL_VERSION << std::endl;


    std::cout << "Point cloud created with " << cloud.points.size() << " points." << std::endl;

    return 0;
}
