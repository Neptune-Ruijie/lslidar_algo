# 主节点
add_executable(lslidar_algo_node main.cpp node_ship_gauge.cpp)

message(STATUS "rclcpp_LIBRARIES = ${rclcpp_LIBRARIES}")
#message(STATUS "libpcap_LIBRARIES = ${libpcap_LIBRARIES}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "PCL_COMMON_INCLUDE_DIRS = ${PCL_COMMON_INCLUDE_DIRS}")

target_link_libraries(lslidar_algo_node ${rclcpp_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(lslidar_algo_node rclcpp std_msgs lslidar_msgs sensor_msgs pcl_conversions neso_interfaces)

#install(DIRECTORY launch params rviz_cfg
#		DESTINATION share/${PROJECT_NAME})

install(TARGETS
		lslidar_algo_node
		DESTINATION lib/${PROJECT_NAME}
		)

ament_export_dependencies(rclcpp std_msgs lslidar_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")