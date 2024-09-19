# gtest
file(GLOB CPP_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/ *.cpp)

message(STATUS "CPP_FILES = ${CPP_FILES}")

ament_add_gtest(test_cloud_msg test_cloud_msg.cpp)
target_include_directories(test_cloud_msg PRIVATE ${LOCAL_SRCS_DIR} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(test_cloud_msg geometry_msgs sensor_msgs)
target_link_directories(test_cloud_msg PRIVATE ${YAML_CPP_LIBRARY_DIR})
#target_link_libraries(test_cloud_msg ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${libpcap_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(test_cloud_msg ${rclcpp_LIBRARIES})
