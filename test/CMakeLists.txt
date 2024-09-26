# gtest
## 设定可执行文件名称
file(GLOB CPP_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/ *.cpp)
message(RUIJIE "CPP_FILES = ${CPP_FILES}")

foreach(path_cpp IN LISTS CPP_FILES)
   message(RUIJIE "Filename without extension: ${path_cpp}")
   string(SUBSTRING ${path_cpp} 0 5 prefix)
   if(prefix STREQUAL "gtest")
      get_filename_component(name ${path_cpp} NAME_WE)
      message(RUIJIE "Filename without extension: ${name}")
      ament_add_gtest(${name} ${name}.cpp)
      target_include_directories(${name} PRIVATE ${LOCAL_SRCS_DIR} ${EIGEN3_INCLUDE_DIR})
      ament_target_dependencies(${name} ${dependencies} geometry_msgs sensor_msgs)
      target_link_directories(${name} PRIVATE ${YAML_CPP_LIBRARY_DIR})
      target_link_libraries(${name} lslidar_algo_node_core ${rclcpp_LIBRARIES} fmt::fmt  yaml-cpp)
   endif()
endforeach()


# 非gtest
ament_add_gtest(test_cloud_msg test_cloud_msg.cpp)
target_include_directories(test_cloud_msg PRIVATE ${LOCAL_SRCS_DIR} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(test_cloud_msg geometry_msgs sensor_msgs)
target_link_directories(test_cloud_msg PRIVATE ${YAML_CPP_LIBRARY_DIR})
#target_link_libraries(test_cloud_msg ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${libpcap_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(test_cloud_msg ${rclcpp_LIBRARIES})