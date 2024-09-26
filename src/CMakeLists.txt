# 主节点
## 主节点目标
set(NODE_CONTROL lslidar_algo_node)
set(library_name ${NODE_CONTROL}_core)


set(dependencies 
    rclcpp
    std_msgs 
	lslidar_msgs 
	sensor_msgs 
	pcl_conversions 
	neso_interfaces)

set(libraries 
    fmt::fmt 
    yaml-cpp
	${rclcpp_LIBRARIES} 
	${Boost_LIBRARIES}
	)

message(STATUS "rclcpp_LIBRARIES = ${rclcpp_LIBRARIES}")
#message(STATUS "libpcap_LIBRARIES = ${libpcap_LIBRARIES}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "PCL_COMMON_INCLUDE_DIRS = ${PCL_COMMON_INCLUDE_DIRS}")


add_library(${library_name} node_ship_gauge.cpp)
target_include_directories(${library_name} PRIVATE ${LOCAL_SRCS_DIR} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${library_name} ${dependencies} )
target_link_directories(${library_name} PRIVATE ${YAML_CPP_LIBRARY_DIR})
target_link_libraries(${library_name} ${libraries})

add_executable(${NODE_CONTROL} main.cpp)
target_include_directories(${NODE_CONTROL} PRIVATE ${LOCAL_SRCS_DIR} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${NODE_CONTROL} ${dependencies} )
target_link_directories(${NODE_CONTROL} PRIVATE ${YAML_CPP_LIBRARY_DIR})
target_link_libraries(${NODE_CONTROL} ${library_name})

## set install 
install(TARGETS ${NODE_CONTROL} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



#add_executable(lslidar_algo_node main.cpp node_ship_gauge.cpp)
#target_link_libraries(lslidar_algo_node ${rclcpp_LIBRARIES} ${Boost_LIBRARIES})
#ament_target_dependencies(lslidar_algo_node )

#install(TARGETS
#		lslidar_algo_node
#		DESTINATION lib/${PROJECT_NAME}
#		)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

#add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")