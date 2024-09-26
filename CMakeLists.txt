cmake_minimum_required(VERSION 3.5)
project(lslidar_algo)

set(CMAKE_BUILD_TYPE Release)    #RelWithDebInfo
#add_compile_options(-std=c++11)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(libpcap_LIBRARIES -lpcap)
#set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
#set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake_gtest REQUIRED)
find_package(Boost REQUIRED)
find_package(angles REQUIRED)
find_package(PCL REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(neso_interfaces REQUIRED PATHS ${WS_INSTALL_PREFIX})

# pcl库有很多使用Anonymous Structs的地方
add_compile_options(-Wno-pedantic)

#include_directories(
#		include
#		${PCL_INCLUDE_DIRS}
#		${Boost_INCLUDE_DIRS}
#		${sensor_msgs_INCLUDE_DIRS}
#)

include_directories(
	${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${WS_INSTALL_PREFIX}/include)


#link_directories(/usr/lib/x86_64-linux-gnu)


message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

add_subdirectory(src)
add_subdirectory(test)

ament_package()
