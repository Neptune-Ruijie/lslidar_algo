
## 本文件规则用来设定工作空间的环境变量(WS_XXX)
## 生成以下变量
## 工作区相关： WS_ROOT_DIR WS_ROS WS_INSTALL_PREFIX
## ROS相关：ROS_ROOT_DIR ROS_FOXY  ROS_DISTRO

## 得到工作空间的根目录，通过ROS的环境变量判断

## + Revision V0.1 @20230816
# support project version parsing
## + Revision V0.2 @20230905 
# fixed vscode cpp plugin failed in developing with container
## + Revision V0.3 @20230911 
# WS_ROOT_DIR can be defined by project CMake 
## + Revision V0.4 @20230927
# 支持编译时间，应用可以使用APP_BUILD_TIME的显示编译时间
## + Revision V0.5 @20230928
# 修改接口包在目标平台中无法编译的问题
## + Revision V0.6 @20240124
# 修改ROS版本不兼容的特性，这样在foxy和humble下都可以编译
## + Revision V0.7 @20240331
# 使用编译脚本传递的WS_INSTALL_DIR作为WS_INSTALL_PREFIX, 这样更灵活

if (NOT NO_ROS)

execute_process(COMMAND "printenv" "ROS_DISTRO" OUTPUT_VARIABLE ROS_DISTRO)
string(REPLACE "\n" "" ROS_DISTRO "${ROS_DISTRO}")
endif()

if (NOT "${ROS_DISTRO}" STREQUAL "")
set(WS_ROS ON)
# 设置ROS根目录
if (CMAKE_CROSSCOMPILING)
set(ROS_ROOT_DIR /opt/nano/rootfs/opt/ros/${ROS_DISTRO})
add_compile_options(-DNANO_CROSS_BUILD)
else()
set(ROS_ROOT_DIR /opt/ros/${ROS_DISTRO})
endif()
endif()

## 在编译目录可以设定根目录，这里可以不用设置
if (WS_ROS)
## 在ROS环境下
if (NOT WS_ROOT_DIR) 
get_filename_component(WS_ROOT_DIR ${PROJECT_SOURCE_DIR}/../../ ABSOLUTE)
endif()
# 设定安装目录
#set(WS_INSTALL_PREFIX ${WS_ROOT_DIR}/install/${PROJECT_NAME})
if (WS_INSTALL_DIR)
set(WS_INSTALL_PREFIX ${WS_INSTALL_DIR})
else()
set(WS_INSTALL_PREFIX ${WS_ROOT_DIR}/install)
endif()

# 设定ROSinclude为system的include目录
include_directories(SYSTEM ${ROS_ROOT_DIR}/include)
set(CMAKE_CXX_STANDARD 14)

if ("${ROS_DISTRO}" STREQUAL "humble")
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rclcpp)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcpputils)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcl)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcl_action)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcl_interfaces)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcl_yaml_param_parser)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rcutils)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rmw)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rosidl_runtime_c)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rosidl_typesupport_interface)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/rosidl_runtime_cpp)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/builtin_interfaces)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/std_srvs)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/std_msgs)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/sensor_msgs)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/statistics_msgs)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/geometry_msgs)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/tracetools)
include_directories(SYSTEM ${ROS_ROOT_DIR}/include/libstatistics_collector)

## 配置为C++标准17
set(CMAKE_CXX_STANDARD 17)
endif()

else()
## 不在ROS环境下
if (NOT WS_ROOT_DIR)  
get_filename_component(WS_ROOT_DIR ${PROJECT_SOURCE_DIR}/../ ABSOLUTE)
endif()
## 设定安装目录
set(WS_INSTALL_PREFIX ${WS_ROOT_DIR}/install/${CMAKE_SYSTEM_PROCESSOR})
endif()

message(STATUS "=> ROS_DISTRO=${ROS_DISTRO}")
message(STATUS "=> WS_ROOT_DIR=${WS_ROOT_DIR}")
message(STATUS "=> WS_INSTALL_PREFIX=${WS_INSTALL_PREFIX}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

if (CMAKE_CROSSCOMPILING)
set(CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH}" "${WS_INSTALL_PREFIX}")
message(STATUS "=> CMAKE_FIND_ROOT_PATH=${CMAKE_FIND_ROOT_PATH}")
endif()

## 设定应用版本号
set(GIT_COMMIT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tools/git_commit_mark.sh)

if (EXISTS ${GIT_COMMIT_SCRIPT})
execute_process(
    COMMAND bash ${GIT_COMMIT_SCRIPT} ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE LOCAL_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(APP_VERSION "V${PROJECT_VERSION}${LOCAL_VERSION}")
else()
set(APP_VERSION "V${PROJECT_VERSION}")
endif()

add_compile_options(-DAPP_VERSION=\"${APP_VERSION}\")
if (BUILD_TIME)
add_compile_options(-DAPP_BUILD_TIME=${BUILD_TIME}UL)
endif ()
message(STATUS "=> PROJECT_VERSION=${PROJECT_VERSION}")
message(STATUS "=> APP_VERSION=${APP_VERSION}")
message(STATUS "=> APP_BUILD_TIME=${BUILD_TIME}")
