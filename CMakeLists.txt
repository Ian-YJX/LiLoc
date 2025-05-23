cmake_minimum_required(VERSION 3.0.2)
project(liloc)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport
  pcl_ros
  ndt_omp
  livox_ros_driver
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(PythonLibs REQUIRED)
find_package(image_transport REQUIRED)

# 消息与服务
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
  save_session.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    message_generation
    visualization_msgs
    image_transport
)

# 包含头文件目录
include_directories(
  lib
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)

# 链接库目录
link_directories(
  lib
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

# 公共源文件
file(GLOB common_lib "lib/*.cpp")

###########
## 编译目标 ##
###########

# 投影节点
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
)

# 地图优化节点（加入 pose_estimator）
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp include/online-relo/pose_estimator.cpp include/sc-relo/Scancontext.cpp include/multi-session/Incremental_mapping.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization
  Boost::timer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
  stdc++fs
)

# IMU预积分节点
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration
  Boost::timer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
)
