cmake_minimum_required(VERSION 3.10)

project(fast_lio_sam_sc_qn)

cmake_policy(SET CMP0074 NEW)

# Set compiler settings
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread -fexceptions -g -ggdb")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

# OpenMP support
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

# Find ROS 2 and external dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../third_party/scancontext_tro
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

# nano_gicp specific configuration
find_library(NANO_GICP_LIB nano_gicp PATHS /root/slam_ws/install/nano_gicp/lib REQUIRED)
if(NANO_GICP_LIB)
  message(STATUS "Found nano_gicp: ${NANO_GICP_LIB}")
else()
  message(FATAL_ERROR "nano_gicp library not found!")
endif()

# quatro specific configuration
find_library(QUATRO_LIB quatro PATHS /root/slam_ws/install/quatro/lib REQUIRED)
if(QUATRO_LIB)
  message(STATUS "Found quatro: ${QUATRO_LIB}")
else()
  message(FATAL_ERROR "quatro library not found!")
endif()

# Define common libraries
set(COMMON_LIBS
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}
  ${OpenMP_CXX_LIBRARIES}
  ${NANO_GICP_LIB}
  ${QUATRO_LIB}
  gtsam
)

# Build shared library
add_library(${PROJECT_NAME}_modules SHARED
  ${CMAKE_SOURCE_DIR}/../third_party/scancontext_tro/Scancontext.cpp
  src/loop_closure.cpp
  src/fast_lio_sam_sc_qn.cpp
)

target_link_libraries(${PROJECT_NAME}_modules ${COMMON_LIBS})
ament_target_dependencies(${PROJECT_NAME}_modules
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
  nano_gicp
  quatro  
  GTSAM
)

# Build executable node
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${COMMON_LIBS} ${PROJECT_NAME}_modules)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
  nano_gicp
  quatro  
  GTSAM
)

# Install targets and directories
install(TARGETS
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 수정된 부분
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
