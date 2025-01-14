#############################################################
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Modifier: Eungchang Mason Lee                             #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(nano_gicp)

# Set compiler
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread -fexceptions -g -ggdb -fPIC")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

# OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Export dependencies
ament_export_dependencies(PCL Eigen3)
ament_export_include_directories(include)

###########
## Build ##
###########
# NanoFLANN
add_library(nanoflann src/nanoflann.cc)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann PCL Eigen3)

# NanoGICP
add_library(nano_gicp src/nano_gicp.cc)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_CXX_LIBRARIES} nanoflann)
ament_target_dependencies(nano_gicp PCL Eigen3)

# Install
install(TARGETS
  nanoflann
  nano_gicp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Ament package
ament_package()
