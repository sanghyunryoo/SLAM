cmake_minimum_required(VERSION 3.10)
project(quatro)

# Set compiler
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-fexceptions -g -ggdb")

# OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(teaserpp REQUIRED)

# Check package availability
if(PCL_FOUND)
  message(STATUS "PCL Version: ${PCL_VERSION}, PCL Include Directory: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING}, Eigen Include Directory: ${EIGEN3_INCLUDE_DIR}")
endif()
if(teaserpp_FOUND)
  message(STATUS "Teaser++ found")
endif()

# TBB configuration
if(QUATRO_TBB)
  message(STATUS "Quatro with TBB")
  list(APPEND CMAKE_MODULE_PATH /usr/local/include /usr/include)
  find_library(TBB_LIBRARY tbb HINTS /usr/lib /usr/local/lib)
  if(TBB_LIBRARY)
    message(STATUS "TBB Found: ${TBB_LIBRARY}")
    add_definitions(-DTBB_EN)
    include(ProcessorCount)
    ProcessorCount(N)
    message(STATUS "Processor Count: ${N}")
    if(N GREATER 4)
      add_definitions(-DTBB_PROC_NUM=${N})
    else()
      add_definitions(-DTBB_PROC_NUM=1)
    endif()
  else()
    message(FATAL_ERROR "TBB not found")
  endif()
endif()

# Debug configuration
if(QUATRO_DEBUG)
  add_definitions(-DQUATRO_DEBUG)
endif()

# Ament package configuration
find_package(ament_cmake REQUIRED)
ament_export_dependencies(Eigen3 PCL teaserpp OpenMP)
ament_export_include_directories(include)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########
# Main library
add_library(${PROJECT_NAME} SHARED src/fpfh.cc src/matcher.cc src/quatro_module.cc)
if (QUATRO_TBB)
  target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_io ${TBB_LIBRARY})
else()
  target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_io)
endif()
ament_target_dependencies(${PROJECT_NAME} Eigen3 PCL teaserpp OpenMP)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Ament package
ament_package()
