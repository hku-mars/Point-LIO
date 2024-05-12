cmake_minimum_required(VERSION 3.8)
project(point_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

ADD_COMPILE_OPTIONS(-std=c++14)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)


# OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

find_package(PythonLibs REQUIRED) # Consider using Pybind11 for ROS2
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

# Declare a ROS2 executable
add_executable(pointlio_mapping src/laserMapping.cpp
                src/li_initialization.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp 
                src/IMU_Processing.cpp)

ament_target_dependencies(pointlio_mapping
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

# Install the executable
install(TARGETS
        pointlio_mapping
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch rviz_cfg
        DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp rclpy geometry_msgs nav_msgs sensor_msgs pcl_ros pcl_conversions tf2_ros livox_ros_driver2 Eigen3 PCL)

ament_package()
