cmake_minimum_required(VERSION 3.1.3)
project(octomap_updater)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  MESSAGE("The architecture is ${CMAKE_SYSTEM_PROCESSOR}")
else()
  MESSAGE("The architecture is ${CMAKE_SYSTEM_PROCESSOR}, pass to compile")
  RETURN()
endif ()

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# list(APPEND CMAKE_PREFIX_PATH "~/.local/lib/gpu_voxel/lib/")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)


if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  pluginlib
  image_transport
  object_recognition_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  nodelet
  pcl_ros
  pcl_conversions
)

find_package(CUDA REQUIRED)
find_package(icl_core REQUIRED )
find_package(gpu_voxels REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV)
find_package(VTK)
find_package(PCL REQUIRED)

set(PACKAGE_LIBRARIES
  moveit_point_containment_filter
  moveit_pointcloud_octomap_updater_fast_core
)

catkin_package(
  INCLUDE_DIRS
    point_containment_filter/include
    pointcloud_octomap_updater/include
    ${octomap_updater_GL_INCLUDE_DIRS}
  LIBRARIES
    ${PACKAGE_LIBRARIES}
  CATKIN_DEPENDS
    image_transport
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    moveit_ros_planning
    nodelet
    object_recognition_msgs
    roscpp
    sensor_msgs
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

add_definitions(
  ${icl_core_DEFINITIONS}
  ${gpu_voxels_DEFINITIONS}
)

message("PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS}")
include_directories(
                    point_containment_filter/include
                    pointcloud_octomap_updater/include
                    ${octomap_updater_GL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${gpu_voxels_INCLUDE_DIRS}
                    ${CUDA_INCLUDE_DIRS}
                    ${VTK_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    )

include_directories(SYSTEM
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${SYSTEM_GL_INCLUDE_DIR}
                    ${X11_INCLUDE_DIR}
                    )

add_subdirectory(point_containment_filter)
add_subdirectory(pointcloud_octomap_updater)

install(
  FILES
    pointcloud_octomap_updater_fast_description.xml
    moveit_depth_self_filter.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
