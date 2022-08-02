cmake_minimum_required(VERSION 3.1.3)
project(moveit_capability)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  MESSAGE("The architecture is ${CMAKE_SYSTEM_PROCESSOR}")
else()
  MESSAGE("The architecture is ${CMAKE_SYSTEM_PROCESSOR}, pass to compile")
  RETURN()
endif()

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MOVEIT_LIB_NAME moveit_move_group_capabilities)

find_package(Boost REQUIRED system filesystem date_time program_options thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  actionlib
  roscpp
  pluginlib
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  LIBRARIES
    ${MOVEIT_LIB_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    moveit_core
    moveit_ros_move_group
    moveit_ros_planning
    roscpp
    std_srvs
    tf2_geometry_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})


add_library(${MOVEIT_LIB_NAME}
  src/move_action_capability.cpp
  src/execute_trajectory_action_capability.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES
  moveit_capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
  # this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
  # add_rostest(test/test_check_state_validity_in_empty_scene.test)
# endif()
