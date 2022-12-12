add_library(${PROJECT_NAME}_planner SHARED
  src/adapt_prm.cpp
)

target_link_libraries(${PROJECT_NAME}_planner
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_planner
  moveit_core
  rclcpp
  Boost
  ompl
)
install(TARGETS ${PROJECT_NAME}_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}_planner
)
install(DIRECTORY include/ DESTINATION include)
