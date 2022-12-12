add_library(${PROJECT_NAME}_objectives
  src/PathLengthUtilizationOptimizationObjective.cpp
)

target_link_libraries(${PROJECT_NAME}_objectives
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_objectives
  RUNTIME DESTINATION lib/${PROJECT_NAME}_objectives
)
install(DIRECTORY include/ DESTINATION include)
