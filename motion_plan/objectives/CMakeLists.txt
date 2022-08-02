add_library(${PROJECT_NAME}_objectives
  src/PathLengthUtilizationOptimizationObjective.cpp
)

target_link_libraries(${PROJECT_NAME}_objectives
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

