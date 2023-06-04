add_library(${PROJECT_NAME}_objectives SHARED
  src/PathLengthUtilizationOptimizationObjective.cpp
)
ament_target_dependencies(${PROJECT_NAME}_objectives
  OMPL
  moveit_core
)
install(DIRECTORY include/ DESTINATION include)
