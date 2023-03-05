add_library(${PROJECT_NAME}_ompl_tools SHARED
  tools/src/parallel_plan_with_return.cpp
)
set_target_properties(${PROJECT_NAME}_ompl_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME}_ompl_tools
  OMPL
)
install(DIRECTORY tools/include/ DESTINATION tools/include)
