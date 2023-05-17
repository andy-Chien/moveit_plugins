add_library(${PROJECT_NAME}_planner SHARED
  src/adapt_prm.cpp
  src/adapt_lazy_prm.cpp
)
set_target_properties(${PROJECT_NAME}_planner PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME}_planner
  moveit_core
  Boost
  OMPL
)
install(DIRECTORY include/ DESTINATION include)
