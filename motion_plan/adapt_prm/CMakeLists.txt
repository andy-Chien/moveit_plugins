add_library(${PROJECT_NAME}_planner SHARED
  src/adapt_prm.cpp
)
set_target_properties(${PROJECT_NAME}_planner PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME}_planner
  Boost
  OMPL
)
install(DIRECTORY include/ DESTINATION include)
