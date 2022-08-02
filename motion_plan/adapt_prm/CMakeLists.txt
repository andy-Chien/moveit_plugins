add_library(${PROJECT_NAME}_planner
  src/adapt_prm.cpp
)

target_link_libraries(${PROJECT_NAME}_planner
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

