add_executable(test_updown
  test_updown.cpp
)

ament_target_dependencies(test_updown
  ${dependencies}
)

install(TARGETS test_updown
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
