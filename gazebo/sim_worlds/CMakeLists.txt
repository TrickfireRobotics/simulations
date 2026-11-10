cmake_minimum_required(VERSION 3.14)
project(sim_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    gui/
  DESTINATION share/${PROJECT_NAME}/gui
)

install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
