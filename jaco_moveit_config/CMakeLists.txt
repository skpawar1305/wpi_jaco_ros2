cmake_minimum_required(VERSION 3.5)
project(jaco_moveit_config)

find_package(ament_cmake REQUIRED)

## Copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

## Copy launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
