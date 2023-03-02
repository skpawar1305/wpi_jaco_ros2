cmake_minimum_required(VERSION 3.5)
project(wpi_jaco_wrapper)

set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(wpi_jaco_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

###########
## Build ##
###########

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "64bit")
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "32bit")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

include_directories(
  include
)

#############
## Install ##
#############

## Declare a cpp executable
add_executable(jaco_arm_trajectory_node
  src/jaco_arm_trajectory_node.cpp
  src/jaco_kinematics.cpp
  src/jaco_conversions.cpp
)
ament_target_dependencies(jaco_arm_trajectory_node rclcpp rclcpp_action control_msgs ecl_geometry wpi_jaco_msgs std_srvs tf2)
target_link_libraries(jaco_arm_trajectory_node
  ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libKinova.API.CommLayerUbuntu.so
  ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libKinova.API.USBCommandLayerUbuntu.so
)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(jaco_arm_trajectory_node
#   ${catkin_LIBRARIES}
# )
# ## Add cmake target dependencies of the executable/library
# add_dependencies(jaco_arm_trajectory_node wpi_jaco_msgs_gencpp)

# ## Declare a cpp executable
# add_executable(jaco_manipulation src/jaco_manipulation.cpp)
# ## Specify libraries to link a library or executable target against
# target_link_libraries(jaco_manipulation
#   ${catkin_LIBRARIES}
# )
# ## Add cmake target dependencies of the executable/library
# add_dependencies(jaco_manipulation wpi_jaco_msgs_gencpp rail_manipulation_msgs_gencpp)

# ## Declare a cpp executable
# add_executable(jaco_kinematics src/jaco_kinematics.cpp)
# ## Specify libraries to link a library or executable target against
# target_link_libraries(jaco_kinematics
#   ${catkin_LIBRARIES}
# )
# ## Add cmake target dependencies of the executable/library
# add_dependencies(jaco_kinematics wpi_jaco_msgs_gencpp)

# ## Declare a cpp executable
# add_executable(jaco_conversions src/jaco_conversions.cpp)
# ## Specify libraries to link a library or executable target against
# target_link_libraries(jaco_conversions
#   ${catkin_LIBRARIES}
# )
# ## Add cmake target dependencies of the executable/library
# add_dependencies(jaco_conversions wpi_jaco_msgs_gencpp)

# #############
# ## Install ##
# #############

# ## Mark executables and/or libraries for installation
# install(TARGETS jaco_conversions jaco_kinematics jaco_manipulation jaco_arm_trajectory_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Copy header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
# )

## Copy launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

## Copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

## LIBRARIES: libraries you create in this project that dependent projects also need
install(FILES
  lib/${ARCH}/libKinova.API.CommLayerUbuntu.so
  lib/${ARCH}/libKinova.API.USBCommandLayerUbuntu.so
  DESTINATION lib
)

## Mark cpp header files for installation
install(DIRECTORY include/jaco_sdk/
  DESTINATION include/jaco_sdk
  FILES_MATCHING PATTERN "*.h"
)

# Mark executables and/or libraries for installation
install(TARGETS jaco_arm_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()