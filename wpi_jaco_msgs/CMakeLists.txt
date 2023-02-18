cmake_minimum_required(VERSION 3.5)
project(wpi_jaco_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AngularCommand.msg"
  "msg/CartesianCommand.msg"
  "msg/Euler.msg"
  "msg/JacoFingerVel.msg"
  "msg/Joints.msg"
  "srv/EStop.srv"
  "srv/EulerToQuaternion.srv"
  "srv/GetAngularPosition.srv"
  "srv/GetCartesianPosition.srv"
  "srv/JacoFK.srv"
  "srv/QuaternionToEuler.srv"
  "action/HomeArm.action"
  "action/Gripper.action"
  "action/Lift.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
