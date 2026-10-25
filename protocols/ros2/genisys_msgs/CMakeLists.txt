cmake_minimum_required(VERSION 3.8)

project(genisys_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joint.msg"
  "msg/MoveResponse.msg"
  "msg/RosChangeMe.msg"
  "msg/ServoJCommand.msg"
  "msg/ServoJPoseCommand.msg"
  "msg/ServoLCommand.msg"
  "msg/TaskContext.msg"
  "srv/GripperForce.srv"
  "srv/GripperPos.srv"
  "srv/MoveJ.srv"
  "srv/MoveJMulti.srv"
  "srv/MoveJPose.srv"
  "srv/MoveJPoseMulti.srv"
  "srv/MoveL.srv"
  "srv/MoveLMulti.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
