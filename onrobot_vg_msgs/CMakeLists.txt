cmake_minimum_required(VERSION 3.16)
project(onrobot_vg_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/OnRobotVGInput.msg"
  "msg/OnRobotVGOutput.msg"
  "srv/SetCommand.srv"
) 

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()