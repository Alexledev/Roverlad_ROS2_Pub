cmake_minimum_required(VERSION 3.8)
project(roverlad_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverData.msg"
  "msg/ControllerData.msg"

  "srv/GetSpeedConfig.srv"
  "srv/GetSpeedConfigs.srv"
  "srv/GetMap.srv"
  "srv/GetMaps.srv"
  "srv/SetInitialPose.srv"

  "action/GetSpeed.action"
  "action/SetSpeed.action"
  "action/SetSaveSpeeds.action"
  "action/SaveMap.action"
  "action/SwitchMap.action"
  "action/ResetRos.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
