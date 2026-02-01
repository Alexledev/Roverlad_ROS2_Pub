cmake_minimum_required(VERSION 3.8)
project(roverlad_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(roverlad_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(include)

add_executable(controller_teleop src/controller_teleop.cpp)
ament_target_dependencies(controller_teleop rclcpp geometry_msgs roverlad_interfaces)
install(TARGETS controller_teleop
  DESTINATION lib/${PROJECT_NAME}
)


add_library(${PROJECT_NAME} SHARED
  src/roverlad_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
  "${SERIAL_LDFLAGS}"
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  roverlad_interfaces
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface roverlad_interface.xml)


add_executable(rover_comms 
  src/rover_comms.cpp 
  src/Kalman.cpp
)
ament_target_dependencies(rover_comms rclcpp std_msgs roverlad_interfaces)
target_include_directories(rover_comms PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(rover_comms "${SERIAL_LDFLAGS}")


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)


install (
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  rover_comms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  roverlad_interfaces
  std_msgs
)


ament_package()
