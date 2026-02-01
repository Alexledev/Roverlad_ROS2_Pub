cmake_minimum_required(VERSION 3.8)
project(roverlad_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY bt config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/roverlad_navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS src/roverlad_navigation/config_node_server.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
