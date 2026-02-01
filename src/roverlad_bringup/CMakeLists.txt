cmake_minimum_required(VERSION 3.8)
project(roverlad_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/roverlad_bringup
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  src/roverlad_bringup/commands_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
