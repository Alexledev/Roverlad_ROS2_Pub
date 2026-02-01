cmake_minimum_required(VERSION 3.8)
project(roverlad_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# add_executable(imu_republisher src/imu_republisher.cpp)
# ament_target_dependencies(imu_republisher rclcpp sensor_msgs)

# install(TARGETS
#   imu_republisher
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

