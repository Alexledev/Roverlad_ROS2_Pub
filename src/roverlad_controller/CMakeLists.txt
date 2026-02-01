cmake_minimum_required(VERSION 3.8)
project(roverlad_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roverlad_interfaces REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs ${Eigen_LIBRARIES})

add_executable(controller_reader src/controller_reader.cpp)
ament_target_dependencies(controller_reader rclcpp geometry_msgs std_msgs roverlad_interfaces ${Eigen_LIBRARIES})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  controller_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
