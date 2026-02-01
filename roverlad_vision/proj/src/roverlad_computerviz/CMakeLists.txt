cmake_minimum_required(VERSION 3.8)
project(roverlad_computerviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ONNXRUNTIME_ROOT "/opt/onnxruntime")
set(ONNXRUNTIME_DIR "/opt/onnxruntime/lib")
include_directories(
  ${ONNXRUNTIME_DIR}

  ${ONNXRUNTIME_ROOT}/include                 # base include
  ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session

  ${ONNXRUNTIME_ROOT}/onnxruntime
  ${ONNXRUNTIME_ROOT}/onnxruntime/core/session
)

link_directories(${ONNXRUNTIME_DIR})

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY roverlad_computerviz
  DESTINATION lib/
)

install(
  PROGRAMS roverlad_computerviz/yolo_stream_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
