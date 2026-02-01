#!/usr/bin/env bash
set -e

pip install /opt/onnxruntime/lib/dist/onnxruntime_gpu-*.whl

export LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
export PYTHONPATH=${ROS_ROOT}/lib/python3.12/site-packages:$PYTHONPATH

source /opt/ros/jazzy/setup.bash

cd /home/app/proj
colcon build

source install/setup.bash

exec ros2 run roverlad_computerviz yolo_stream_node.py