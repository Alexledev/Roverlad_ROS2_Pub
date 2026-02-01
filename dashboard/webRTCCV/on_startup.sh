#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/app/ros2_ws/install/setup.bash

exec python -u webRTCServer_host.py 