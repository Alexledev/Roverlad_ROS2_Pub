# Miniature Autonomous Rover - Roverlad

![License](https://img.shields.io/badge/license-MIT-blue.svg)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green.svg)
![YDLIDAR](https://img.shields.io/badge/YDLIDAR-LiDAR-blue?logo=ros)
![YOLO](https://img.shields.io/badge/YOLOv8s-AI-orange.svg)

![STM32](https://img.shields.io/badge/STM32-FreeRTOS-lightgrey.svg)
![Jetson Orin Nano](https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano-green?logo=nvidia&logoColor=white)

![Docker](https://img.shields.io/badge/Docker-blue?logo=docker&logoColor=white)
![Nginx](https://img.shields.io/badge/nginx-green?logo=nginx&logoColor=white)
![FastAPI](https://img.shields.io/badge/FastAPI-009688?logo=fastapi&logoColor=white)
![WebSocket](https://img.shields.io/badge/WebSocket-enabled-blue?logo=websocket)
![WebRTC](https://img.shields.io/badge/WebRTC-real--time-orange?logo=webrtc)



### Extra Links: 
> **[STM32 Firmware Code](https://github.com/Alexledev/RoverLad)**

---

## Overview
### An autonomous mobile robot system with embedded real-time control, ROS2 navigation, AI-based perception, and a web-based monitoring interface.
The car features:
- **SLAM mapping and localisation** as well as **Nav2 navigation and pathfinding** using LIDAR and ROS2 Jazzy.
- **Object Detection** with YOLOv8s and OpenCV camera stream for emergency slow and stop for nearby obstacles.
- **FreeRTOS Integration** in the STM32 controller for real-time task management.
- **NGINX-hosted webapp** with mapping, navigation and settings page to replace Rviz2.
- **FastAPI, WebSockets, and WebRTC** integration with the webapp

---

## Technical Stack

- **Embedded Systems**
  - STM32 microcontroller
  - FreeRTOS real-time controller
- **Robotics Middleware**
  - ROS2 Jazzy
  - Nav2 navigation
  - SLAM mapping
  - LIDAR integration from 
- **Computer Vision & AI**
  - Camera streaming
  - YOLOv8s for real-time object detection
  - Emergency stop mechanism
- **Web Application**
  - Custom webapp replacing RViz
  - Features: monitoring, remote control, map storage & reuse, settings page
- **Docker Integration**
  - Easier deployment of Web and AI + Computer Vision features


## Key Features
- Multi-technology integration: Embedded, ROS2, AI, Web, Docker.
- Real-world applicability: autonomous navigation & obstacle handling.
- User-friendly: webapp interface instead of RViz.
- Extensible: modular design for future upgrades.


## Achievements
- Successfully built a mini autonomous car with **mapping, navigation, object detection (with average of 15~17 FPS), and emergency stop**.
- Developed a **web-based dashboard** for monitoring and control.
- Demonstrated integration of **embedded firmware, robotics middleware, AI, and web technologies** in one system.

  
## Future Improvements
- Utilize Depth Camera for distance perception without Lidar for cost-efficiency in Navigation.
- Optimize YOLO inference with **TensorRT** for faster real-time detection.
  
---


