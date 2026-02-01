
# FROM python:3.11-slim
FROM ros:jazzy-ros-base


ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv\
    ffmpeg \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    portaudio19-dev \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    ros-jazzy-nav2-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

RUN pip install --no-cache-dir aiohttp aiohttp-cors aiortc av opencv-python-headless sounddevice pyyaml 


WORKDIR /home/app
COPY . /home/app
# COPY webRTCServer_host.py /home/app
# COPY config.yaml /home/app/config/config.yaml
# COPY load_config.py /home/app
# COPY micfinder.py /home/app
# COPY cyclone.xml /home/app
# COPY on_startup.sh /home/app
# COPY ros2_ws /home/app/ros2_ws

ENV CYCLONEDDS_URI=file:///home/app/cyclone.xml
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

SHELL ["/bin/bash", "-c"]
# CMD source /opt/ros/jazzy/setup.bash && python3 -u webRTCServer_host.py

ENTRYPOINT ["/home/app/on_startup.sh"]

CMD ["bash"] 