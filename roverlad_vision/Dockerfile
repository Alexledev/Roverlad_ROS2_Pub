FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ---- system basics ----
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-numpy \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    ca-certificates \
    curl \
    wget \
    git \
    nano \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 Jazzy ----
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools \
    ros-jazzy-nav2-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS environment ----
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV PATH=${ROS_ROOT}/bin:$PATH
ENV LD_LIBRARY_PATH=${ROS_ROOT}/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=${ROS_ROOT}/lib/python3.12/site-packages:$PYTHONPATH

ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET


RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH

RUN python3 -m pip install --upgrade pip setuptools wheel

# ---- OpenCV (Python) ----
RUN pip install opencv-python-headless 
#==4.8.1.78

# ---- PyTorch + TorchVision (Jetson-compatible) ----
RUN pip install --no-cache-dir \
    torch torchvision \
    --extra-index-url https://download.pytorch.org/whl/cu126 

RUN pip install catkin_pkg
RUN pip install pyyaml
RUN pip install empy lark
# RUN pip install numpy
#==1.26.4
    
# Mounted
# COPY onnxruntime/include /opt/onnxruntime/include
# COPY onnxruntime/build/Linux/Release/libonnxruntime.so.1.24.0 /opt/onnxruntime/lib/


WORKDIR /home/app
COPY . /home/app


# Auto-source ROS
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
ENV CYCLONEDDS_URI=file:///home/app/cyclone.xml
# RUN echo "source /home/app/proj/install/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["/home/app/on_startup.sh"]

#, "-c", "source /opt/ros/jazzy/setup.bash"
CMD ["bash"] 

