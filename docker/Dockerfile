FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. 필수 시스템, 빌드 도구 및 ROS 2 라이브러리 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    libpcl-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libusb-1.0-0-dev \
    udev \
    libpcap-dev \
    net-tools \
    iputils-ping \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-geometry-msgs \
    ros-humble-eigen3-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# 2. Livox-SDK2 설치 (Mid-360 라이다 통신용 필수 C++ SDK)
WORKDIR /root/thirdparty
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd /root/thirdparty && rm -rf Livox-SDK2

# 3. 워크스페이스 환경 설정 및 bashrc 구성
WORKDIR /root/zenith_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/zenith_ws/install/setup.bash ]; then source /root/zenith_ws/install/setup.bash; fi" >> /root/.bashrc

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash"]
