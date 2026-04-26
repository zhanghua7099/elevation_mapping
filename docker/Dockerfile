FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    locales \
    git \
    curl \
    wget \
    vim \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    x11-apps \
    mesa-utils \
    ros-jazzy-grid-map \
    && locale-gen en_US en_US.UTF-8 

RUN rosdep init || true && rosdep update || true

RUN git clone --branch v2.57.6 --depth 1 https://github.com/realsenseai/librealsense.git /opt/librealsense \
    && cd /opt/librealsense \
    && mkdir build && cd build \
    && cmake ../ \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=true \
        -DBUILD_PYTHON_BINDINGS=true \
        -DPYTHON_EXECUTABLE=$(which python3) \
    && make -j$(nproc) \
    && make install

# Copy udev rules for RealSense cameras
RUN mkdir -p /etc/udev/rules.d/ && cp /opt/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/

# Set Python path for pyrealsense2 (following README instructions)
ENV PYTHONPATH=/opt/librealsense/build/Release

RUN apt-get -y install "ros-jazzy-realsense2-*"

RUN apt-get -y install build-essential \
	cmake \
	curl \
	git \
	libgtk-3-dev \
	libxkbcommon-x11-0 \
	patch \
	python3-pip \
	python3-dev \
	unzip \
	vulkan-tools \
	wget \
	pkg-config \
	libusb-1.0-0-dev \
	libssl-dev \
	libglfw3-dev \
	libgl1-mesa-dev \
	libglu1-mesa-dev \
	libatlas-base-dev \
	libsuitesparse-dev \
	libeigen3-dev \
	ros-jazzy-imu-filter-madgwick

RUN apt -y install ros-jazzy-rtabmap-ros
RUN rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws/src

CMD ["bash"]
