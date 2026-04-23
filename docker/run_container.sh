#!/bin/bash

CONTAINER_NAME="elevation_ros2_container"
IMAGE_NAME="elevation-ros-jazzy"

xhost +local:docker

if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "容器已在运行，直接进入..."
    docker exec -it "$CONTAINER_NAME" bash
elif docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "容器已存在，正在启动..."
    docker start "$CONTAINER_NAME" >/dev/null
    docker exec -it "$CONTAINER_NAME" bash
else
    echo "容器不存在，正在创建..."
    docker run -it \
      --name "$CONTAINER_NAME" \
      --gpus all \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      --privileged \
      --network host \
      --ipc host \
      --pid host \
      -e DISPLAY="$DISPLAY" \
      -e XAUTHORITY=$XAUTH \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=0 \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$(pwd)":/ros2_ws/src/ \
      -v $XSOCK:$XSOCK \
      -v $XAUTH:$XAUTH \
      -v /dev/bus/usb:/dev/bus/usb \
      -w /ros2_ws/ \
      "$IMAGE_NAME"
fi

