#!/bin/bash
xhost +

DOCKER_ARGS+=("-e DISPLAY=:1")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# DOCKER_ARGS+=("--user 1000")
#DOCKER_ARGS+=("-e USER")
#DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
#DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")

DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
DOCKER_ARGS+=("-v /tmp/:/tmp/")
DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
DOCKER_ARGS+=("--pid=host") 
    
DOCKER_ARGS+=("--mount source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached")
DOCKER_ARGS+=("--mount source=/dev/dri,target=/dev/dri,type=bind,consistency=cached")  
 
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -v /etc/X11:/etc/X11 \
    ${DOCKER_ARGS[@]} \
    --runtime nvidia \
    ghcr.io/mavtech-srl/fast-lio-slam ros2 launch src/slam_tools/launch/slam_avia.launch.py
