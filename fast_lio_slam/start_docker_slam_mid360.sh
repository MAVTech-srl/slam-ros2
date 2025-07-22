#!/bin/bash
xhost +

DOCKER_ARGS+=("-e DISPLAY=:1")

DOCKER_ARGS+=("-v /tmp/:/tmp/")
REMOTE_USER=rosdev
DOCKER_ARGS+=("-v ${HOME}/Desktop/rosbag:/home/${REMOTE_USER}/ros2_ws/rosbag")
DOCKER_ARGS+=("--pid=host") 
    
DOCKER_ARGS+=("--mount source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached")
 
PLATFORM=$(cat /proc/cpuinfo | grep 'Model' | awk '{print $3}')
if [ "$PLATFORM" = "Raspberry" ]; then # Run Raspberry image
    docker run -it --rm \
        --privileged \
        --network host \
        --ipc=host \
        -v /etc/X11:/etc/X11 \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-rasp-dev ros2 launch src/slam_tools/launch/slam.launch.py sigterm_timeout:=300
elif [ -f /etc/nv_tegra_release ]; then # Run Jetson docker image
    docker run -it --rm \
        --privileged \
        --network host \
        --ipc=host \
        -v /etc/X11:/etc/X11 \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-dev ros2 launch src/slam_tools/launch/slam.launch.py sigterm_timeout:=300
