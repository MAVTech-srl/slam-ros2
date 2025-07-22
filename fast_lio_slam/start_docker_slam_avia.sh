#!/bin/bash
xhost +

DOCKER_ARGS+=("-e DISPLAY=:1")
REMOTE_USER=rosdev
DOCKER_ARGS+=("-v /tmp/:/tmp/")
DOCKER_ARGS+=("-v ${HOME}/Desktop/rosbag:/home/${REMOTE_USER}/ros2_ws/rosbag")
DOCKER_ARGS+=("--pid=host") 
LIDAR_CONFIG_PATH=$(echo "${PWD}/scripts/slam-ros2/fast_lio_slam/config/livox_lidar_config.json")
DOCKER_ARGS+=("-v ${LIDAR_CONFIG_PATH}:/home/${REMOTE_USER}/ros2_ws/install/livox_ros2_driver/share/livox_ros2_driver/config/livox_lidar_config.json")
    
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
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-rasp-dev ros2 launch src/slam_tools/launch/slam_avia.launch.py sigterm_timeout:=300
elif [ -f /etc/nv_tegra_release ]; then # Run Jetson docker image
    docker run -it --rm \
        --privileged \
        --network host \
        --ipc=host \
        -v /etc/X11:/etc/X11 \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-dev ros2 launch src/slam_tools/launch/slam_avia.launch.py sigterm_timeout:=300
