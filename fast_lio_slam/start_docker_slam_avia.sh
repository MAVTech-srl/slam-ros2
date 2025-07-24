#!/bin/bash

## Helper script to start fast-lio SLAM with Livox AVIA
#
#
## NOTE: NO NOT RUN DIRECTLY THIS SCRIPT! USE MAVMANAGER TO RUN THIS!
#
#

help()
{
    echo "Helper script to start fast-lio SLAM with Livox AVIA

## NOTE: NO NOT RUN DIRECTLY THIS SCRIPT! USE MAVMANAGER TO RUN THIS!

Usage: bash start_docker_slam_avia.sh [OPTION]
[OPTION] are:
   --external-monitor    [=yes/no]      Execute with a plugged external monitor (default 'no')
   -h, --help                           Print this help"
   exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    --external-monitor=*)
      EXTERNAL_MONITOR="${1#*=}"
      ;;
    --help|-h)
      help
      ;;
    *)
      help
      exit 1
  esac
  shift
done


if [ "$EXTERNAL_MONITOR" = "yes" ]; then
    echo "Running with external monitor"
    xhost +

    DOCKER_ARGS+=("-e DISPLAY=:1")
    DOCKER_ARGS+=("--mount source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached")
    DOCKER_ARGS+=("-v /etc/X11:/etc/X11")
fi

REMOTE_USER=rosdev
DOCKER_ARGS+=("-v /tmp/:/tmp/")
DOCKER_ARGS+=("-v ${HOME}/Desktop/rosbag:/home/${REMOTE_USER}/ros2_ws/rosbag")
DOCKER_ARGS+=("--pid=host") 
LIDAR_CONFIG_PATH=$(echo "${PWD}/scripts/slam-ros2/fast_lio_slam/config/livox_lidar_config.json")
DOCKER_ARGS+=("-v ${LIDAR_CONFIG_PATH}:/home/${REMOTE_USER}/ros2_ws/install/livox_ros2_driver/share/livox_ros2_driver/config/livox_lidar_config.json")
    

 
PLATFORM=$(cat /proc/cpuinfo | grep 'Model' | awk '{print $3}')
if [ "$PLATFORM" = "Raspberry" ]; then # Run Raspberry image
    echo "Running Raspberry image"
    docker run -it --rm \
        --init \
        --privileged \
        --network host \
        --ipc=host \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-rasp-dev ros2 launch --noninteractive src/slam_tools/launch/slam_avia.launch.py sigterm_timeout:=3
elif [ -f /etc/nv_tegra_release ]; then # Run Jetson docker image
    echo "Running Jetson image"
    docker run -it --rm \
        --init \
        --privileged \
        --network host \
        --ipc=host \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.4-dev ros2 launch --noninteractive src/slam_tools/launch/slam_avia.launch.py sigterm_timeout:=3
fi