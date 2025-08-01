#!/bin/bash

## Helper script to start fast-lio SLAM with Livox MID360
#
#
## NOTE: NO NOT RUN DIRECTLY THIS SCRIPT! USE MAVMANAGER TO RUN THIS!
#
#

help()
{
    echo -e "Helper script to start fast-lio SLAM with Livox MID360

\033[31;1;4m## NOTE: NO NOT RUN DIRECTLY THIS SCRIPT! USE MAVMANAGER TO RUN THIS!\033[0m

Usage: bash start_docker_slam_avia.sh [OPTION]
[OPTION] are:
   --external-monitor    [=yes/no]      Execute with a plugged external monitor. This will enable Rviz2 (default 'no')
   --save-pcd-cloud
   --save-utm-pcd-cloud
   -h, --help                           Print this help"
   exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    --external-monitor=*)
      EXTERNAL_MONITOR="${1#*=}"
      ;;
    --save-pcd-cloud=*)
      SAVE_PCD_CLOUD="${1#*=}"
      ;;
    --save-utm-pcd-cloud=*)
      SAVE_UTM_PCD_CLOUD="${1#*=}"
      ;;
    --convert-livox-cloud=*)
      CONVERT_LIVOX_CLOUD="${1#*=}"
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

RVIZ_USE=False
if [ "$EXTERNAL_MONITOR" = "yes" ]; then
    echo "Running with external monitor"
    xhost +

    DOCKER_ARGS+=("-e DISPLAY=:1")
    DOCKER_ARGS+=("--mount source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached")
    DOCKER_ARGS+=("-v /etc/X11:/etc/X11")

    RVIZ_USE=True
    echo "Rviz Running"
fi

LOCAL_PCD_USE=False
if [ "$SAVE_PCD_CLOUD" = "yes" ]; then
    LOCAL_PCD_USE=True
fi

UTM_PCD_USE=False
if [ "$SAVE_UTM_PCD_CLOUD" = "yes" ]; then
    UTM_PCD_USE=True
fi

CONVERT=False
if [ "$CONVERT_LIVOX_CLOUD" = "yes" ]; then
    CONVERT=True
fi

DOCKER_ARGS+=("-v /tmp/:/tmp/")
REMOTE_USER=rosdev
DOCKER_ARGS+=("-v ${HOME}/Desktop/rosbag:/home/${REMOTE_USER}/ros2_ws/rosbag")
DOCKER_ARGS+=("--pid=host")

PLATFORM=$(cat /proc/cpuinfo | grep 'Model' | awk '{print $3}')
if [ "$PLATFORM" = "Raspberry" ]; then # Run Raspberry image
    docker run --rm \
        --init \
        --privileged \
        --network host \
        --ipc=host \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.5-rasp-dev ros2 launch --noninteractive src/slam_tools/launch/slam.launch.py \
              rviz:=$RVIZ_USE \
              save_pcd_cloud:=$LOCAL_PCD_USE \
              save_UTM_pcd_cloud:=$UTM_PCD_USE \
              convert_livox_cloud:=$CONVERT \
              sigterm_timeout:=10
elif [ -f /etc/nv_tegra_release ]; then # Run Jetson docker image
    docker run --rm \
        --init \
        --privileged \
        --network host \
        --ipc=host \
        ${DOCKER_ARGS[@]} \
        --name fast-lio-slam \
        ghcr.io/mavtech-srl/fast-lio-slam:0.5-dev ros2 launch --noninteractive src/slam_tools/launch/slam.launch.py \
              rviz:=$RVIZ_USE \
              save_pcd_cloud:=$LOCAL_PCD_USE \
              save_UTM_pcd_cloud:=$UTM_PCD_USE \
              convert_livox_cloud:=$CONVERT \
              sigterm_timeout:=10
fi