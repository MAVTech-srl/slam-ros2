#!/bin/bash

docker pull ghcr.io/mavtech-srl/mavros:0.1

docker run -it --rm --privileged --network=host -v /dev:/dev ghcr.io/mavtech-srl/mavros:0.1 \
    ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB:921600