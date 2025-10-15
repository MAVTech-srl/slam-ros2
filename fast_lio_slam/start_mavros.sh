#!/bin/bash

# Check number of arguments
if [ "$#" -ne 2 ]; then
    echo "[ start_mavros.sh ] Illegal number of parameters. It must be exactly 2"
    exit 1
fi

docker run --rm --privileged --network=host -v /dev:/dev --name mavros ghcr.io/mavtech-srl/mavros:0.6-dev \
    ros2 launch mavros px4.launch fcu_url:=/dev/$1:$2