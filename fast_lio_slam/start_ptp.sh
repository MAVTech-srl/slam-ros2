#!/bin/bash
ETH_DEV_NAME=$(ls /sys/class/net | grep ^e)
sudo ptp4l -i ${ETH_DEV_NAME} -S -ml 6