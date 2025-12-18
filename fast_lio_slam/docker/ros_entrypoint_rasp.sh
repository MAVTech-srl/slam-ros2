#!/bin/bash

# add sources to .bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /home/rosdev/ros2_ws/install/local_setup.bash" >> ~/.bashrc

source /opt/ros/"$ROS_DISTRO"/setup.bash --
source /home/rosdev/ros2_ws/install/local_setup.bash --

exec "$@"
