#!/bin/bash

#add sourcing to .bashrc
echo "source /opt/ros/"$ROS_DISTRO"/setup.bash" >> /etc/bash.bashrc
echo "source /usr/src/mavros_ws/install/local_setup.bash" >> /etc/bash.bashrc

# setup ros2 environment
source /opt/ros/"$ROS_DISTRO"/setup.bash --
source /usr/src/mavros_ws/install/local_setup.bash --

exec "$@"