ARG BASE_IMAGE=dustynv/ros:humble-desktop-l4t-r36.4.0
FROM $BASE_IMAGE AS slam

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \	
    wget

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# # Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# # Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 

RUN apt-get remove -y libpcl-*

RUN apt-get install -y \
            git \
            python3-pip \
            nano \
            software-properties-common \
            wget \
            python3-colcon-common-extensions \
            pkg-config \
            libnanoflann-dev \
	        mpich openmpi-bin openmpi-common openmpi-doc libopenmpi-dev
# Install Livox SDK
WORKDIR /usr/src
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd ./Livox-SDK2/ \
    && mkdir build \
    && cd build \
    && cmake .. && make -j8 \
    && make install

# Install PCL
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz -O pcl.tar.gz \
    && mkdir pcl_source \
    && tar -xvf pcl.tar.gz -C pcl_source --strip-components=1 \
    && rm pcl.tar.gz \
    && cd pcl_source \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_GPU=ON .. \
    && sudo make -j8 install

RUN git clone https://github.com/shenxw0127/Livox-SDK-24.04.git ./Livox-SDK \
    && cd Livox-SDK \
    && git checkout 36207aaa9f1faa6eb9113138732490587fce6367 \
    && cd build && cmake .. \
    && make -j4 \
    && make install


# ADD repo's of submodules
WORKDIR /home/${USERNAME}/ros2_ws
RUN echo

RUN mkdir -p src && cd src/ && \
    git clone https://github.com/MAVTech-srl/FAST_LIO_ROS2.git --recursive ./fast_lio
    
RUN git clone https://github.com/MAVTech-srl/livox_avia_ros2_driver.git ./src/livox_avia_ros2_driver

RUN git clone https://github.com/MAVTech-srl/livox_ros_driver2.git ./src/livox_ros_driver2

RUN git clone https://github.com/ros-perception/perception_pcl.git --branch humble ./src/perception_pcl

RUN git clone https://github.com/MAVTech-srl/slam_tools.git ./src/slam_tools

RUN source /opt/ros/humble/install/setup.sh && colcon build

ENV SHELL="/bin/bash"

# [Optional] Set the default user. Omit if you want to keep the default as root, though you won't be able to see GUI
# USER $USERNAME
COPY ./ros_entrypoint.sh /

ENTRYPOINT [ "/bin/bash", "/ros_entrypoint.sh"]
CMD ["bash"]