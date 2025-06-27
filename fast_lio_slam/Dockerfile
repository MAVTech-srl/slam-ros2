ARG BASE_IMAGE=dustynv/ros:humble-desktop-l4t-r36.4.0
FROM $BASE_IMAGE AS slam

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1

RUN rm /etc/apt/sources.list.d/ros2.list && \
    apt update && apt install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

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
	        mpich openmpi-bin openmpi-common openmpi-doc libopenmpi-dev \
            geographiclib-tools \
            libgeographic-dev \
            libeigen-stl-containers-dev 

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

RUN apt-get install -y gdbserver libgeographic-dev libeigen-stl-containers-dev libpdal-dev geographiclib-tools sqlite3
RUN pip install future lxml

RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh

RUN wget https://download.osgeo.org/proj/proj-9.6.2.tar.gz -O proj.tar.gz \
    && mkdir proj_source \
    && tar -xvf proj.tar.gz -C proj_source --strip-components=1 \
    && rm proj.tar.gz \
    && cd proj_source \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && sudo make -j8 install

# ADD repo's of submodules
WORKDIR /home/${USERNAME}/ros2_ws

RUN mkdir -p src && cd src/ && \
    git clone https://github.com/MAVTech-srl/FAST_LIO_ROS2.git --recursive ./fast_lio
    
RUN git clone https://github.com/MAVTech-srl/livox_avia_ros2_driver.git ./src/livox_avia_ros2_driver

RUN git clone https://github.com/MAVTech-srl/livox_ros_driver2.git ./src/livox_ros_driver2

RUN git clone https://github.com/ros-perception/perception_pcl.git --branch humble ./src/perception_pcl

RUN git clone https://github.com/MAVTech-srl/slam_tools.git ./src/slam_tools

RUN git clone https://github.com/ros-geographic-info/geographic_info.git --branch ros2 ./geographic_info

RUN git clone https://github.com/ros/diagnostics.git --branch ros2-humble ./src/diagnostics

RUN git clone https://github.com/ros2-gbp/mavlink-gbp-release.git --branch release/humble/mavlink ./src/mavlink

RUN git clone https://github.com/mavlink/mavros.git --branch ros2 ./src/mavros

RUN source /opt/ros/humble/install/setup.sh && colcon build

RUN mkdir -p rosbag
ENV SHELL="/bin/bash"

LABEL org.opencontainers.image.source="https://github.com/MAVTech-srl/slam-ros2.git"

# [Optional] Set the default user. Omit if you want to keep the default as root, though you won't be able to see GUI
# USER $USERNAME
COPY ./ros_entrypoint.sh /

ENTRYPOINT [ "/bin/bash", "/ros_entrypoint.sh"]
CMD ["bash"]