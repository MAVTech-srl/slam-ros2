# Deploying the containers
Once pushed the changes to this [fast-lio repo](https://github.com/MAVTech-srl/FAST_LIO_ROS2), build the needed containers using the utility bash file:
```bash
# cd to this folder
bash build_images.sh --image=<fast-lio|mavros> --target=<jetson|raspberry> (--tag=<tag>) (--push)
```
where:
- ```--image=``` specifies whether to build fast-lio or MAVROS image.
- ```--target=``` specifies which companion PC will run the container. It has no effect when building MAVROS image.
- (Optional) ```--tag=``` sets the tag (defaults to "latest")
- (Optional) ```--push``` pushes the image to ghcr

**Recipies:**
- ```Dockerfile``` builds a container using as base image ```dustynv/ros:humble-desktop-l4t-r36.4.0```. It is compatible only with Jetson boards and it includes CUDA libraries and drivers;
- ```Dockerfile.rasp``` builds a container for ARM64 platforms. It is compatible with Raspberry Pi 5;
- ```Dockerfile.mavros``` builds a container for MAVROS. 