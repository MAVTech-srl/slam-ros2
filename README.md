# Utilities for SLAM libraries
This repo contains Dockerfiles and utility files for different SLAM libraries. As of now, only fast-lio based SLAM is available.

## Fast-LIO
The folder ```fast_lio_slam``` contains utility scripts for MAVManager and the recipies for deploying fast-lio containers on the robots.
- The ```config``` folder contains configuration files automatically modified by MAVManager and passed as arguments to the fast-lio container. **Do NOT modify the content of this folder.**
- The ```docker``` folder contains recipies and instructions to build fast-lio and mavros containers.
- The bash scripts are utility scripts executed by MAVManager. **Do NOT modify or run these bash scripts.**
