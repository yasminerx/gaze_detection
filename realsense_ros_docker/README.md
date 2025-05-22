# RealSense ROS Docker Files
Simple docker compose file along with dockerfile to use Intel RealSense cameras with ROS inside docker environment
[Original ROS RealSense repo](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy/realsense2_camera/launch)

## Setup
- [install docker](https://docs.docker.com/get-docker/)
- clone this repo with `git clone git@github.com:hoenigpeter/ros_docker.git` 
- `docker compose up` to build and run the docker container
- this starts `roslaunch --wait realsense2_camera rs_camera.launch` with the basic RealSense topics
- change the docker-compose.yml file to enable other launch files such as: `roslaunch --wait realsense2_camera rs_d435_camera_with_model.launch.launch`
- this launch files also starts RViz visualisation, therefore you need to enable `xhost +` before
