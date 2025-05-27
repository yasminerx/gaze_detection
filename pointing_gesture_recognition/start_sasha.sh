#!/bin/bash
source /root/catkin_ws/devel/setup.bash 
export CONFIG=params_sasha.yaml
rosparam load /root/code/${CONFIG} /pose_estimator
exec "$@"
