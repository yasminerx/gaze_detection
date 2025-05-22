source /root/catkin_ws/devel/setup.bash 
export CONFIG=params_realsense.yaml
rosparam load /code/${CONFIG} /pose_estimator