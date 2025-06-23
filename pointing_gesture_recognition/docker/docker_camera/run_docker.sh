#!/bin/bash

docker run -it --name camera_docker_pointing_gaze -e ros.env --network host -v /home/yasmine/Documents/v4r_yasmine/gaze_detection/pointing_gesture_recognition:/root/code/ pointing_gesture_recognition bash
