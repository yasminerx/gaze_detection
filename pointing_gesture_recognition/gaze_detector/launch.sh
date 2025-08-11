#!/bin/bash
# Lancer le node qui fournit le service
python3 -m ros.pointing_ros_node &

# Récupérer le PID pour pouvoir l'arrêter plus tard
POINTING_PID=$!

python3 -m ros.gaze_ros_node &
GAZE_PID=$!


# Lancer le node client
python3 -m ros.test_arm
python3 -m ros.test_gaze

# Quand le client est terminé, arrêter le node service
kill $POINTING_PID
kill $GAZE_PID
