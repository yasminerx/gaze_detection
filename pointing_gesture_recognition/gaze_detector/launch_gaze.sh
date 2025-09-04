#!/bin/bash

# Récupérer le PID pour pouvoir l'arrêter plus tard
python3 -m ros.gaze_ros_node &
GAZE_PID=$!


# Lancer le node client
python3 -m ros.test_gaze 

# Quand le client est terminé, arrêter le node service
kill $GAZE_PID

