#!/bin/bash
# Lancer le node qui fournit le service
python3 -m ros.pointing_ros_node &

# Récupérer le PID pour pouvoir l'arrêter plus tard
POINTING_PID=$!


# Lancer le node client
python3 -m ros.test_arm 

# Quand le client est terminé, arrêter le node service
kill $POINTING_PID

