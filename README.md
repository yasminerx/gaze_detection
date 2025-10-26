# Gaze and pointing detection

## Pointing and gaze detection node : 
```bash
cd pointing_gesture_recognition
```

This repository contains a ROS node that processes depth and RGB image streams to detect arm keypoints as well as gaze direction and publishes pointing the correspondings arrows. The published topics can be visualized in RViz.
This implementation is based on the work of Matthias Hirschmanner "Show me what to pick" : https://github.com/v4r-tuwien/pointing_gesture_recognition


### Features

- Detects arm keypoints using MediaPipe.
- Detects the pupil and other useful face keypoints using Mediapipe.
- Publishes markers for arm joints and pointing arrows in RViz.
- Publishes arros for gaze direction in RViz.
- Indicates whether a pointing gesture is detected.
- Indicates whether a face is detected.
- Configurable parameters for flexibility in different environments.

### Getting Started

#### Prerequisites

- ROS (Robot Operating System)
- Docker (for running the node in a container)
- OpenCV
# Gaze & Pointing Detection (project overview)

This repository contains modules and tools for gaze detection and pointing-gesture detection in Python. The code can be run locally, inside Docker containers, and integrated with ROS (rospy).

This README explains:
- repository layout and important folders
- how to install and run modules locally (virtualenv)
- how to use the provided Docker images/containers
- how to run the ROS nodes and visualize results in RViz
---

## Arborescence clé

- `pointing_gesture_recognition/` — code principal (détecteurs, librairies `gaze_lib/` et `arm_lib/`, scripts `dev/`, utilitaires et `ros/` wrappers).
- `mediapipe/` — scripts tests et un environnement virtuel (`env/`) pour MediaPipe.
- `realsense_ros_docker/` — configuration Docker / docker-compose pour une Intel RealSense (fournit les topics ROS camera).
- `start/` — paramètres et scripts de démarrage (`params_realsense.yaml`, `start_realsense.sh`, ...).
- `rosbags/` — enregistrements ROS (bagfiles) pour tests hors-ligne.
- `rapport/` — sources LaTeX du rapport de stage.

---


## Prerequisites

- Linux (Ubuntu recommended for ROS compatibility)
- Python 3.8+
- pip
- Docker & docker-compose (if using containers)
- ROS (if you want to run ROS nodes natively)
- Python dependencies: OpenCV, NumPy, MediaPipe, matplotlib (optionally filterpy)

Note: some subfolders include specific virtual environments or install scripts (e.g. `mediapipe/env/`).

---


## Local installation (recommended for development)

1. Clone the repository:

```bash
git clone https://github.com/yasminerx/gaze_detection.git
cd gaze_detection
```

2. Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

3. Install core Python dependencies:

```bash
pip install numpy opencv-python mediapipe matplotlib
# install other packages if needed (e.g. filterpy)
```

If you prefer to reuse the provided MediaPipe environment, check `mediapipe/env/`.

---


## Main scripts / executables

- Gaze detector (examples): `pointing_gesture_recognition/dev/gaze_detector.py` and `pointing_gesture_recognition/gaze_detector/`
- Pointing detector: `pointing_gesture_recognition/dev/pointing_detector.py` and `pointing_gesture_recognition/arm_lib/`
- ROS wrappers: `pointing_gesture_recognition/ros/gaze_ros_node.py`, `pointing_gesture_recognition/ros/pointing_ros_node.py`, etc.

Run a development script locally (without Docker):

```bash
source .venv/bin/activate
python pointing_gesture_recognition/dev/gaze_detector.py
python pointing_gesture_recognition/dev/pointing_detector.py
```

These scripts can be adapted to read a video file or a rosbag for offline testing.

---


## Running with Docker

There are several Docker folders and compose setups in the project:

1. RealSense container - `realsense_ros_docker/` publishes the camera ROS topics.

```bash
cd realsense_ros_docker
docker-compose up --build
```

2. Containers to run detection code (e.g. `pointing_gesture_recognition/docker/docker_camera/`):

Check for `run_docker.sh` or `docker_build.sh` scripts in each docker folder. Example:

```bash
cd pointing_gesture_recognition/docker/docker_camera
./run_docker.sh
```

After containers are up, verify ROS topics with `rostopic list` and open RViz to visualize `Marker` topics.

---


## ROS & RViz usage

The ROS wrappers typically publish:

- `visualization_msgs/Marker` for arrows/joints (arm joints, gaze arrow)
- `std_msgs/Bool` for boolean states (e.g. `is_pointing`)
- other local topics/services depending on the script

Example to run a ROS wrapper (in a ROS-enabled environment):

```bash
source /opt/ros/<distro>/setup.bash
roscore &
python pointing_gesture_recognition/ros/pointing_ros_node.py
```

Open RViz and add a `Marker` display pointing to the topics published by the node.

---

## Offline testing (rosbag)

Replay a bagfile for testing without hardware:

```bash
rosbag play rosbags/pointing_gaze_0.bag
```
---


## Troubleshooting quick tips

- If MediaPipe installation fails, try a fresh virtualenv and install `mediapipe` via pip.
- If camera topics are missing, start the RealSense container or replay a rosbag.
- If you have ROS/Python version problems, ensure `rospy` is installed for the Python used by ROS.

## Useful commands (collected from project notes)

Below are helpful commands copied and adapted from the project's `commandes_utiles.txt`.

- Run a container with host networking and mount the code directory:

```bash
docker run -it --network host -v /home/yasmine/Documents/v4r/pointing_gesture_recognition:/code pointing_gesture_recognition bash
```

- Start an existing container by name and run roscore:

```bash
docker start docker_pointing
# then inside the container (or a container shell):
roscore
```

- Rename a running container (find its name with `docker ps`):

```bash
docker ps
docker rename old_name docker_pointing
docker exec -it docker_pointing bash
```

- Inside the container: build and source a catkin workspace:

```bash
catkin build
source /root/catkin_ws/devel/setup.bash
export CONFIG=params_realsense.yaml
rosparam load /code/${CONFIG} /pose_estimator
```

- Publish static transforms (example):

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 head_tilt_link head_rgbd_sensor_rgb_frame 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_link 100
```

- Useful git shortcuts:

```bash
# undo last commit but keep changes staged
git reset HEAD~1

# show branches and commits graph
git log --oneline --graph --all

# fetch remote commits
git fetch

# merge latest from origin/main into current branch
git merge origin/main
```

- Inspect tf transforms between frames:

```bash
rosrun tf tf_echo map head_rgbd_sensor_rgb_frame
```

- Record a rosbag with specific topics (example):

```bash
rosbag record /hsrb/head_rgbd_sensor/rgb/image_rect_color /hsrb/head_rgbd_sensor/depth_registered/image_rect_raw /found_markers_array /gaze --output-name=test_run_0722.bag -b 2048 --lz4
```

- Record split rosbag and include multiple topics (example):

```bash
rosbag record -O pointing_gaze_frame --split --size=2048 \
	/pointing/arm_joints \
	/pointing/x_vector \
	/pointing/y_vector \
	/pointing/z_vector \
	/pointing/left_eye \
	/pointing/right_eye \
	/pointing/elbow_to_wrist \
	/pointing/gaze \
	/pointing/filtered_gaze \
	/camera/color/image_raw \
	/camera/color/camera_info \
	/camera/aligned_depth_to_color/image_raw \
	/camera/aligned_depth_to_color/camera_info \
	/tf \
	/tf_static
```

- If RViz fails to start due to graphics, force software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz
```
