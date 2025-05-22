# Pointing Gesture Detector ROS Node

This repository contains a ROS node that processes depth and RGB image streams to detect arm keypoints and publish pointing arrows, a pointing indicator, and right arm joints. The published topics can be visualized in RViz.
This implementation is based on the work of Matthias Grabners Robot Vision project "Show me what to pick".

## Features

- Detects arm keypoints using MediaPipe.
- Publishes markers for arm joints and pointing arrows in RViz.
- Indicates whether a pointing gesture is detected.
- Configurable parameters for flexibility in different environments.

## Getting Started

### Prerequisites

- ROS (Robot Operating System)
- Docker (for running the node in a container)
- OpenCV
- MediaPipe
- NumPy
- Matplotlib

### Installation

Clone the repository:

```bash
git clone https://github.com/v4r-tuwien/pointing_gesture_recognition.git
cd pointing_gesture_recognition
```

### Docker Setup

Build the Docker image:

```bash
docker_build.sh
```

### Running the Node

You can start the ROS node using the following command:

```bash
python pointing_detector.py
```

### Configuration

The node can be configured via command-line arguments or ROS parameters. Here are the available options:

- `--frame-id`: Frame ID for the camera (default: `camera_color_optical_frame`)
- `--color-topic`: Topic for the RGB image stream (default: `/camera/color/image_raw`)
- `--depth-topic`: Topic for the depth image stream (default: `/camera/aligned_depth_to_color/image_raw`)
- `--arm-angle-thresh`: Threshold angle to detect pointing (default: `140.0`)
- `--arrow-length`: Length of the pointing arrow (default: `2.0`)
- `--model-complexity`: Complexity of the pose detection model (default: `1`)
- `--min-detection-confidence`: Minimum confidence for detection (default: `0.9`)
- `--static-image-mode`: Whether to treat the input as a batch of static images (default: `False`)

Example:

```bash
python pointing_detector.py --frame-id='camera_frame' --color-topic='/camera/rgb/image_raw' --depth-topic='/camera/depth/image_raw'
```

## Usage

The node publishes the following topics:

- `/pointing/shoulder_to_wrist` (visualization_msgs/Marker): Arrow from shoulder to wrist.
- `/pointing/elbow_to_wrist` (visualization_msgs/Marker): Arrow from elbow to wrist.
- `/pointing/arm_joints` (visualization_msgs/Marker): Line strip showing the arm joints.
- `/pointing/is_pointing` (std_msgs/Bool): Indicator whether a pointing gesture is detected.

These topics can be visualized in RViz for monitoring and debugging purposes.

## Sources and References

- [ROS Publisher/Subscriber Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [Introduction to Pose Detection](https://bleedaiacademy.com/introduction-to-pose-detection-and-basic-pose-classification/)
- [Converting Between ROS Images and OpenCV Images](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
- [RViz Marker Display Types](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- [RViz Marker Publishing Example](https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/)
- [Interactive Matplotlib Figures](https://matplotlib.org/stable/users/explain/figure/interactive.html)
- [Stack Overflow - Subplot Argument](https://stackoverflow.com/questions/3584805/what-does-the-argument-mean-in-fig-add-subplot111)