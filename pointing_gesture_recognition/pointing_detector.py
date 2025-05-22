#!/usr/bin/env python3

# ROS node that takes depth and RGB image streams, detects arm keypoints and publishes pointing arrows, a pointing indicator and right arm joints
# Published topic can be displayed in RVIZ

# Sources:
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
# https://bleedaiacademy.com/introduction-to-pose-detection-and-basic-pose-classification/
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# http://wiki.ros.org/rviz/DisplayTypes/Marker
# http://docs.ros.org/en/jade/api/visualization_msgs/html/msg/Marker.html
# https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/
# https://answers.ros.org/question/380232/rviz-shows-uninitialized-quaternion-assuming-identity-when-displaying-octomap-mapping/
# http://wiki.ros.org/roslaunch
# http://wiki.ros.org/roslaunch/XML/param
# https://matplotlib.org/stable/users/explain/figure/interactive.html
# https://stackoverflow.com/questions/3584805/what-does-the-argument-mean-in-fig-add-subplot111

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from object_detector_msgs.srv import estimate_pointing_gesture
from object_detector_msgs.srv import estimate_pointing_gestureResponse


import cv2
import numpy as np
import mediapipe as mp
import argparse

from utils import *

class PointingDetector:
    def __init__(
            self,
            frame_id="camera_color_optical_frame",
            color_topic="/camera/color/image_raw",
            depth_topic="/camera/aligned_depth_to_color/image_raw",
            arm_angle_thresh=120.0,
            arrow_length=2.0,
            model_complexity=1,
            min_detection_confidence=0.9,
            static_image_mode=False,
                ):

        self.camera_info = np.asarray(rospy.get_param('/pose_estimator/intrinsics'))
        self.depth_encoding = rospy.get_param('/pose_estimator/depth_encoding')
        self.depth_scale = rospy.get_param('/pose_estimator/depth_scale')        
        self.frame_id = frame_id
        self.color_topic = color_topic
        self.depth_topic = depth_topic

        self.arm_angle_thresh = arm_angle_thresh
        self.arrow_length = arrow_length

        # Setup CvBridge to convert ROS messages to OpenCV readable images
        self.bridge = CvBridge()

        # Initialize mediapipe pose class
        self.mp_pose = mp.solutions.pose

        # Pose detection model complexity (0, 1, 2) can be set as rosparam
        model_complexity = model_complexity
        if model_complexity not in [0, 1, 2]:
            model_complexity = 1
        print(f"Model complexity = {model_complexity}")

        # Setup pose detection model
        self.pose = self.mp_pose.Pose(static_image_mode=static_image_mode, min_detection_confidence=min_detection_confidence, model_complexity=model_complexity)
        
        self.service = rospy.Service("/detect_pointing_gesture", estimate_pointing_gesture, self.detect_pointing_gesture)
        rospy.loginfo("Service /detect_pointing_gesture started")
        # Setup publishing of pointing arrows, arm joint markers and pointing indicator
        self.pub_arrow_shoulder = rospy.Publisher("/pointing/shoulder_to_wrist", Marker, queue_size=10)
        self.pub_arrow_elbow = rospy.Publisher("/pointing/elbow_to_wrist", Marker, queue_size=10)
        self.pub_lines = rospy.Publisher("/pointing/arm_joints", Marker, queue_size=10)
        self.pub_is_pointing = rospy.Publisher("/pointing/is_pointing", Bool, queue_size=10)

        self.is_pointing = False

        # Setup arrow and joint markers:
        # Setup arrow from shoulder to wrist
        self.arrow_shoulder = Marker()
        self.arrow_shoulder.header.frame_id = self.frame_id
        self.arrow_shoulder.header.stamp = rospy.Time.now()
        self.arrow_shoulder.ns = "pointing_arrow_shoulder_to_wrist"
        self.arrow_shoulder.id = 0
        self.arrow_shoulder.type = Marker.ARROW
        self.arrow_shoulder.action = Marker.DELETE
        # Set the start and end points of the arrow
        self.arrow_shoulder_start = Point(); self.arrow_shoulder_start.x = 0; self.arrow_shoulder_start.y = 0; self.arrow_shoulder_start.z = 0
        self.arrow_shoulder_end = Point();   self.arrow_shoulder_end.x = 0;   self.arrow_shoulder_end.y = 0;   self.arrow_shoulder_end.z = 1
        self.arrow_shoulder.points.append(self.arrow_shoulder_start); self.arrow_shoulder.points.append(self.arrow_shoulder_end)
        # Set the scale, color, orientation
        self.arrow_shoulder.scale.x = 0.03; self.arrow_shoulder.scale.y = 0.1; self.arrow_shoulder.scale.z = 0.1
        self.arrow_shoulder.color.a = 1.0; self.arrow_shoulder.color.r = 1.0; self.arrow_shoulder.color.g = 0.0; self.arrow_shoulder.color.b = 0.0
        self.arrow_shoulder.pose.orientation.x = 0.0; self.arrow_shoulder.pose.orientation.y = 0.0; self.arrow_shoulder.pose.orientation.z = 0.0; self.arrow_shoulder.pose.orientation.w = 1.0

        # Setup arrow from elbow to wrist
        self.arrow_elbow = Marker()
        self.arrow_elbow.header.frame_id = self.frame_id
        self.arrow_elbow.header.stamp = rospy.Time.now()
        self.arrow_elbow.ns = "pointing_arrow_elbow_to_wrist"
        self.arrow_elbow.id = 1
        self.arrow_elbow.type = Marker.ARROW
        self.arrow_elbow.action = Marker.DELETE
        # Set the start and end points of the arrow
        self.arrow_shoulder_end = Point();    self.arrow_shoulder_end.x = 0;    self.arrow_shoulder_end.y = 0;    self.arrow_shoulder_end.z = 0
        self.arrow_elbow_end = Point();      self.arrow_elbow_end.x = 0;      self.arrow_elbow_end.y = 0;      self.arrow_elbow_end.z = 1
        self.arrow_elbow.points.append(self.arrow_shoulder_end); self.arrow_elbow.points.append(self.arrow_elbow_end)
        # Set the scale, color, orientation
        self.arrow_elbow.scale.x = 0.03; self.arrow_elbow.scale.y = 0.1; self.arrow_elbow.scale.z = 0.1
        self.arrow_elbow.color.a = 1.0; self.arrow_elbow.color.r = 1.0; self.arrow_elbow.color.g = 0.0; self.arrow_elbow.color.b = 0.8
        self.arrow_elbow.pose.orientation.x = 0.0; self.arrow_elbow.pose.orientation.y = 0.0; self.arrow_elbow.pose.orientation.z = 0.0; self.arrow_elbow.pose.orientation.w = 1.0

        # Setup the line markers for the arm joints
        self.lines = Marker()
        self.lines.header.frame_id = self.frame_id
        self.lines.header.stamp = rospy.Time.now()
        self.lines.ns = "arm_joint_lines"
        self.lines.id = 2
        self.lines.type = Marker.LINE_STRIP
        self.lines.action = Marker.ADD
        # Set the scale, color, orientation
        self.lines.scale.x = 0.06
        self.lines.color.a = 1; self.lines.color.r = 0.0; self.lines.color.g = 1.0; self.lines.color.b = 1.0
        self.lines.pose.orientation.x = 0.0; self.lines.pose.orientation.y = 0.0; self.lines.pose.orientation.z = 0.0; self.lines.pose.orientation.w = 1.0

    def get_arm_keypoints(self, pose_results, rgb_image, depth_image, depth_compensation=True):
        # Extract the x, y coordinates of the arm keypoints from the detected pose keypoints
        # Get depth coordinate from depth image and apply depth compensation (optional)
        
        # Indices of the right arm joints in order: shoulder (12), elbow (14), wrist (16)
        # https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
        arm_keypoints_indices = [12, 14, 16]
        
        arm_keypoints = []

        # Get height and width of the RGB image
        image_height, image_width, _ = rgb_image.shape

        # Check if any keypoints are found
        if pose_results.pose_landmarks:
            # Get 2D arm keypoints, transform to pixel coordinates, get depth coordinate from depth image
            for i in arm_keypoints_indices:
                keypoint = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark(i).value]
                x = int(keypoint.x * image_width)
                y = int(keypoint.y * image_height)

                arm_keypoint = get_depth_coordinates(x, y, depth_image)
                arm_keypoints.append(arm_keypoint)
        
        # Check if the full right arm was detected (shoulder, elbow, wrist)
        is_full_arm = False
        if len(arm_keypoints) == 3:
            is_full_arm = True

            # Compensate the depth value of the joints by half of the thickness of a shoulder, elbow and wrist to get the depth in the middle of the arm
            if depth_compensation:
                arm_keypoints[0][2] += 0.04
                arm_keypoints[1][2] += 0.04
                arm_keypoints[2][2] += 0.04

        # Return the arm keypoints and if the full arm was detected
        return arm_keypoints, is_full_arm

    def detect_pointing_gesture(self, req):
        #rospy.loginfo("fonction callback detect_pointing_gesture")
        rgb = req.rgb
        depth = req.depth

        try:
            depth.encoding = self.depth_encoding
            depth_img = CvBridge().imgmsg_to_cv2(depth, self.depth_encoding)
            depth_img = depth_img/int(self.depth_scale)
        except CvBridgeError as e:
            rospy.logerr(f"Depth Image CvBridge Error: {e}")

        try:
            # Convert ROS Image message to OpenCV image with rgb8 encoding
            rgb_img = self.bridge.imgmsg_to_cv2(rgb, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(f"RGB Image CvBridge Error: {e}")  

        # Poining indicator that describes if a pointing motion is occuring at the moment
        self.is_pointing = False

        # Perform main pose detection
        pose_results = self.pose.process(rgb_img)

        # Find arm keypoints in pixel coordinates and get corresponding depth
        arm_keypoints_pc, is_full_arm = self.get_arm_keypoints(pose_results, rgb_img, depth_img)

        # Convert pixel coordinates and depth to camera coordinates
        arm_keypoints_cc = []
        for keypoint in arm_keypoints_pc:
            keypoint_cc = pixel_to_camera_coordinates(keypoint, self.camera_info)
            arm_keypoints_cc.append(keypoint_cc)

        # Initialize the arm_angle variable
        arm_angle = 0.0

        # If the full arm is detected perform angle calculation and set points to publish
        if is_full_arm:
            arm_angle = get_arm_angle(arm_keypoints_cc[0], arm_keypoints_cc[1], arm_keypoints_cc[2])
            
            # Set the is_pointing indicator to True if the angle is larger than the threshold
            # Show the published arrows only if currently pointing
            if arm_angle >= self.arm_angle_thresh:
                is_pointing = True
                self.arrow_shoulder.action = Marker.ADD
                self.arrow_elbow.action = Marker.ADD

            else:
                is_pointing = False
                self.arrow_shoulder.action = Marker.DELETE
                self.arrow_elbow.action = Marker.DELETE

            # Get desired arrow length (meters) from rosparam
            arrow_length_m = self.arrow_length
            
            # Set start and endpoints of the arrows (unscaled)
            arrow_shoulder_startpoint = arm_keypoints_cc[0]
            arrow_shoulder_endpoint = arm_keypoints_cc[2]
            arrow_elbow_startpoint = arm_keypoints_cc[1]
            arrow_elbow_endpoint = arm_keypoints_cc[2]

            # If the desired arrow length is not zero then scale the vector accordingly
            if arrow_length_m != 0:
                arrow_shoulder_startpoint, arrow_shoulder_endpoint = set_vector_length(arm_keypoints_cc[0], arm_keypoints_cc[2], arrow_length_m)
                arrow_elbow_startpoint, arrow_elbow_endpoint = set_vector_length(arm_keypoints_cc[1], arm_keypoints_cc[2], arrow_length_m)

            # Set the points of the arrows for publishing
            set_point(self.arrow_shoulder_start, arrow_shoulder_startpoint)
            set_point(self.arrow_shoulder_end, arrow_shoulder_endpoint)
            set_point(self.arrow_shoulder_end, arrow_elbow_startpoint)
            set_point(self.arrow_elbow_end, arrow_elbow_endpoint)

            # Set the points for the arm line marker
            self.lines.points = []
            for keypoint in arm_keypoints_cc:
                point = Point()
                set_point(point, keypoint)
                self.lines.points.append(point)

            # Publish the arrow and line markers and pointing indicator
            self.arrow_shoulder.header.stamp = rospy.Time.now()
            self.pub_arrow_shoulder.publish(self.arrow_shoulder)
            self.arrow_elbow.header.stamp = rospy.Time.now()
            self.pub_arrow_elbow.publish(self.arrow_elbow)
            self.lines.header.stamp = rospy.Time.now()
            self.pub_lines.publish(self.lines)
            self.pub_is_pointing.publish(is_pointing)

            shoulder_point = Point()
            set_point(shoulder_point, arm_keypoints_cc[0])
            elbow_point = Point()
            set_point(elbow_point, arm_keypoints_cc[1])
            wrist_point = Point()
            set_point(wrist_point, arm_keypoints_cc[2])

            res = estimate_pointing_gestureResponse()
            res.shoulder = shoulder_point
            res.elbow = elbow_point
            res.wrist = wrist_point
            return res
            #return shoulder_point, elbow_point, wrist_point  
        print("Full arm not detected")
        empty_point = Point()
        res = estimate_pointing_gestureResponse()
        res.shoulder = empty_point
        res.elbow = empty_point
        res.wrist = empty_point
        return res
        #return empty_point, empty_point, empty_point      

    

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame-id', type=str, default='camera_color_optical_frame')
    parser.add_argument('--color-topic', type=str, default='/camera/color/image_raw')
    parser.add_argument('--depth-topic', type=str, default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--arm-angle-thresh', type=float, default=140.0)
    parser.add_argument('--arrow-length', type=float, default=2.0)
    parser.add_argument('--model-complexity', type=int, default=1)
    parser.add_argument('--min-detection-confidence', type=float, default=0.9)
    parser.add_argument('--static-image-mode', type=bool, default=False)

    opt = parser.parse_args()
    return opt

if __name__ == "__main__":
    try:
        rospy.init_node('pointingdetector')
        opt = parse_opt()
        PointingDetector(**vars(opt))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


