import cv2
import numpy as np
import mediapipe as mp
from .utils import *
from .filters import OneEuroFilter, KalmanWrapper
import matplotlib.pyplot as plt



class PointingDetector:
    def __init__(self, t0, camera_info, depth_encoding, depth_scale, model_complexity=1, static_image_mode=False):
        self.depth_encoding = depth_encoding
        self.depth_scale = depth_scale
        self.camera_info = camera_info

        # Initialize mediapipe pose class
        self.mp_pose = mp.solutions.pose

        # Pose detection model complexity (0, 1, 2) can be set as rosparam
        model_complexity = model_complexity
        if model_complexity not in [0, 1, 2]:
            model_complexity = 1
        print(f"Model complexity = {model_complexity}")

        # Setup pose detection model
        self.pose = self.mp_pose.Pose(static_image_mode=static_image_mode, min_detection_confidence=0.9, model_complexity=model_complexity)

        self.is_pointing = False

        

        
    def get_arm_keypoints(self, pose_results, rgb_image, depth_image):
        ''' Extract the x, y coords of both the arm keypoints from the detected body landmarks.'''
        arm_keypoints = {}
        arm_detected = {}
        image_height, image_width, _ = rgb_image.shape

        # Indices of the right arm joints in order: shoulder (12), elbow (14), wrist (16)
        # https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

        arm_indices = {"right_shoulder": 12,
                         "right_elbow": 14, 
                         "right_wrist": 16 
                        #  ,"left_shoulder": 11,
                        #  "left_elbow": 13, 
                        #  "left_wrist": 15
        }

        if pose_results.pose_landmarks:
            for pose_landmarks in pose_results.multi_pose_landmarks:
                # get the x, y coordinates of the arm keypoints
                for arm_joint, i in arm_indices.items():
                    keypoint = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark(i).value]
                    x = int(keypoint.x * image_width)
                    y = int(keypoint.y * image_height)

                    arm_keypoint = get_depth_coordinates(x, y, depth_image)
                    arm_keypoints[arm_joint] = arm_keypoint
                    arm_detected[arm_joint] = True

        # check if entire arm is detected
        if (arm_keypoints["right_shoulder"] is not None and 
            arm_keypoints["right_elbow"] is not None and 
            arm_keypoints["right_wrist"] is not None):
            self.is_pointing = True
            print("Right arm keypoints detected in the image.")
        else:
            self.is_pointing = False
            print("No right arm detected in the image.")
            
        # no need to compensate the depth value (?? to be checked)
        #rospy.loginfo("get_eye_keypoints finished")
        return arm_keypoints, arm_detected
 

    def detect_pointing_gesture(self, rgb_img, depth_img, t):
        print("detect pointing gesture started")

        self.arm_detected = False

        # main arm detection
        arm_results = self.pose.process(rgb_img)
        arm_keypoints, arm_detected = self.get_arm_keypoints(arm_results, rgb_img, depth_img)
        # print("arm keypoints", arm_keypoints)

        if self.arm_detected:
            # convert the pixel coordinates to camera coordinates
            arm_keypoints_cc = {}
            for side, i in arm_keypoints.items():
                arm_keypoints_cc[side] = pixel_to_camera_coordinates(i, self.camera_info)

            print("returning values")
            return arm_keypoints_cc, arm_detected
        else:
            return None, False


