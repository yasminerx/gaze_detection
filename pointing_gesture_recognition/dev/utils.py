import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp
import argparse
from filterpy.kalman import KalmanFilter

def init_kalman():
    kf = KalmanFilter(dim_x=6, dim_z=3)
    dt = 0.1 # 10hz pour la boucle principale (à verifier)
    kf.F = np.array([[1, 0, 0, dt, 0, 0],
                        [0, 1, 0, 0, dt, 0],
                        [0, 0, 1, 0, 0, dt],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]])
    kf.H = np.array([[1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0]])
    kf.P *= 1000.0  # covariance matrix
    kf.R *= 0.1
    kf.Q *= 0.01
    return kf


def update_kalman(kf, z_measured):
    kf.predict()
    kf.update(z_measured)
    z_filtered = kf.x[:3].flatten()
    return kf, z_filtered / np.linalg.norm(z_filtered)
    
def set_point(point, keypoint):
    # Set the coordinates of a geometry_msgs point to the coordinates of a keypoint
    point.x = keypoint[0]
    point.y = keypoint[1]
    point.z = keypoint[2]

def get_depth_coordinates(x, y, depth_image, filtering=True, filter_width=3):
    # Get the depth coordinate at desired x, y coordinate. Returns list of x,y,z
    # Filtering the depth values in the neighborhood

    # Unfiltered z-coordinate (depth) in meters
    z = depth_image[y, x]
    
    if filtering:
        half = filter_width // 2

        height, width = depth_image.shape

        # Do not apply filtering if filter would be outside of image
        if (x - half) < 0 or (x + half) > (width - 1):
            filtering = False
        elif (y - half) < 0 or (y + half) > (height - 1):
            filtering = False

        # Return the filtered depth value of the pixel neighborhood
        if filtering:
            z_list = depth_image[y - half : y + half, x - half : x + half]
            z = np.median(z_list)

    if z <= 0:
        rospy.logwarn("Warning! Depth is zero!")

    return [x, y, z]


def get_head_coordinate_system(keypoints_cc):
    # with the keypoints in the camera coordinates, calculate the head position
    x_vector = np.array(keypoints_cc["outer_right"]) - np.array(keypoints_cc["outer_left"])
    x_vector = x_vector / np.linalg.norm(x_vector)
    y_vector = np.array((np.array(keypoints_cc["outer_right"]) + np.array(keypoints_cc["outer_left"]))/2 - np.array(keypoints_cc["chin"]))
    y_vector = y_vector / np.linalg.norm(y_vector)
    z_vector = np.cross(x_vector, y_vector)
    z_vector = z_vector / np.linalg.norm(z_vector)
    head_coordinate_system = np.array([x_vector, y_vector, z_vector])

    return head_coordinate_system


def get_eye_direction(keypoints_cc, head_coordinate_system):
    right_eye_center = np.array(keypoints_cc["right_eye_center"])
    left_eye_center = np.array(keypoints_cc["left_eye_center"])
    right_eye_pupil = np.array(keypoints_cc["right"])
    left_eye_pupil = np.array(keypoints_cc["left"])
    # upper_right = np.array(keypoints_cc["upper_right"])
    # upper_left = np.array(keypoints_cc["upper_left"])
    # lower_right = np.array(keypoints_cc["lower_right"])
    # lower_left = np.array(keypoints_cc["lower_left"])

    # eye direction vector
    # for the dx difference
    x_vector = head_coordinate_system[0]
    dx_right = (right_eye_center - right_eye_pupil)@ x_vector
    dx_left = (left_eye_center - left_eye_pupil)@ x_vector
    if dx_right > 1.02 :
        rospy.loginfo("regarde vers la droite")
    if dx_right < 0.99 :
        rospy.loginfo("regarde vers la gauche")
    dx = 1 + dx_right/dx_left

    # for the dy difference
    y_vector = head_coordinate_system[1]
    dy_right = (right_eye_center - right_eye_pupil)@ y_vector
    dy_left = (left_eye_center - left_eye_pupil)@ y_vector
    if dy_right > 1.02 :
        rospy.loginfo("regarde vers le haut")
    if dy_right < 0.99 :
        rospy.loginfo("regarde vers le bas")
    #dy = 1 + dy_right/dy_left
    dy = 0
    rospy.loginfo("dx: %s", dx)
    rospy.loginfo("dy: %s", dy)
    return dx*3, dy*2

    

def pixel_to_camera_coordinates(keypoint_pc, camera_info):
    # Get intrinsic camera parameters from camera info and transform x,y,z from pixel coordinates to camera coordinates
    fx = camera_info[0][0]
    fy = camera_info[1][1]
    cx = camera_info[0][2]
    cy = camera_info[1][2]

    # Transformation to camera coordinates
    x_c = (keypoint_pc[0] - cx) * keypoint_pc[2] / fx
    y_c = (keypoint_pc[1] - cy) * keypoint_pc[2] / fy

    return [x_c, y_c, keypoint_pc[2]]

def get_arm_angle(shoulder, elbow, wrist):
    # Return the angle between vectors shoulder-elbow and elbow-wrist
    # Angle can be used as a measure whether a person is pointing or not

    shoulder = np.array(shoulder)
    elbow = np.array(elbow)
    wrist = np.array(wrist)

    # Create the vectors
    v_shoulder_elbow = elbow - shoulder
    v_elbow_wrist = wrist - elbow

    # Calculate the angle in degrees
    angle_cos = np.dot(v_shoulder_elbow, v_elbow_wrist) / (np.linalg.norm(v_shoulder_elbow) * np.linalg.norm(v_elbow_wrist))
    angle_cos = np.clip(angle_cos, -1.0, 1.0)    
    angle_degrees = 180.0 - np.degrees(np.arccos(angle_cos))

    return angle_degrees


def set_vector_length(startpoint, endpoint, length_m):
    # Scales a vector consisting of a start and endpoint to the desired length

    startpoint_np = np.array(startpoint)
    endpoint_np = np.array(endpoint)
    
    direction = endpoint_np - startpoint_np
    current_length = np.linalg.norm(direction)
    
    # If the current length is zero return the initial points
    if current_length == 0:
        return startpoint, endpoint
    
    endpoint_np = startpoint + direction * (length_m / current_length)
    endpoint = endpoint_np.tolist()
    
    return startpoint, endpoint
