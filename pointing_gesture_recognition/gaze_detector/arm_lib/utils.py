import numpy as np
import mediapipe as mp
try:
    from geometry_msgs.msg import Point
except ImportError:
    pass

    
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
        print("Invalid depth value at pixel coordinates ({}, {}): {}".format(x, y, z))

    return [x, y, z]


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
