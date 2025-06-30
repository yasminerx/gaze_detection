import cv2
import numpy as np
import mediapipe as mp
from .utils import *
from .filters import OneEuroFilter, KalmanWrapper



class GazeDetector:
    def __init__(self, t0, camera_info, depth_encoding, depth_scale, model_complexity=1, static_image_mode=False):
        # initialise the face detection model
        self.mp_face_mesh = mp.solutions.face_mesh
        self.depth_encoding = depth_encoding
        self.depth_scale = depth_scale
        self.camera_info = camera_info

        # initalise the filters
        self.kw = KalmanWrapper()
        # min_cutoff : sensibilité aux changements rapides (haut = moins de lissage)
        # beta : réactivité aux changements rapides (haut = plus de variations acceptées)
        self.dx_filter = OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.007)
        self.dy_filter = OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.007)

        # Pose detection model complexity (0, 1, 2) can be set as rosparam
        model_complexity = model_complexity
        if model_complexity not in [0, 1, 2]:
            model_complexity = 1
        print(f"Model complexity = {model_complexity}")
        
        
        # setup face detection model
        self.face = self.mp_face_mesh.FaceMesh(static_image_mode=static_image_mode, 
        refine_landmarks=True,
        max_num_faces=1)
        self.eye_detected = False

        

        
    def get_eye_keypoints(self, face_results, rgb_image, depth_image):
        ''' Extract the x, y coords of both eyes from the detected face keypoints.'''
        gaze_keypoints = {'right': None, 'left': None}
        eye_detected = {'right': False, 'left': False}
        image_height, image_width, _ = rgb_image.shape

        #indices of the right and left pupil (in order)
        # https://github.com/tensorflow/tfjs-models/blob/master/face-landmarks-detection/mesh_map.jpg

        pupil_indices = {"right": 468, 
                         "left": 473, 
                         "inner_right":362, 
                         "outer_right": 263, 
                         "inner_left":133, 
                         "outer_left": 33, 
                         "nose_bridge":168, 
                         "nose": 1,
                         "chin": 200,
                         "upper_right": 386,
                         "lower_right": 374,
                         "upper_left": 159,
                         "lower_left": 145,}
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                # get the x, y coordinates of the pupils
                for side, i in pupil_indices.items():
                    landmark = face_landmarks.landmark[i]
                    x = int(landmark.x * image_width)
                    y = int(landmark.y * image_height)
                    gaze_keypoint = get_depth_coordinates(x, y, depth_image)
                    gaze_keypoints[side] = gaze_keypoint
                    eye_detected[side] = True
            self.eye_detected = True
            gaze_keypoints['right_eye_center'] = (np.array(gaze_keypoints["inner_right"]) + np.array(gaze_keypoints['outer_right']))/2
            gaze_keypoints['left_eye_center'] = (np.array(gaze_keypoints['inner_left']) + np.array(gaze_keypoints['outer_left']))/2

            print("Face detected in the image.")

        else:
            print("No face detected in the image.")
        # no need to compensate the depth value (?? to be checked)
        #rospy.loginfo("get_eye_keypoints finished")
        return gaze_keypoints, eye_detected
 
    def update_head_coordinate_system(self, gaze_keypoints_cc):
        head_coordinate_system = get_head_coordinate_system(gaze_keypoints_cc)
        # update z with the kalman filter
        try :
            kf, head_coordinate_system[2] = self.kw.update(head_coordinate_system[2])
            self.kw.kf = kf
        except Exception as e:
            print(f"Error updating head coordinate system with Kalman filter: {e}")
            pass
        return head_coordinate_system


    def detect_eye_gaze(self, rgb_img, depth_img, t):
        print("detect eye gaze started")

        # is there a face in the imge ?
        self.eye_detected = False

        # main face detection 
        face_results = self.face.process(rgb_img)
        gaze_keypoints, eye_detected = self.get_eye_keypoints(face_results, rgb_img, depth_img)
        print("gaze keypoints", gaze_keypoints)
        if self.eye_detected :
            # convert the pixel coordinates to camera coordinates
            gaze_keypoints_cc = {}
            for side, i in gaze_keypoints.items():
                gaze_keypoints_cc[side] = pixel_to_camera_coordinates(i, self.camera_info)

            try:
                head_coordinate_system = self.update_head_coordinate_system(gaze_keypoints_cc)
                dx, dy = get_eye_direction(gaze_keypoints_cc, head_coordinate_system)

                # apply the one euro filter to the dx and dy values
                dx = self.dx_filter(t, dx)
                dy = self.dy_filter(t, dy)
                print(f"dx: {dx}, dy: {dy}")

            except Exception as e:
                print(f"Error updating head coordinate system: {e}")
                head_coordinate_system = None
                dx, dy = 0.0, 0.0 

            return gaze_keypoints_cc, eye_detected, head_coordinate_system, dx, dy
        else:
            return None, False, None, 0.0, 0.0


