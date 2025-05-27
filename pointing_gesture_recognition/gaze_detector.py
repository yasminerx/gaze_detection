import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from object_detector_msgs.srv import estimate_pointing_gesture
from object_detector_msgs.srv import estimate_pointing_gestureResponse
from object_detector_msgs.srv import estimate_eye_position
from object_detector_msgs.srv import estimate_eye_positionResponse

import cv2
import numpy as np
import mediapipe as mp
import argparse
from one_euro_filter import OneEuroFilter

from utils import *

class GazeDetector:
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
        self.frame_id = rospy.get_param('/pose_estimator/color_frame_id')  
        self.color_topic = rospy.get_param('/pose_estimator/color_topic')  
        self.depth_topic = rospy.get_param('/pose_estimator/depth_topic')  

        # Setup CvBridge to convert ROS messages to OpenCV readable images
        self.bridge = CvBridge()

        # initialise the face detection model
        self.mp_face_mesh = mp.solutions.face_mesh

        # initalise the filters
        self.kf = init_kalman()
        t0 = rospy.Time.now().to_sec()
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

        self.service = rospy.Service("/detect_eyes", estimate_eye_position, self.detect_eye_gaze)
        rospy.loginfo("Service /detect_eyes started")

        self.pub_right_eye = rospy.Publisher("/pointing/right_eye", Marker, queue_size=10)
        self.pub_left_eye = rospy.Publisher("/pointing/left_eye", Marker, queue_size=10)
        self.pub_is_looking = rospy.Publisher("/pointing/is_looking", Bool, queue_size=10)
        self.pub_x_vector = rospy.Publisher("/pointing/x_vector", Marker, queue_size=10)
        self.pub_y_vector = rospy.Publisher("/pointing/y_vector", Marker, queue_size=10)
        self.pub_z_vector = rospy.Publisher("/pointing/z_vector", Marker, queue_size=10)
        self.pub_gaze = rospy.Publisher("/pointing/gaze", Marker, queue_size=10)
        self.is_looking = False

        scale = 0.01
        self.arrow_length = 0.5

        self.right_eye = Marker()
        self.right_eye.header.frame_id = self.frame_id
        self.right_eye.header.stamp = rospy.Time.now()
        self.right_eye.ns = "marker_right_eye"
        self.right_eye.id = 0
        self.right_eye.type = Marker.SPHERE
        self.right_eye.action = Marker.ADD
        self.right_eye.scale.x = scale
        self.right_eye.scale.y = scale
        self.right_eye.scale.z = scale
        self.right_eye.color.r = 0.0
        self.right_eye.color.g = 1.0
        self.right_eye.color.b = 0.0
        self.right_eye.color.a = 1.0

        self.left_eye = Marker()
        self.left_eye.header.frame_id = self.frame_id
        self.left_eye.header.stamp = rospy.Time.now()
        self.left_eye.ns = "marker_left_eye"
        self.left_eye.id = 1
        self.left_eye.type = Marker.SPHERE
        self.left_eye.action = Marker.ADD
        self.left_eye.scale.x = scale
        self.left_eye.scale.y = scale
        self.left_eye.scale.z = scale
        self.left_eye.color.r = 0.0
        self.left_eye.color.g = 1.0
        self.left_eye.color.b = 0.0
        self.left_eye.color.a = 1.0

        self.middle_right_eye = Marker()
        self.middle_right_eye.header.frame_id = self.frame_id
        self.middle_right_eye.header.stamp = rospy.Time.now()
        self.middle_right_eye.ns = "marker_middle_right_eye"
        self.middle_right_eye.id = 2
        self.middle_right_eye.type = Marker.SPHERE
        self.middle_right_eye.action = Marker.ADD
        self.middle_right_eye.scale.x = scale
        self.middle_right_eye.scale.y = scale
        self.middle_right_eye.scale.z = scale
        self.middle_right_eye.color.r = 0.0
        self.middle_right_eye.color.g = 0.0
        self.middle_right_eye.color.b = 1.0
        self.middle_right_eye.color.a = 1.0

        self.middle_left_eye = Marker()
        self.middle_left_eye.header.frame_id = self.frame_id
        self.middle_left_eye.header.stamp = rospy.Time.now()
        self.middle_left_eye.ns = "marker_middle_left_eye"
        self.middle_left_eye.id = 3
        self.middle_left_eye.type = Marker.SPHERE
        self.middle_left_eye.action = Marker.ADD
        self.middle_left_eye.scale.x = scale
        self.middle_left_eye.scale.y = scale
        self.middle_left_eye.scale.z = scale
        self.middle_left_eye.color.r = 0.0
        self.middle_left_eye.color.g = 0.0
        self.middle_left_eye.color.b = 1.0
        self.middle_left_eye.color.a = 1.0
        
        self.x_vector = Marker()
        self.x_vector.header.frame_id = self.frame_id
        self.x_vector.header.stamp = rospy.Time.now()
        self.x_vector.ns = "marker_x_vector"
        self.x_vector.id = 4
        self.x_vector.type = Marker.ARROW
        self.x_vector.action = Marker.ADD
        self.x_vector.scale.x = 0.01
        self.x_vector.scale.y = 0.02
        self.x_vector.scale.z = 0.02
        self.x_vector.color.r = 1.0
        self.x_vector.color.g = 0.0
        self.x_vector.color.b = 0.0
        self.x_vector.color.a = 1.0

        self.y_vector = Marker()
        self.y_vector.header.frame_id = self.frame_id
        self.y_vector.header.stamp = rospy.Time.now()
        self.y_vector.ns = "marker_y_vector"
        self.y_vector.id = 5
        self.y_vector.type = Marker.ARROW
        self.y_vector.action = Marker.ADD
        self.y_vector.scale.x = 0.01
        self.y_vector.scale.y = 0.02
        self.y_vector.scale.z = 0.02
        self.y_vector.color.r = 0.0
        self.y_vector.color.g = 1.0
        self.y_vector.color.b = 0.0
        self.y_vector.color.a = 1.0

        self.z_vector = Marker()
        self.z_vector.header.frame_id = self.frame_id
        self.z_vector.header.stamp = rospy.Time.now()
        self.z_vector.ns = "marker_z_vector"
        self.z_vector.id = 6
        self.z_vector.type = Marker.ARROW
        self.z_vector.action = Marker.ADD
        self.z_vector.scale.x = 0.01
        self.z_vector.scale.y = 0.02
        self.z_vector.scale.z = 0.02
        self.z_vector.color.r = 0.0
        self.z_vector.color.g = 0.0
        self.z_vector.color.b = 1.0
        self.z_vector.color.a = 1.0

        self.gaze = Marker()
        self.gaze.header.frame_id = self.frame_id
        self.gaze.header.stamp = rospy.Time.now()
        self.gaze.ns = "marker_gaze"
        self.gaze.id = 7
        self.gaze.type = Marker.ARROW
        self.gaze.action = Marker.ADD
        self.gaze.scale.x = 0.01
        self.gaze.scale.y = 0.02
        self.gaze.scale.z = 0.02
        self.gaze.color.r = 0.0
        self.gaze.color.g = 1.0
        self.gaze.color.b = 1.0
        self.gaze.color.a = 1.0

        
    def get_eye_keypoints(self, face_results, rgb_image, depth_image):
        ''' Extract the x, y coords of both eyes from the detected face keypoints.'''
        #rospy.loginfo("get_eye_keypoints started")
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

        # no need to compensate the depth value (?? to be checked)
        #rospy.loginfo("get_eye_keypoints finished")
        return gaze_keypoints, eye_detected
 
    def update_head_coordinate_system(self, gaze_keypoints_cc):
        head_coordinate_system = get_head_coordinate_system(gaze_keypoints_cc)
        # update z with the kalman filter
        self.kf, head_coordinate_system[2] = update_kalman(self.kf, head_coordinate_system[2])

        # plot with rviz the arrows
        origin = gaze_keypoints_cc['nose_bridge']
        end_x = origin + head_coordinate_system[0] * self.arrow_length
        end_y = origin + head_coordinate_system[1] * self.arrow_length
        end_z = origin + head_coordinate_system[2] * self.arrow_length
        point_origin = Point(origin[0], origin[1], origin[2])
        point_end_x = Point(end_x[0], end_x[1], end_x[2])
        point_end_y = Point(end_y[0], end_y[1], end_y[2])
        point_end_z = Point(end_z[0], end_z[1], end_z[2])
        self.x_vector.points = [point_origin, point_end_x]
        self.y_vector.points = [point_origin, point_end_y]
        self.z_vector.points = [point_origin, point_end_z]
        self.x_vector.action = Marker.ADD
        self.y_vector.action = Marker.ADD
        self.z_vector.action = Marker.ADD
        self.x_vector.header.stamp = rospy.Time.now()
        self.y_vector.header.stamp = rospy.Time.now()
        self.z_vector.header.stamp = rospy.Time.now()
        self.pub_x_vector.publish(self.x_vector)
        self.pub_y_vector.publish(self.y_vector)
        self.pub_z_vector.publish(self.z_vector)

        return head_coordinate_system


    def update_gaze_marker(self, dx, dy, head_coordinate_system, gaze_keypoints_cc):
        gaze_vector = head_coordinate_system[0]*dx + head_coordinate_system[1]*dy + head_coordinate_system[2]
        origin = gaze_keypoints_cc['nose_bridge']
        end_gaze = origin + gaze_vector * self.arrow_length
        point_origin = Point(origin[0], origin[1], origin[2])
        point_end_gaze = Point(end_gaze[0], end_gaze[1], end_gaze[2])
        self.gaze.points = [point_origin, point_end_gaze]
        self.gaze.action = Marker.ADD
        self.gaze.header.stamp = rospy.Time.now()
        self.pub_gaze.publish(self.gaze)
        


    def detect_eye_gaze(self, req):
        rospy.loginfo("detect eye gaze started")
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

        # is there a face in the imge ?
        self.eye_detected = False

        # main face detection 
        face_results = self.face.process(rgb_img)

        gaze_keypoints, eye_detected = self.get_eye_keypoints(face_results, rgb_img, depth_img)


        if self.eye_detected :
            # convert the pixel coordinates to camera coordinates
            gaze_keypoints_cc = {}
            for side, i in gaze_keypoints.items():
                gaze_keypoints_cc[side] = pixel_to_camera_coordinates(i, self.camera_info)

            
            head_coordinate_system = self.update_head_coordinate_system(gaze_keypoints_cc)
            dx ,dy = get_eye_direction(gaze_keypoints_cc, head_coordinate_system)

            # apply the one euro filter to the dx and dy values
            t = rospy.Time.now().to_sec()
            dx = self.dx_filter(t, dx)
            dy = self.dy_filter(t, dy)

            self.update_gaze_marker(dx, dy, head_coordinate_system, gaze_keypoints_cc)
            
            if eye_detected['right']:
                #gaze_keypoints_cc['right'] = pixel_to_camera_coordinates(gaze_keypoints['right'], self.camera_info)
                self.is_looking = True
                self.right_eye.action = Marker.ADD
                self.right_eye.header.stamp = rospy.Time.now()
                self.right_eye.pose.position.x = gaze_keypoints_cc['right'][0]
                self.right_eye.pose.position.y = gaze_keypoints_cc['right'][1]
                self.right_eye.pose.position.z = gaze_keypoints_cc['right'][2]
                self.pub_right_eye.publish(self.right_eye)

                self.middle_right_eye.action = Marker.ADD
                self.middle_right_eye.header.stamp = rospy.Time.now()
                self.middle_right_eye.pose.position.x = (gaze_keypoints_cc['inner_right'][0]+gaze_keypoints_cc['outer_right'][0])/2
                self.middle_right_eye.pose.position.y = (gaze_keypoints_cc['inner_right'][1]+gaze_keypoints_cc['outer_right'][1])/2
                self.middle_right_eye.pose.position.z = (gaze_keypoints_cc['inner_right'][2]+gaze_keypoints_cc['outer_right'][2])/2
                self.pub_right_eye.publish(self.middle_right_eye)
            else:
                self.right_eye.action = Marker.DELETE
                self.pub_right_eye.publish(self.right_eye)

                self.middle_right_eye.action = Marker.DELETE
                self.pub_right_eye.publish(self.middle_right_eye)
            
            if eye_detected['left']:
                #gaze_keypoints_cc['left'] = pixel_to_camera_coordinates(gaze_keypoints['left'], self.camera_info)
                self.is_looking = True
                self.left_eye.action = Marker.ADD
                self.left_eye.header.stamp = rospy.Time.now()
                self.left_eye.pose.position.x = gaze_keypoints_cc['left'][0]
                self.left_eye.pose.position.y = gaze_keypoints_cc['left'][1]
                self.left_eye.pose.position.z = gaze_keypoints_cc['left'][2]
                self.pub_left_eye.publish(self.left_eye)

                self.middle_left_eye.action = Marker.ADD
                self.middle_left_eye.header.stamp = rospy.Time.now()
                self.middle_left_eye.pose.position.x = (gaze_keypoints_cc['inner_left'][0]+gaze_keypoints_cc['outer_left'][0])/2
                self.middle_left_eye.pose.position.y = (gaze_keypoints_cc['inner_left'][1]+gaze_keypoints_cc['outer_left'][1])/2
                self.middle_left_eye.pose.position.z = (gaze_keypoints_cc['inner_left'][2]+gaze_keypoints_cc['outer_left'][2])/2
                self.pub_left_eye.publish(self.middle_left_eye)
            else:
                self.left_eye.action = Marker.DELETE
                self.pub_left_eye.publish(self.left_eye)

                self.middle_left_eye.action = Marker.DELETE
                self.pub_left_eye.publish(self.middle_left_eye)

            res = estimate_eye_positionResponse()
            right_eye_point = Point()
            set_point(right_eye_point, gaze_keypoints_cc['right'])
            left_eye_point = Point()
            set_point(left_eye_point, gaze_keypoints_cc['left'])
            res.right_eye = right_eye_point
            res.left_eye = left_eye_point
            return res
        else :
            return None

    

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
        rospy.init_node('gazedetector')
        opt = parse_opt()
        GazeDetector(**vars(opt))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


