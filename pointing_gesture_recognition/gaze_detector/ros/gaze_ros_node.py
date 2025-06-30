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

from gaze_lib.detector import GazeDetector
import numpy as np


class GazeRosNode :
    def __init__(self):
        self.camera_info = np.asarray(rospy.get_param('/pose_estimator/intrinsics'))
        self.depth_encoding = rospy.get_param('/pose_estimator/depth_encoding')
        self.depth_scale = rospy.get_param('/pose_estimator/depth_scale')        
        self.frame_id = rospy.get_param('/pose_estimator/color_frame_id')  
        self.color_topic = rospy.get_param('/pose_estimator/color_topic')  
        self.depth_topic = rospy.get_param('/pose_estimator/depth_topic')  

        # Setup CvBridge to convert ROS messages to OpenCV readable images
        self.bridge = CvBridge()
        t0 = rospy.Time.now().to_sec()
        self.detector = GazeDetector(t0, self.camera_info, self.depth_encoding, self.depth_scale)

        # ros services :
        self.service = rospy.Service("/gaze_callback", estimate_eye_position, self.gaze_callback)
        rospy.loginfo("Service /gaze_callback started")

        self.pub_right_eye = rospy.Publisher("/pointing/right_eye", Marker, queue_size=10)
        self.pub_left_eye = rospy.Publisher("/pointing/left_eye", Marker, queue_size=10)
        self.pub_is_looking = rospy.Publisher("/pointing/is_looking", Bool, queue_size=10)
        self.pub_x_vector = rospy.Publisher("/pointing/x_vector", Marker, queue_size=10)
        self.pub_y_vector = rospy.Publisher("/pointing/y_vector", Marker, queue_size=10)
        self.pub_z_vector = rospy.Publisher("/pointing/z_vector", Marker, queue_size=10)
        self.pub_gaze = rospy.Publisher("/pointing/gaze", Marker, queue_size=10)

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


    def update_markers(self, dx, dy, head_coordinate_system, gaze_keypoints_cc):
        gaze_vector = head_coordinate_system[0]*dx + head_coordinate_system[1]*dy + head_coordinate_system[2]
        origin = gaze_keypoints_cc['nose_bridge']
        end_gaze = origin + gaze_vector * self.arrow_length
        point_origin = Point(origin[0], origin[1], origin[2])
        point_end_gaze = Point(end_gaze[0], end_gaze[1], end_gaze[2])
        self.gaze.points = [point_origin, point_end_gaze]
        self.gaze.action = Marker.ADD
        self.gaze.header.stamp = rospy.Time.now()
        self.pub_gaze.publish(self.gaze)

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

    def update_eye_markers(self, gaze_keypoints_cc, eye_detected):
        if eye_detected['right']:
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


    def gaze_callback(self, req):
        rospy.loginfo("Received gaze request")
        rgb = req.rgb
        depth = req.depth

        try:
            depth.encoding = self.depth_encoding
            depth_img = CvBridge().imgmsg_to_cv2(depth, self.depth_encoding)
            depth_img = depth_img/int(self.depth_scale)
        except CvBridgeError as e:
            print(f"Depth Image CvBridge Error: {e}")

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb, "rgb8")
        except CvBridgeError as e:
            print(f"RGB Image CvBridge Error: {e}")


        try :
            detected_gaze_keypoints_cc, eye_detected, head_coordinate_system, dx, dy = self.detector.detect_eye_gaze(rgb_img, depth_img, rospy.Time.now().to_sec())
            self.update_markers(dx, dy, head_coordinate_system, detected_gaze_keypoints_cc)
            self.update_eye_markers(detected_gaze_keypoints_cc, eye_detected)
        except Exception as e:
            rospy.logerr(f"Error updating markers: {e}")


if __name__ == "__main__":
    try:
        rospy.init_node('gazenodedetector')
        GazeRosNode()
        rospy.loginfo("Gaze detector node initialized")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
