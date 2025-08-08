import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from object_detector_msgs.srv import estimate_pointing_gesture
from object_detector_msgs.srv import estimate_pointing_gestureResponse

from arm_lib.detector import PointingDetector
from arm_lib.utils import set_point
import numpy as np


class PointingRosNode :
    def __init__(self, arm_angle_thresh=120.0,
            arrow_length=2.0):
        self.camera_info = np.asarray(rospy.get_param('/pose_estimator/intrinsics'))
        self.depth_encoding = rospy.get_param('/pose_estimator/depth_encoding')
        self.depth_scale = rospy.get_param('/pose_estimator/depth_scale')        
        self.frame_id = rospy.get_param('/pose_estimator/color_frame_id')  
        self.color_topic = rospy.get_param('/pose_estimator/color_topic')  
        self.depth_topic = rospy.get_param('/pose_estimator/depth_topic')  

        self.arm_angle_thresh = arm_angle_thresh
        self.arrow_length = arrow_length

        # Setup CvBridge to convert ROS messages to OpenCV readable images
        self.bridge = CvBridge()
        t0 = rospy.Time.now().to_sec()
        self.detector = PointingDetector(t0, self.camera_info, self.depth_encoding, self.depth_scale)

        # ros services :
        self.service = rospy.Service("/pointing_callback",
        estimate_pointing_gesture, self.pointing_callback)
        rospy.loginfo("Service /pointing_callback started")


        self.pub_arrow_shoulder = rospy.Publisher("/pointing/shoulder_to_wrist", Marker, queue_size=10)
        self.pub_arrow_elbow = rospy.Publisher("/pointing/elbow_to_wrist", Marker, queue_size=10)
        self.pub_lines = rospy.Publisher("/pointing/arm_joints", Marker, queue_size=10)
        self.pub_is_pointing = rospy.Publisher("/pointing/is_pointing", Bool, queue_size=10)

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
        print("markers updated")

    def update_arm_markers(self, arm_keypoints_cc, arm_detected):
        arm_angle = 0.0
        if arm_detected['right_shoulder'] and arm_detected['right_elbow'] and arm_detected['right_wrist']:
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
            arrow_shoulder_startpoint = arm_keypoints_cc['right_shoulder']
            arrow_shoulder_endpoint = arm_keypoints_cc['right_wrist']
            arrow_elbow_startpoint = arm_keypoints_cc['right_elbow']
            arrow_elbow_endpoint = arm_keypoints_cc['right_wrist']

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
        else :
            empty_point = Point()
            res = estimate_pointing_gestureResponse()
            res.shoulder = empty_point
            res.elbow = empty_point
            res.wrist = empty_point
            return res



    def pointing_callback(self, req):
        rospy.loginfo("Received pointing request")
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
            arm_keypoints_cc, arm_detected = self.detector.detect_pointing_gesture(rgb_img, depth_img, rospy.Time.now().to_sec())

            res = self.update_arm_markers(arm_keypoints_cc, arm_detected)
            return res

        except Exception as e:
            rospy.logerr(f"Error updating markers: {e}")

        return None
    
if __name__ == "__main__":
    try:
        rospy.init_node('pointing_node_detector')
        PointingRosNode()
        rospy.loginfo("Pointing detector node initialized")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
