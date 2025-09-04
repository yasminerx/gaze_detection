import rospy
from gaze_lib.filters import OneEuroFilter, KalmanWrapper
from visualization_msgs.msg import Marker


class FilterGaze:
    def __init__(self):
        self.frame_id = rospy.get_param('/pose_estimator/color_frame_id')  

        self.kw = KalmanWrapper()
        t0 = rospy.Time.now().to_sec()
        self.dx_filter = OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.007)
        self.dy_filter = OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.007)

        self.Service("pointing/gaze", Marker, self.filter_gaze)

        self.pub_filtered_gaze = rospy.Publisher("pointing/filtered_gaze", Marker, queue_size=10)

        self.filtered_gaze = Marker()
        self.filtered_gaze.header.frame_id = self.frame_id
        self.filtered_gaze.header.stamp = rospy.Time.now()
        self.filtered_gaze.ns = "marker_gaze"
        self.filtered_gaze.id = 7
        self.filtered_gaze.type = Marker.ARROW
        self.filtered_gaze.action = Marker.ADD
        self.filtered_gaze.scale.x = 0.01
        self.filtered_gaze.scale.y = 0.02
        self.filtered_gaze.scale.z = 0.02
        self.filtered_gaze.color.r = 0.0
        self.filtered_gaze.color.g = 1.0
        self.filtered_gaze.color.b = 1.0
        self.filtered_gaze.color.a = 1.0
        self.filtered_gaze.pose.orientation.w = 1.0  # Quaternion identité
        #initialize the markers


    