#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import estimate_pointing_gesture
from sensor_msgs.msg import Image

class PoseCalculator:
    def __init__(self):
        print("PoseCalculator initialized")

    def detect_pointing_gesture(self, rgb, depth):
        rospy.wait_for_service('detect_pointing_gesture')
        try:
            detect_pointing_gesture_service = rospy.ServiceProxy('detect_pointing_gesture', estimate_pointing_gesture)
            print("service ok, on appelle le service")
            response = detect_pointing_gesture_service(rgb, depth)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("calculate_poses")
    try:
        pose_calculator = PoseCalculator()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # print("waiting for rgb")
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            # print("waiting for depth")
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            print('Perform Pointing Detection...')
            joint_positions = pose_calculator.detect_pointing_gesture(rgb, depth)
            print("joint_positions: ", joint_positions)
            print('... received pointing gesture.')

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

