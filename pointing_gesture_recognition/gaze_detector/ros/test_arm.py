#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import estimate_pointing_gesture
from sensor_msgs.msg import Image

class ArmCalculator:
    def __init__(self):
        print("ArmCalculator initialized")

    def detect_arm_position(self, rgb, depth):
        rospy.wait_for_service('pointing_callback')
        try:
            detect_arm_position_service = rospy.ServiceProxy('pointing_callback', estimate_pointing_gesture)
            #print("service ok, on appelle le service")
            response = detect_arm_position_service(rgb, depth)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("calculate_pointing")
    try:
        arm_calculator = ArmCalculator()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print("waiting for rgb")
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            #print("waiting for depth")
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            print('Perform arm Detection...')
            arm_positions = arm_calculator.detect_arm_position(rgb, depth)
            print("arm_positions: ", arm_positions)
            print('... received arm.')

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

