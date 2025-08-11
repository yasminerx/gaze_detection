#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import estimate_eye_position
from sensor_msgs.msg import Image

class GazeCalculator:
    def __init__(self):
        print("GazeCalculator initialized")

    def detect_eye_position(self, rgb, depth):
        rospy.wait_for_service('detect_eyes')
        try:
            detect_eye_position_service = rospy.ServiceProxy('detect_eyes', estimate_eye_position)
            #print("service ok, on appelle le service")
            response = detect_eye_position_service(rgb, depth)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("calculate_gaze")
    try:
        gaze_calculator = GazeCalculator()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print("waiting for rgb")
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            #print("waiting for depth")
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            print('Perform gaze Detection...')
            iris_positions = gaze_calculator.detect_eye_position(rgb, depth)
            print("iris_positions: ", iris_positions)
            print('... received gaze.')

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

