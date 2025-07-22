#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations

def main():
    rospy.init_node('static_pose_publisher')

    # Publisher sur le topic /robot_pose_static
    pub = rospy.Publisher('/robot_pose_static', PoseStamped, queue_size=10)

    # Valeurs données
    translation = [0.021, 0.016, 0.752]
    quaternion = [0.001, -0.223, 0.004, 0.975]

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()  # utilisera use_sim_time si activé
        pose_msg.header.frame_id = "map"  # ou "odom" selon ton contexte

        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
