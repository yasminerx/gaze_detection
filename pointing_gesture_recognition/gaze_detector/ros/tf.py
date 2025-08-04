#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg



def main():
    rospy.init_node('static_tf_map_to_camera')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transform = geometry_msgs.msg.TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "head_rgbd_sensor_rgb_frame"

    # Replace with your actual values
    static_transform.transform.translation.x = 0.021
    static_transform.transform.translation.y = 0.016
    static_transform.transform.translation.z = 0.752

    static_transform.transform.rotation.x = 0.001
    static_transform.transform.rotation.y = -0.223
    static_transform.transform.rotation.z = 0.004
    static_transform.transform.rotation.w = 0.975

    broadcaster.sendTransform(static_transform)
    rospy.spin()

if __name__ == '__main__':
    main()
