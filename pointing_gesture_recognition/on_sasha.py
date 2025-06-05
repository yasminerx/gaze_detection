#! /usr/bin/env python3
from math import pi
import rospy
import hsrb_interface
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped, PoseArray
import tf
from tmc_vision_msgs.msg import MarkerArray
import numpy as np
import tf2_ros
import subprocess
import os
import signal
import yaml

class FindMarker:
    def __init__(self):
        rospy.init_node("findmarker")

        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')
        self.head_pan_low = -3.839
        self.head_pan_high = 1.745
        self.head_pan_step = np.pi/10

        self.accept_markers = False

        self.id_15 = False
        self.id_16 = False
        self.id_17 = False
        self.id_719 = False
        self.

        self.transformer = tf.TransformListener(True, rospy.Duration(10))
        self.markers_detected = False
        self.marker_sub = rospy.Subscriber("/ar_marker", MarkerArray, self.marker_cb)
        self.pose_array_pub = rospy.Publisher("/found_markers_array", PoseArray, queue_size=10)

        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "map"
        self.pose_array.header.stamp = rospy.Time.now()

        self.pose_array_pub.publish(self.pose_array)


        self.pose_15 = PoseStamped()
        self.pose_16 = PoseStamped()
        self.pose_17 = PoseStamped()
        self.pose_719 = PoseStamped()
        self.pose_717 = PoseStamped()

        self.tts = self.robot.get('default_tts')
        self.tts.language = self.tts.ENGLISH
        rospy.sleep(3)

    def get_head_pan_level(self):
        self.marker_id = None
        rospy.loginfo('Executing state get_head_pan_level')
        self.whole_body.move_to_go()
        while not rospy.is_shutdown() :
            head_pan_level = self.whole_body.joint_positions['head_pan_joint']
            rospy.loginfo(f"Current head pan level: {head_pan_level}")
            rospy.sleep(0.5)
        return self.marker_id

    def get_marker_id(self):
        rospy.loginfo('Executing state find_marker')
        # initial posture of the robot
        self.whole_body.move_to_go()
        rospy.sleep(1.0)

        # once sasha is in the right position, we can start looking for markers
        self.accept_markers = True
        self.marker_id = None
        rospy.loginfo(f"initial markers : {self.marker_id}")
    
        head_pan_level = self.whole_body.joint_positions['head_pan_joint']
        # it's supposed to be around 0 ?

        directions = [-1, 1]                

        for direction in directions: 
            while not rospy.is_shutdown() and not self.markers_detected:    # rotate once in positive direction and once in the other direction
                rospy.loginfo("Turning head")
                head_pan_level += self.head_pan_step * direction
                head_pan_level = np.clip(head_pan_level, -3.839, 1.745)
                print(f"{head_pan_level=}")
                # maybe use hsrb/command_velocity from RQT
                self.whole_body.move_to_joint_positions({'head_pan_joint': head_pan_level})
                rospy.sleep(0.8)
                # if the head pan level is too high/low : go the other direction
                if head_pan_level >= self.head_pan_high - 0.2:
                    direction = -1
                    self.whole_body.move_to_joint_positions({'head_tilt_joint': 0})
                    rospy.sleep(0.5)
                elif head_pan_level <= self.head_pan_low + 0.2:
                    direction = 1
                    self.whole_body.move_to_joint_positions({'head_tilt_joint': 0})
                    rospy.sleep(0.5)
                
                if self.id_15 and self.id_16 and self.id_17 and self.id_719 and self.id_717:
                    self.markers_detected = True
                    self.accept_markers = False
                    rospy.sleep(0.5)
                    rospy.loginfo("tous les markers trouvés")
                    self.save_markers_to_yaml()

        self.pose_array_pub.publish(self.pose_array)
        self.whole_body.move_to_go()        
        return self.marker_id


    def marker_cb(self, markers):
        rospy.loginfo("marker callback :")
        if not self.accept_markers:
            rospy.loginfo("waiting for sasha to be in the right position")
            return
        #rospy.loginfo(f"{markers}")
        if len(markers.frame_ids) > 0:
            # Found a marker, taking the first marker
            for frame_id in markers.frame_ids:
                id = int(frame_id.split("/")[1])
                stamp = markers.header.stamp
                rospy.loginfo(f"{id}")
                rospy.loginfo(f"{frame_id}")

                if id == 15 and not self.id_15:
                    self.id_15 = True
                    rospy.loginfo("id = 15")
                    pose = self.transform_marker(str(id), stamp)
                    if pose is not None:
                        self.pose_15.pose = pose
                        self.pose_15.header.frame_id = "map"
                        self.pose_15.header.stamp = stamp

                if id == 16 and not self.id_16:
                    self.id_16 = True
                    rospy.loginfo("id = 16")
                    pose = self.transform_marker(str(id), stamp)
                    if pose is not None:
                        self.pose_16.pose = pose
                        self.pose_16.header.frame_id = "map"
                        self.pose_16.header.stamp = stamp
                    
                if id == 17 and not self.id_17:
                    self.id_17 = True
                    rospy.loginfo("id = 17")
                    pose = self.transform_marker(str(id), stamp)
                    if pose is not None:
                        self.pose_17.pose = pose
                        self.pose_17.header.frame_id = "map"
                        self.pose_17.header.stamp = stamp

                if id == 719 and not self.id_719:
                    self.id_719 = True
                    rospy.loginfo("id = 719")
                    pose = self.transform_marker(str(id), stamp)
                    if pose is not None:
                        self.pose_719.pose = pose
                        self.pose_719.header.frame_id = "map"
                        self.pose_719.header.stamp = stamp
                
                if id == 717 and not self.id_717:
                    self.id_717 = True
                    rospy.loginfo("id = 717")
                    pose = self.transform_marker(str(id), stamp)
                    if pose is not None:
                        self.pose_717.pose = pose
                        self.pose_717.header.frame_id = "map"
                        self.pose_717.header.stamp = stamp
                


    def transform_pose(self, target_frame, source_frame, pose_stamp, time_stamp): #transform
            # Copyright (C) Elisabeth F
            # transform pose from source to target frame
            source_pose = PoseStamped()
            source_pose.header.frame_id = source_frame
            source_pose.header.stamp = time_stamp
            source_pose.pose = pose_stamp.pose

            self.transformer.waitForTransform(target_frame, source_frame, time_stamp, rospy.Duration(4.0))
            try:
                target_pose = self.transformer.transformPose(target_frame, source_pose)
            except tf.Exception as e:
                rospy.logerr(f"Can't transform {target_frame} and {source_frame}: {e}")
                return 0
            else:
                rospy.loginfo(f"Can transform {target_frame} and {source_frame}")
                return target_pose
            
    def transform_marker(self, id, stamp):
        # Pose in map
        pose_end = PoseStamped()
        # Pose in marker
        pose_mid = PoseStamped()
        # Pose in robot (origin)
        pose_start = PoseStamped()
        pose_start.pose.position.x=0
        pose_start.pose.position.y=0
        pose_start.pose.position.z=0
        pose_start.pose.orientation.x=0
        pose_start.pose.orientation.y=0
        pose_start.pose.orientation.z=0
        pose_start.pose.orientation.w=1.0
        pose_start.header.frame_id = f"ar_marker/{id}"
        try:
            pose_mid = self.transform_pose("map", f"ar_marker/{id}", pose_start, stamp)
            if pose_mid == 0:
                return None

            rospy.loginfo(f"{pose_mid}")
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Error while transforming to base: {e}")
            return None

        self.pose_array.poses.append(pose_mid.pose)

        rospy.loginfo("Published pose")
        return pose_mid.pose


    def save_markers_to_yaml(self, filename="marker_positions.yaml"):
        data = {
            "marker_15": {
                "position": {
                    "x": float(self.pose_15.pose.position.x),
                    "y": float(self.pose_15.pose.position.y),
                    "z": float(self.pose_15.pose.position.z)
                }
            },
            "marker_16": {
                "position": {
                    "x": float(self.pose_16.pose.position.x),
                    "y": float(self.pose_16.pose.position.y),
                    "z": float(self.pose_16.pose.position.z)
                }
            },
            "marker_17": {
                "position": {
                    "x": float(self.pose_17.pose.position.x),
                    "y": float(self.pose_17.pose.position.y),
                    "z": float(self.pose_17.pose.position.z)
                }
            },
            "marker_719": {
                "position": {
                    "x": float(self.pose_719.pose.position.x),
                    "y": float(self.pose_719.pose.position.y),
                    "z": float(self.pose_719.pose.position.z)
                }
            },
            "marker_717": {
                "position": {
                    "x": float(self.pose_717.pose.position.x),
                    "y": float(self.pose_717.pose.position.y),
                    "z": float(self.pose_717.pose.position.z)
                }
            }
        }

        # save the file
        path = os.path.join(os.path.dirname(__file__), filename)
        with open(path, "w") as file:
            yaml.dump(data, file, default_flow_style=False)

        rospy.loginfo(f"Markers saved to {path}")



    def for_gaze_detection(self):
        head_tilt_looking_up = 0.33
        # setting sasha in the right position to look at the person
        self.whole_body.move_to_joint_positions({'head_tilt_joint': head_tilt_looking_up})
        rospy.sleep(0.5)

        self.tts.say("look at marker 16")
        rospy.sleep(6)

        self.tts.say("look at marker 15")
        rospy.sleep(6)

        self.tts.say("look at marker 17")
        rospy.sleep(6)

        self.tts.say("look at marker 717")
        rospy.sleep(6)

        self.tts.say("look at marker 719")
        rospy.sleep(6)

        self.tts.say("I have finished looking at the markers")
        rospy.sleep(2)
        self.whole_body.move_to_go()
        rospy.loginfo("Gaze detection finished, returning to initial position")

    
if __name__ == '__main__':
    rospy.logerr("Starting markers research node")
    marker_finder = FindMarker()
    marker_finder.get_marker_id()

    # #test pour trouver les angles de la tete 
    # marker_finder.get_head_pan_level()
    rospy.spin()