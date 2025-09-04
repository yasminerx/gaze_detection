#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from datetime import datetime

class PointingAngleCalculator:
    def __init__(self):
        rospy.init_node("pointing_angle_calculator", anonymous=True)

        # Subscribers
        rospy.Subscriber("/pointing/gaze", Marker, self.gaze_callback)
        rospy.Subscriber("/pointing/elbow_to_wrist", Marker, self.elbow_callback)
        rospy.Subscriber("/pointing/filtered_gaze", Marker, self.filtered_callback)

        # Storage for vectors
        self.gaze_vec = None
        self.elbow_vec = None
        self.filtered_vec = None

        # Storage for results
        self.results = []

        rospy.loginfo("Pointing angle calculator node started.")
        rospy.spin()

    def marker_to_vector(self, marker):
        """
        Convert an arrow Marker to a vector.
        Uses the first 2 points if available, otherwise pose orientation.
        """
        if marker.points and len(marker.points) >= 2:
            p1, p2 = marker.points[0], marker.points[1]
            vec = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
            return vec
        else:
            rospy.logwarn("Marker has no points, cannot compute vector.")
            return None

    def angle_between(self, v1, v2):
        """Compute angle in degrees between two vectors."""
        if v1 is None:
            rospy.logwarn("Vector 1 is None")
            return None
        if v2 is None:
            rospy.logwarn("Vector 2 is None")
            return None
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 < 1e-6 or norm2 < 1e-6:
            return None
        cos_theta = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
        return np.degrees(np.arccos(cos_theta))

    def compute_and_log(self):
        """Compute angles and log results if all vectors are available."""
        if self.gaze_vec is not None \
         and self.elbow_vec is not None \
         and self.filtered_vec is not None:

            angle_gaze_filtered = self.angle_between(self.gaze_vec, self.filtered_vec)
            angle_gaze_elbow = self.angle_between(self.gaze_vec, self.elbow_vec)
            angle_filtered_elbow = self.angle_between(self.filtered_vec, self.elbow_vec)

            timestamp = str(rospy.Time.now().to_sec())
            rospy.loginfo(f"[{timestamp}] "
                          f"Gaze-Filtered: {angle_gaze_filtered:.2f}°, "
                          f"Gaze-Elbow: {angle_gaze_elbow:.2f}°, "
                          f"Filtered-Elbow: {angle_filtered_elbow:.2f}°"
                        )

            # Save in memory for Excel export
            self.results.append({
                "time": timestamp,
                "gaze_filtered_angle": angle_gaze_filtered,
                "gaze_elbow_angle": angle_gaze_elbow,
                "filtered_elbow_angle": angle_filtered_elbow
            })

            # Write to Excel (overwrite each time)
            df = pd.DataFrame(self.results)
            df.to_excel("pointing_angles_01_09.xlsx", index=False)

    def gaze_callback(self, msg):
        self.gaze_vec = self.marker_to_vector(msg)
        rospy.loginfo("Gaze vector updated.")
        rospy.loginfo("Gaze = %s", self.gaze_vec)
        self.compute_and_log()

    def elbow_callback(self, msg):
        self.elbow_vec = self.marker_to_vector(msg)
        rospy.loginfo("Elbow vector updated.")
        rospy.loginfo("Elbow = %s", self.elbow_vec)
        self.compute_and_log()

    def filtered_callback(self, msg):
        self.filtered_vec = self.marker_to_vector(msg)
        rospy.loginfo("Filtered vector updated.")
        rospy.loginfo("Filtered = %s", self.filtered_vec)
        self.compute_and_log()

if __name__ == "__main__":
    try:
        PointingAngleCalculator()
    except rospy.ROSInterruptException:
        pass
