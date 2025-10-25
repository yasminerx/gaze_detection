#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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

        # Results for export
        self.results = []

        rospy.loginfo("Pointing angle calculator node started.")
        rospy.spin()

    # -------------------
    # Utility functions
    # -------------------

    def marker_to_vector(self, marker):
        """Convert an arrow Marker to a 3D vector."""
        if marker.points and len(marker.points) >= 2:
            p1, p2 = marker.points[0], marker.points[1]
            vec = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
            return vec
        else:
            rospy.logwarn("Marker has no points.")
            return None

    def project_to_xy(self, vec):
        """Project a 3D vector onto the horizontal (x, y) plane and normalize."""
        if vec is None:
            return None
        # Remove the z component
        projected = np.array([vec[0], vec[1], 0.0])
        norm = np.linalg.norm(projected)
        if norm < 1e-6:
            return None
        return projected / norm

    def angle_between(self, v1, v2):
        """Compute the 2D angle (degrees) between two projected vectors."""
        if v1 is None or v2 is None:
            return None
        cos_theta = np.clip(np.dot(v1, v2), -1.0, 1.0)
        return np.degrees(np.arccos(cos_theta))

    # -------------------
    # Core computation
    # -------------------

    def compute_and_log(self):
        """Project vectors, compute planar angles, and save results."""
        if self.gaze_vec is not None and \
           self.elbow_vec is not None and \
           self.filtered_vec is not None:

            # Project all into the horizontal plane
            gaze_xy = self.project_to_xy(self.gaze_vec)
            elbow_xy = self.project_to_xy(self.elbow_vec)
            filtered_xy = self.project_to_xy(self.filtered_vec)

            # Compute planar angles
            angle_gaze_filtered = self.angle_between(gaze_xy, filtered_xy)
            angle_gaze_elbow = self.angle_between(gaze_xy, elbow_xy)
            angle_filtered_elbow = self.angle_between(filtered_xy, elbow_xy)

            timestamp = str(rospy.Time.now().to_sec())
            rospy.loginfo(f"[{timestamp}] "
                          f"Gaze-Filtered: {angle_gaze_filtered:.2f}°, "
                          f"Gaze-Elbow: {angle_gaze_elbow:.2f}°, "
                          f"Filtered-Elbow: {angle_filtered_elbow:.2f}°")

            # Save to memory
            self.results.append({
                "time": timestamp,
                "gaze_filtered_angle": angle_gaze_filtered,
                "gaze_elbow_angle": angle_gaze_elbow,
                "filtered_elbow_angle": angle_filtered_elbow
            })

            # Save to Excel file
            df = pd.DataFrame(self.results)
            df.to_excel("pointing_angles_projected.xlsx", index=False)

    # -------------------
    # Callbacks
    # -------------------

    def gaze_callback(self, msg):
        self.gaze_vec = self.marker_to_vector(msg)
        rospy.loginfo(f"Gaze vector updated: {self.gaze_vec}")
        self.compute_and_log()

    def elbow_callback(self, msg):
        self.elbow_vec = self.marker_to_vector(msg)
        rospy.loginfo(f"Elbow vector updated: {self.elbow_vec}")
        self.compute_and_log()

    def filtered_callback(self, msg):
        self.filtered_vec = self.marker_to_vector(msg)
        rospy.loginfo(f"Filtered vector updated: {self.filtered_vec}")
        self.compute_and_log()


if __name__ == "__main__":
    try:
        PointingAngleCalculator()
    except rospy.ROSInterruptException:
        pass
