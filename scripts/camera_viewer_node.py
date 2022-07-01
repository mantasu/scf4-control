#!/usr/bin/env python

import cv2
import rospy
from scf4_control.camera_viewer import CameraViewer

def run_camera_viewer(freq=10):
    # Publisher rate and camera
    interval = rospy.Rate(freq)
    camera_viewer = CameraViewer()

    while not rospy.is_shutdown():
        # While not down
        interval.sleep()
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initialize & run the node
    rospy.init_node("camera_viewer", anonymous=True)
    run_camera_viewer()
    