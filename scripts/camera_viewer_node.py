#!/usr/bin/env python

import cv2
import rospy
import argparse
from scf4_control.camera.viewer import CameraViewer

def parse_args():
    """Parses command-line arguments

    Gets the desired window width and height. The window captures the
    real-time camera output.

    Returns:
        tuple: The desired video output dimensions, 1920x1080 by default
    """
    # Create a parser for command args
    parser = argparse.ArgumentParser()

    # Add an argument for the output video width
    parser.add_argument("--width", type=int,
        default=1920, help="The output video width")

    # Add an argument for the output video height
    parser.add_argument("--height", type=int,
        default=1080, help="The output video height")

    # Parse known command-line arguments
    args, _ = parser.parse_known_args()

    return args.width, args.height

def run_camera_viewer(args=(), freq=10):
    # Publisher rate and camera
    interval = rospy.Rate(freq)
    camera_viewer = CameraViewer(*args)

    while not rospy.is_shutdown():
        # While not down
        interval.sleep()

    # Destroy opencv window
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Get cmd-line args
    args = parse_args()

    # Initialize and run the camera viewer node
    rospy.init_node("camera_viewer", anonymous=True)
    run_camera_viewer(args)
    