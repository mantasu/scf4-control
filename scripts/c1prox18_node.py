#!/usr/bin/env python

import rospy
import argparse
from scf4_control.c1prox18 import C1ProX18

DEFAULT_CONFIG_PATH = "config.json"

def parse_args():
    """Parses command line arguments

    Gets the location for the JSON configuration file for controller and
    camera properties(if not provided, the default one is within the
    package). Also gets the argument telling if the path is relative to 
    this ros package location, i.e., is inside this package or should be
    treated as an absolute path.

    Returns:
        tuple: A tuple containing config.json file path and whether the
            path is relative to the package location
    """
    # Create a parser for command args
    parser = argparse.ArgumentParser()

    # Add an argument for the JSON configuration path
    parser.add_argument("-p", "--config-path", type=str,
        default=DEFAULT_CONFIG_PATH, help="The path to JSON config file")

    # Add an argument for whether the path is in pkg
    parser.add_argument("--is-relative", type=bool,
        default=True, help="Whether the path is relative to this package")

    # Parse known command-line arguments
    args, _ = parser.parse_known_args()

    return args.config_path, args.is_relative

def run_c1prox18(args=(), freq=30):
    # Publisher rate and camera
    interval = rospy.Rate(freq)
    camera = C1ProX18(*args)

    while not rospy.is_shutdown():
        # While not down
        camera.publish()
        interval.sleep()

        if rospy.is_shutdown():
            # Free resources
            camera.release()

if __name__ == '__main__':
    # Get cmd-line args
    args = parse_args()

    # Initialize and run the main camera node
    rospy.init_node("c1prox18", anonymous=True)
    run_c1prox18(args)