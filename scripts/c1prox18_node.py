#!/usr/bin/env python

import rospy
import argparse
from scf4_control.c1prox18_camera import C1ProX18

DEFAULT_CONFIG_PATH = "config.json"

def run_c1prox18(freq=10):
    # Publisher rate and camera
    interval = rospy.Rate(freq)
    camera = C1ProX18()

    while not rospy.is_shutdown():
        # While not down
        camera.publish()
        interval.sleep()

        if rospy.is_shutdown():
            # Release capture frame
            camera.capture.release()

if __name__ == '__main__':
    # Initialize & run the node
    rospy.init_node("c1prox18", anonymous=True)
    run_c1prox18()