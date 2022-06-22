#!/usr/bin/env python

import rospy
from src.c1prox18_camera import C1ProX18

def run_c1prox18(freq=10):
    # Publisher rate and camera
    interval = rospy.Rate(freq)
    camera = C1ProX18()

    while not rospy.is_shutdown():
        # While not down
        camera.publish()
        interval.sleep()

if __name__ == '__main__':
    # Initialize & run the node
    rospy.init_node("c1prox18")
    run_c1prox18()