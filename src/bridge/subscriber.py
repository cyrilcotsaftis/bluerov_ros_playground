#!/usr/bin/env python

import rospy
from bluerov_ros_playground.msg import Bar30
def callback(data):
    print(data)

        
def listener():
    rospy.init_node('CMD')
    rospy.Subscriber("/BlueRov2/bar30", Bar30, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()
