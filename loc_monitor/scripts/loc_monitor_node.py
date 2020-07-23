#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w

    rospy.loginfo('x: {} y: {} o: ({}, {}, {}, {}) '.format(x, y, ox, oy, oz, ow))

def main():
    rospy.init_node('loc_monitor')
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
