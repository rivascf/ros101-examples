#!/usr/bin/env python

import rospy
from receiver_pkg.msg import Status

def talker():
    pub = rospy.Publisher('status_info', Status, queue_size=10)
    rospy.init_node('my_talker', anonymous=True)
    r = rospy.Rate(10) #10 Hz
    msg = Status()
    msg.name = 'Ok'
    msg.code = 100

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    talker()