#!/usr/bin/env python

import rospy
from receiver_pkg.msg import Status

def callback(data):
    rospy.loginfo('El estado del robot es {} - {}'.format(data.code, data.name))
                # 'El estado del robot es %d %s', (data.code, data.name)
def listener():
    rospy.init_node('my_listener', anonymous=True)
    rospy.Subscriber('status_info', Status, callback)

    rospy.spin()

if __name__=='__main__':
    listener()