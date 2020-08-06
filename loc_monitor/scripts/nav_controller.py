#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from math import atan2

x = 0.0  #metros
y = 0.0  #metros
theta = 0.0 #Radianes

def controller_callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    oq = msg.pose.pose.orientation

    #formato de quaterniones (w, x, y, z) -> (x, y, z, w)
    (roll, pitch, theta) = euler_from_quaternion([oq.x, oq.y, oq.z, oq.w])

def main():
    rospy.init_node('nav_controller')
    
    sub = rospy.Subscriber('/odom', Odometry, controller_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    goal1 = Point()
    goal1.x = 4
    goal1.y = 0

    goal = Point()
    goal.x = 4
    goal.y = 0

    speed = Twist()
    r = rospy.Rate(4)
    current_goal = goal1

    while not rospy.is_shutdown():

        dx = current_goal.x - x
        dy = current_goal.y - y
        alpha = atan2(dy, dx)
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        rospy.loginfo('dx: {} dy:{} alpha: {}'.format(dx, dy, alpha))

        if abs(alpha - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3 # rad/s
        else:
            if (x < current_goal.x and y < current_goal.y):
                speed.linear.x = 0.5 # m/s
                speed.angular.z = 0.0
            elif current_goal == goal1 :
                current_goal = goal
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0    
                            
        pub.publish(speed)
        r.sleep()

if __name__ == '__main__':
    main()