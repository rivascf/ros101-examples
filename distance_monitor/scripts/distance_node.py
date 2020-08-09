#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from loc_msgs.srv import (GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse)
import math

class DistanceMonitorServer():

    def __init__(self):
        self._landmarks = {
            "brick_box_3x1x3": (3.0, 0.0), 
            "box_0": (1.13615, 6.00198), 
            "sphere_0": (2.45136, -3.96208), 
            "cylinder_0": (-2.45033023042, 0.837319245163)
        }
        self._position = Point()
        self._odometry = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self._getClosestSrv = rospy.Service('/distance_monitor_server/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/distance_monitor_server/get_distance', GetDistance, self.get_distance_srv)

    def odometry_callback(self, msg):
        self._position = msg.pose.pose.position

    def get_closest_srv(self, request):
        rospy.loginfo('GetClosest called')
        closest_landmark = ''
        closest_distance = -1
        for name, (x, y) in self._landmarks.items():
            dx = x - self._position.x
            dy = y - self._position.y
            sq_dist = dx * dx + dy * dy
            if closest_distance == -1 or sq_dist < closest_distance:
                closest_distance = sq_dist
                closest_landmark = name

        response = GetClosestResponse()
        response.name = closest_landmark
        return response        

    def get_distance_srv(self, request):
        rospy.loginfo('Called GetDistanceSrv with {}'.format(request.name))
        if request.name not in self._landmarks:
            rospy.logerr('landmark no encontrada "{}"'.format(request.name))
            return None
        
        x, y = self._landmarks[request.name]
        dx = x - self._position.x
        dy = y - self._position.y
        response = GetDistanceResponse()
        response.distance = math.sqrt(dx * dx + dy * dy)
        return response


def main():
    rospy.init_node('distance_monitor_server')
    rospy.loginfo('Initializing Distance Monitor Services...')
    monitor = DistanceMonitorServer()
    rospy.spin()

if __name__ == '__main__':
    main()    