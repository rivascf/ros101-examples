#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from loc_msgs.srv import (GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse)
from gazebo_msgs.srv import (GetWorldProperties, GetModelState)
import math

class GazeboUtils():
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)      
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(wp.status_message)    
                return None
        except rospy.ServiceException, e:
            print "/gazebo/get_world_properties %s"%e

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:
                print ms.status_message
                return None
        except rospy.ServiceException, e:
            print "/gazebo/get_model_state %s"%e

    def get_model_pose(self, model_name):
        ms = getModelState(model_name)
        if ms:
            return ms.pose
        else:
            return None


class DistanceMonitorServer():

    def __init__(self):
        self._landmarks = {}
        self._exclude = ['ground_plane', 'mobile_base']
        self._utils = GazeboUtils()
        self._position = Point()
        self._odometry = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self._getClosestSrv = rospy.Service('/distance_monitor_server/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/distance_monitor_server/get_distance', GetDistance, self.get_distance_srv)

    def _init(self):
        wp = self._utils.getWorldProperties()
        for model in wp.model_names:
            if model not in self._exclude:
                # ms = self._utils.get_model_pose(model)
                ms = self._utils.getModelState(model)
                position =  (ms.pose.position.x, ms.pose.position.y)
                self._landmarks.update({ model: position})
                # self._landmarks.update(model, [ms.point.x, ms.point.y, ms.point.z])
        rospy.loginfo("landmarks detected: ")
        rospy.loginfo(self._landmarks)

    def _refresh_landmarks(self):
        self._landmarks = {}
        self._init()   

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
    monitor._init()
    rospy.spin()

if __name__ == '__main__':
    main()    