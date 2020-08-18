#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from gazebo_msgs.srv import (GetWorldProperties, GetModelState)


def getWorldProperties():
    try:
        get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)      
        wp = get_world_properties()
        if wp.success:
            return wp
        else:
            print wp.status_message    
            return None
    except rospy.ServiceException, e:
        print "/gazebo/get_world_properties %s"%e

def getModelState(model_name, relative_entity_name='world'):
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

def get_model_pose(model_name):
    ms = getModelState(model_name)
    if ms:
        return ms.pose
    else:
        return None

def main():
    exclude = ['ground_plane', 'mobile_base']
    wp = getWorldProperties()
    print wp.model_names
    for model in wp.model_names:
        if model not in exclude:
            print model
            ms = get_model_pose(model)
            print ms.position
    

if __name__ == '__main__':
    main()    