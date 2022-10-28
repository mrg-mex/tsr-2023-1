#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState

class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {wp.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_world_properties': {se}")    

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:    
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {ms.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_model_state': {se}")    
        

def main():
    gazebo_utils = GazeboUtils()
    wp = gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            ms = gazebo_utils.getModelState(model)
            position = (ms.pose.position.x, ms.pose.position.y) 
            print(f"model_name: {model} (x:{position[0]}, y:{position[1]})")


if __name__ == '__main__':
    main()