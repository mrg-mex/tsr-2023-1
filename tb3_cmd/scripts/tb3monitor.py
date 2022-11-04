#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
from gazebo_msgs.srv import GetWorldProperties, GetModelState

from tb3_cmd.srv import GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse

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
        

class DistanceMonitor():
    def __init__(self):
        self._landmarks = {}
        self._excluded_objs = ['ground_plane', 'turtlebot3_waffle']
        self._gazebo_utils = GazeboUtils()
        self._position = Point()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_callback)
        self._getClosestSrv = rospy.Service('/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/get_distance', GetDistance, self.get_distance_srv)
        self._ini()

    def _ini(self):
        wp = self._gazebo_utils.getWorldProperties()
        if wp:
            for model in wp.model_names:
                if model not in self._excluded_objs:
                    ms = self._gazebo_utils.getModelState(model)
                    position = (ms.pose.position.x, ms.pose.position.y)
                    self._landmarks.update({model: position})

    def _on_odom_callback(self, msg):
        self._position = msg.pose.pose.position    

    def get_closest_srv(self, req):
        closest_landmark = ''
        closest_distance = -1
        for model_name, (x, y) in self._landmarks.items():
            dx = x - self._position.x 
            dy = y - self._position.y
            sqr_dist = (dx * dx) + (dy * dy)
            if closest_distance == -1 or sqr_dist < closest_distance:
                closest_distance = sqr_dist
                closest_landmark = model_name

        response = GetClosestResponse()
        response.object_name = closest_landmark
        response.success = True
        response.status_message = "Todo OK"

        return response


    def get_distance_srv(self, req):
        response = GetDistanceResponse()
        if req.object_name not in self._landmarks:
            response.object_distance = 0.0
            response.success = False
            response.status_message = f"El objeto '{req.object_name}' no fue encontrado."
            return response

        x, y = self._landmarks[req.object_name]        
        dx = x - self._position.x 
        dy = y - self._position.y
        response.object_distance = math.hypot(dx, dy)
        response.success = True
        response.status_message = "Todo ok"

        return response


def test_services():
    gazebo_utils = GazeboUtils()
    wp = gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            ms = gazebo_utils.getModelState(model)
            position = (ms.pose.position.x, ms.pose.position.y) 
            print(f"model_name: {model} (x:{position[0]}, y:{position[1]})")


def main():
    rospy.init_node('distance_monitor_server')
    monitor = DistanceMonitor()
    rospy.spin()

if __name__ == '__main__':
    main()