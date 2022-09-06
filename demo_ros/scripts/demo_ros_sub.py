#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def subscriber():
    node = rospy.init_node('demo_ros_sub_node')
    sub = rospy.Subscriber('demo_ros_topic', String, proceso_callback)
    rospy.spin()

def proceso_callback(msg):
    rospy.loginfo('Recibi el mensaje: ' +  msg.data)    

if __name__ == "__main__":
    subscriber()