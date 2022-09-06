#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('demo_ros_topic', String, queue_size=10)
    nodo = rospy.init_node("demo_ros_node")

    r = rospy.Rate(10)
    msg = "Hola demo"

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()


if __name__ == "__main__":
    publisher()