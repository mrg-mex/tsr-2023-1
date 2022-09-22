#!/usr/bin/env python3

import rospy
from tb3_cmd.msg import TB3CmdMsg
from geometry_msgs.msg import Twist

class TB3CmdListener(object):
    def __init__(self):
        self._cdmmsg = TB3CmdMsg()
        self._cmd_msg_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._tb3cmd_sub = rospy.Subscriber('/tb3_cmd_listener', TB3CmdMsg, self._on_tb3cmd_clbk)
        self._max_vel_lin = 0.5
        self._min_vel_lin = 0.05
        self._max_vel_ang = 0.3
        self._min_vel_ang = 0.03

    def _on_tb3cmd_clbk(self, msg):
        self._cdmmsg = msg
        robot_state = Twist()
        if not self._cdmmsg is None:
            if self._cdmmsg.comando.lower() == 'avanza':    #'AVANZA', 'Avanza', 'AvAnZa'
                robot_state.linear.x = self._cdmmsg.valor
                robot_state.angular.z = 0.0
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
            elif self._cdmmsg.comando.lower() == 'gira':    
                robot_state.linear.x = 0.0
                robot_state.angular.z = self._cdmmsg.valor
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
            elif self._cdmmsg.comando.lower() == 'detente':    
                robot_state.linear.x = 0.0
                robot_state.angular.z = 0.0
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
            else:
                rospy.logwarn('Recibi el comando ({}, {}) -> comando desconocido, descartado.'
                .format(self._cdmmsg.comando, self._cdmmsg.valor))
    
    def loop(self):
        rospy.spin()


def main():
    try:
        rospy.init_node('tb3_cmd')
        tb3cmd_list = TB3CmdListener()
        tb3cmd_list.loop()
    except rospy.ROSInterruptException as e:
        print(str(e))


if __name__ == "__main__":
    main()