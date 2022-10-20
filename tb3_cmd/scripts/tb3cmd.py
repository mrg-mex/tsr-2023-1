#!/usr/bin/env python3

import rospy
from tb3_cmd.msg import TB3CmdMsg
from geometry_msgs.msg import Twist

class TB3CmdListener(object):
    def __init__(self):
        self._cdmmsg = TB3CmdMsg()
        self._cmd_msg_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._tb3cmd_sub = rospy.Subscriber('/tb3_cmd_listener', TB3CmdMsg, self._on_tb3cmd_clbk)
        self._model = ''
        self._current_lin_vel = 0.0
        self._current_ang_vel = 0.0
        self._max_vel_lin = 0.5
        self._min_vel_lin = 0.05
        self._max_vel_ang = 0.3
        self._min_vel_ang = 0.03
        self.WAFFLE_MAX_LIN_VEL = 0.26  # m/s
        self.WAFFLE_MAX_ANG_VEL = 1.82  # rad/s
        self.BURGUER_MAX_LIN_VEL = 0.22
        self.BURGUER_MAX_ANG_VEL = 2.84
        self.LIN_VEL_INC = 0.01
        self.ANG_VEL_INC = 0.1

    def loadParameters(self):
        self._model = rospy.get_param("tb3_model", "burger")
        if self._model == 'waffle':
            self._max_vel_lin = rospy.get_param("tb3_max_lin_vel", self.WAFFLE_MAX_LIN_VEL)
            self._max_vel_ang = rospy.get_param("tb3_max_lin_vel", self.WAFFLE_MAX_ANG_VEL)
        else:
            self._max_vel_lin = rospy.get_param("tb3_max_lin_vel", self.BURGUER_MAX_LIN_VEL)
            self._max_vel_ang = rospy.get_param("tb3_max_lin_vel", self.BURGUER_MAX_ANG_VEL)

    def _constraint(self, input_value, low, high):
        if input_value < low:
            input_value = low
        elif input_value > high:
            input_value = high
        else:
            input_value = input_value

        return input_value

    def _checkLinearLimitVel(self, vel):
        if self._model == 'burguer':
            vel =self._constraint(vel, -self.BURGUER_MAX_LIN_VEL, self.BURGUER_MAX_LIN_VEL)
        elif self._model == 'waffle' or self._model == 'waffle_pi':
            vel =self._constraint(vel, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)

        return vel  

    def _checkAngularLimitVel(self, vel):
        if self._model == 'burguer':
            vel =self._constraint(vel, -self.BURGUER_MAX_ANG_VEL, self.BURGUER_MAX_ANG_VEL)
        elif self._model == 'waffle' or self._model == 'waffle_pi':
            vel =self._constraint(vel, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_ANG_VEL)

        return vel

    def vels(self, current_lin_vel, current_ang_vel):
        return "Actualmente:\tvel lineal %s\tvel angular %s" % (current_lin_vel, current_ang_vel)

    def _on_tb3cmd_clbk(self, msg):
        self._cdmmsg = msg
        robot_state = Twist()
        # Para cualquiera de las velocidades:
        # - Verificar si el valor está en los limites de acuerdo al modelo
        # - Ajustar el valor de acuerdo al limite
        # - Agregar funciionalidad para incremento fijo de acuerdo al modelo
        if not self._cdmmsg is None:
            if self._cdmmsg.comando.lower() == 'avanza':    #'AVANZA', 'Avanza', 'AvAnZa'
                self._current_lin_vel = self._checkLinearLimitVel(self._cdmmsg.valor)
                robot_state.linear.x = self._current_lin_vel
                robot_state.angular.z = self._current_ang_vel
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
                rospy.loginfo(self.vels(self._current_lin_vel, self._current_ang_vel))
            elif self._cdmmsg.comando.lower() == 'gira':  
                # Tarea: Completar  
                robot_state.linear.x = 0.0
                robot_state.angular.z = self._cdmmsg.valor
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
            elif self._cdmmsg.comando.lower() == 'detente':    
                # Tarea: Completar  
                robot_state.linear.x = 0.0
                robot_state.angular.z = 0.0
                self._cmd_msg_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando ({}, {}) -> comando enviado.'.format(self._cdmmsg.comando, self._cdmmsg.valor))
            elif self._cdmmsg.comando.lower() == 'incl':
                # Tarea: Desarrollar  
                pass
            elif self._cdmmsg.comando.lower() == 'inca':
                # Tarea: Desarrollar  
                pass
            else:
                rospy.logwarn('Recibi el comando ({}, {}) -> comando desconocido, descartado.'
                .format(self._cdmmsg.comando, self._cdmmsg.valor))
    
    def loop(self):
        rospy.spin()


def main():
    try:
        rospy.init_node('tb3_cmd')
        tb3cmd_list = TB3CmdListener()
        tb3cmd_list.loadParameters()
        rospy.loginfo('Model: %s' % tb3cmd_list._model)
        tb3cmd_list.loop()
    except rospy.ROSInterruptException as e:
        print(str(e))


if __name__ == "__main__":
    main()