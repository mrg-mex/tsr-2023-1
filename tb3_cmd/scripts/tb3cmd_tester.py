#!/usr/bin/env python3

import rospy
from tb3_cmd.msg import TB3CmdMsg


def main():
    try:
        rospy.init_node('tb3_cmd_tester')
        cmd_pub = rospy.Publisher('/tb3_cmd_listener', TB3CmdMsg, queue_size=1)
        rate = rospy.Rate(1)
        comando = TB3CmdMsg()
        comando.comando = 'IDLE'
        comando.valor = 0.0
        while not rospy.is_shutdown():
            cmd_word = input("Comando?: ")
            if str(cmd_word).lower() == 'terminar':
                break
            else:
                comando.comando = str(cmd_word)
                input_val = float(input("Valor?: "))
                if input_val > 0.5:
                    comando.valor = 0.05
                    rospy.logwarn('El valor ({}) es demasiado alto, usando 0.05 de castigo.'.format(input_val))
                else:
                    comando.valor = input_val

            cmd_pub.publish(comando)
            rate.sleep()

    except rospy.ROSInterruptException as e:
        print(e)

if __name__ == '__main__':
    main()