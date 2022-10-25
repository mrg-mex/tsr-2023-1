#!/usr/bin/env python3

import rospy
import cmdutil
from tb3_cmd.msg import TB3CmdMsg


def main():
    try:
        rospy.init_node('tb3_cmd_tester')
        cmd_pub = rospy.Publisher('/tb3_cmd_listener', TB3CmdMsg, queue_size=1)

        cmd_msg = TB3CmdMsg()
        cmd_msg.comando = ''
        cmd_msg.valor = 0.0
        prompt = 'Ingresa el comando'
        input_string = ''

        while not rospy.is_shutdown():
            key = cmdutil.getKey()
            if key == cmdutil.CTRL_C_CHAR:
                break
            elif key == cmdutil.BACKSPC_CHAR:
                if input_string != '':
                    input_string = input_string[:-1]    
            elif key == cmdutil.CR_CHAR:
                if input_string != '':
                    result_cmd = cmdutil.parseCommand(input_string)
                    if len(result_cmd) > 0 and result_cmd[0] != '':
                        if result_cmd[0].lower() == 'terminar':
                            break
                        else:
                            cmd_msg.comando = result_cmd[0]
                            cmd_msg.valor = float(result_cmd[1])

                        print('\ncomando: %s valor: %.2f' % (result_cmd[0], float(result_cmd[1])))

                    input_string = ''    

            else:
                input_string += key        
                print('%s: %s' % (prompt, input_string), end='\r')

            if cmd_msg.comando != '':
                cmd_pub.publish(cmd_msg)

    except rospy.ROSInterruptException as e:
        print(e)

    except Exception as ex:
        print(ex)

if __name__ == '__main__':
    main()