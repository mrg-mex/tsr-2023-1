#!/usr/bin/env python3

# sys — Parámetros y funciones específicos del sistema <https://docs.python.org/es/3/library/sys.html?highlight=sys#module-sys>
# select — Esperando la finalización de E/S <https://docs.python.org/es/3/library/select.html>
# os — Interfaces misceláneas del sistema operativo <https://docs.python.org/es/3/library/os.html?highlight=#module-os>
import sys, select, os
# tty — Funciones de control de terminal <https://docs.python.org/es/3/library/tty.html>
# termios —Control tty estilo POSIX <https://docs.python.org/es/3/library/termios.html>
# io — Herramientas principales para trabajar con streams <https://docs.python.org/es/3/library/io.html#io.IOBase.fileno>
import tty, termios   
import re
from unittest import result

re_pattern = r'(^[a-zA-Z]+)|([-.0-9]+)'
CTRL_C_CHAR = '\x03'
CR_CHAR = '\r' # CR \r + LF 
ESC_CHAR = chr(27)
BACKSPC_CHAR = chr(127)

pmt = '$$'
settings = termios.tcgetattr(sys.stdin)

def getKey():

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def parseCommand(input_str):

#    tokens = iter(re.split(re_pattern, input_str))
    tokens = re.findall(re_pattern, input_str, re.I)
    result = []
    if len(tokens) > 1:
        result.append(tokens[0][0])
        result.append(tokens[1][1])
    if len(tokens) == 1:
        result.append(tokens[0][0])
        result.append('0.0')
    return result



if __name__ == '__main__':
    #settings = termios.tcgetattr(sys.stdin)
    input_string = ''
    print(pmt + ': ', end='\r')
    while (1):
        key = getKey()
        #if key != '':
        #      print('Pressed: [\\x%s]' % ord(key))
        if key == CTRL_C_CHAR:
            break
        elif key == ESC_CHAR:
            input_string = ''
        elif key == BACKSPC_CHAR:
            if input_string != '':
                input_string = input_string[:-1]    
        elif key == CR_CHAR:
            if input_string != '':
                result_str = parseCommand(input_string)
                input_string = ''
                print('comando: %s valor: %.2f' % (result_str[0], float(result_str[1])))
        else:
            # print(key)
            input_string += key        
            print('%s: %s' % (pmt, input_string), end='\r') 

    print('fin')
