#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tb3_move.msg import TB3MoveAction, TB3MoveFeedback, TB3MoveResult
from tf import transformations
import math

class TB3MoveActionServer():
    def __init__(self):
        # Descripcion actual del robot (Pose, Header)
        self._header = Header()
        self._ipose = Pose2D() #pose actual del robot
        # Estados del robot
        self._irobot_state_code = 0
        self._itb3_robot_states = ['STOP', 'TWIST', 'GO', 'GOAL']
        # GOAL y variables de control
        self._goal = None
        self._goal_queue = []
        self._distance_to_goal = 0.0
        self._iyaw_error = 0.0
        self._ang_vel = 0.1     # rads/seg
        self._lin_vel = 0.1     # m/seg
        self._tol_err_yaw = 0.0872665 # rads = 5 grados
        self._tol_err_dist = 0.05     # metros
        # Publicadores y subscriptores
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update) 
        # Inicializacion de Simple Action Server
        self._action_server = actionlib.SimpleActionServer('TB3_Move_server', TB3MoveAction, self._execute, False)
        rospy.loginfo("TB3MoveActionServer: Inicializado")


    #######################################################
    ## Aqui migramos el codigo del script tb3_go2point   ##
    #######################################################

    # 'odom' funcion de callback
    def _on_odometry_update(self, odom_msg):
        pose_act = odom_msg.pose.pose
        quaternion = [
            pose_act.orientation.x,
            pose_act.orientation.y,
            pose_act.orientation.z,
            pose_act.orientation.w
        ]
        ang_euler = transformations.euler_from_quaternion(quaternion) # -> tupla(roll, pitch, yaw)
        self._ipose.x = pose_act.position.x # en metros
        self._ipose.y = pose_act.position.y # en metros
        self._ipose.theta = ang_euler[2]  # Solo tomo el angulo de rot en 'Z' (yaw) en radianes

    # Funcion para el calculo del error en la distancia y orientacion
    def _compute_goal(self):
        dx = (self._goal.x - self._ipose.x)
        dy = (self._goal.y - self._ipose.y)
        phi = math.atan2(dy, dx)
        dif_dist = math.hypot(dx, dy)
        dyaw = phi - self._ipose.theta
        return dyaw, dif_dist

    # Giro en la orientacion (heading)
    def _head_towards_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        # rospy.loginfo(f'HEADING: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        self._iyaw_error = goal_yaw
        self._distance_to_goal = dist_to_goal
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            # mandar el comando de giro al robot
            self._send_vel_robot(vel_ang=ang_vel, robot_state=1) #robot_state='TWIST'
        else:
            self._irobot_state_code = 2       # self._robot_state = 'GO'    

    # Desplazamiento hacia la meta
    def _go_staight(self):
        goal_yaw, dist_to_goal = self._compute_goal()    
        # rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        self._iyaw_error = goal_yaw
        self._distance_to_goal = dist_to_goal
        if self._irobot_state_code not in [0,3]:        # if self._robot_state not in ['GOAL','STOP']
            if dist_to_goal > self._tol_err_dist:
                self._send_vel_robot(vel_lin=self._lin_vel, robot_state=2) # robot_state='GO'
            else: # Llegamos a la meta
                self._irobot_state_code = 3      # self._robot_state = 'GOAL'
                # rospy.loginfo(f"GOAL!, yaw error: {goal_yaw:.6f}, dist error: {dist_to_goal:.6f}")
                self._send_vel_robot(robot_state=3)
            if math.fabs(goal_yaw) > self._tol_err_yaw:
                #self._head_towards_goal()    
                self._irobot_state_code = 1     # self._robot_state = 'TWIST'

    def _send_vel_robot(self, vel_ang = 0.0, vel_lin = 0.0, robot_state=0):
        self._irobot_state_code = robot_state
        cmd_twist =  Twist()
        cmd_twist.linear.x = vel_lin
        cmd_twist.angular.z = vel_ang

        self._cmd_vel_pub.publish(cmd_twist)


    # Funcion para detener al robot
    def _stop_robot(self):
        # rospy.loginfo('Deteniendo al robot...')
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self._irobot_state_code = 0

    ############################################################
    ## Aqui empieza la implementacion de SimpleActionServer   ##
    ############################################################

    # Actionlib callback function
    def _execute(self, goal):
        success = True  
        # Recibimos un nuevo GOAL 
        rospy.loginfo("NEW GOAL received!")

        if not self._accept_goal(goal):
            rospy.logerr("NEW GOAL abortada.")
            self._action_server.set_aborted()
            return

        # NEW GOAL aceptada
        rospy.loginfo(f"NEW GOAL aceptada, GOAL({self._goal.x},{self._goal.y}, {self._goal.theta})")    

        # Ejecusion del proceso de tb3_go2point
        # Ciclamos mientras no lleguemos al estado 3 ('GOAL')
        while not self._irobot_state_code == 3:
            if self._action_server.is_preempt_requested():
                success = False
                rospy.logwarn("PREEMPT flag recibida! Finalizando el proceso para la meta actual.")
                break
            if self._irobot_state_code == 1:
                self._head_towards_goal()
            elif self._irobot_state_code == 2:
                self._go_staight()
            else:
                success = False
                rospy.logerr(f"Assert error, irobot state code ({self._irobot_state_code}).")
                break

            # Al terminar la iteracion envio el feedback al cliente
            self._send_feedback()

        # Al terminar el ciclo con: 
        # success = False y el estado actual del robot
        # o 
        # success = True y el estado actual del robot == 3 ('GOAL')

        result_msg = self._create_result_msg(success)
        self._stop_robot()
        if success:
            self._action_server.set_succeeded(result_msg)
            rospy.loginfo("Proceso terminado exitosamente.")
        else:
            self._action_server.set_preempted(result_msg)
            rospy.logwarn("Proceso terminado, GOAL PREEMPTED")

        # self._goal_queue.pop(0)    

    def _accept_goal(self, new_goal):
        # Estados del robot
        #   0        1       2      3
        # 'STOP', 'TWIST', 'GO', 'GOAL'
        #  Una vez que se evalue si la nueva meta es válida
        # self._goal_queue.append(new_goal) 
        if self._irobot_state_code == 0 or self._irobot_state_code == 3:
            self._irobot_state_code = 2
            self._goal = new_goal.target

            return True

        rospy.logwarn(f"Estado actual del robot {self._itb3_robot_states[self._irobot_state_code]}. GOAL rechazada!")
        return False  

    def _send_feedback(self):
        feedback_msg = TB3MoveFeedback()
        feedback_msg.position = self._ipose
        feedback_msg.state_name = self._itb3_robot_states[self._irobot_state_code]
        feedback_msg.distance_error = self._distance_to_goal
        feedback_msg.yaw_error = self._iyaw_error

        self._action_server.publish_feedback(feedback_msg)

    def _create_result_msg(self, success):
        result_msg = TB3MoveResult()
        result_msg.header = Header()
        result_msg.header.seq = 1
        result_msg.header.stamp = rospy.Time.now()
        result_msg.header.frame_id = ""
        result_msg.position = self._ipose
        result_msg.state_name = self._itb3_robot_states[self._irobot_state_code]
        result_msg.distance_error = self._distance_to_goal
        result_msg.yaw_error = self._iyaw_error
        result_msg.success = success
        # result_msg.goal_message = "GOAL completada exitosamente" if success else "GOAL fallo"
        if success:
            result_msg.goal_message = "GOAL completada exitosamente" 
        else:
            result_msg.goal_message = "GOAL fallo"

        return result_msg    

    def start(self):
        rospy.loginfo("Server started.")
        self._action_server.start()    


def main():
    rospy.init_node('tb3_actionlib_server_node')
    tb3_actionsrv = TB3MoveActionServer()
    tb3_actionsrv.start()
    rospy.spin()

if __name__ == '__main__':
    main()