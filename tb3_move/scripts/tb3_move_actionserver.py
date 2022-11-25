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
        self._ipose = Pose2D #pose actual del robot
        # Estados del robot
        self._irobot_state_code = 0
        self._itb3_robot_states = ['STOP', 'TWIST', 'GO', 'GOAL']
        # GOAL y variables de control
        self._goal = None
        self._distance_to_goal = 0.0
        self._iyaw_error = 0.0
        self._ang_vel = 0.1     # rads/seg
        self._lin_vel = 0.1     # m/seg
        self._tol_err_yaw = 0.0872665 # rads = 5 grados
        self._tol_err_dist = 0.05     # metros
        # Publicadores y subscriptores
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update) 
        rospy.loginfo("TB3MoveActionServer: Inicializado")

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
        rospy.loginfo(f'HEADING: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            # mandar el comando de giro al robot
            self._send_vel_robot(vel_ang=ang_vel, robot_state='TWIST')
        else:
            self._robot_state = 'GO'    

    # Desplazamiento hacia la meta
    def _go_staight(self):
        goal_yaw, dist_to_goal = self._compute_goal()    
        rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if self._robot_state not in ['GOAL','STOP']:
            if dist_to_goal > self._tol_err_dist:
                self._send_vel_robot(vel_lin=self._lin_vel, robot_state='GO')
            else: # Llegamos a la meta
                self._robot_state = 'GOAL'
                rospy.loginfo(f"GOAL!, yaw error: {goal_yaw:.6f}, dist error: {dist_to_goal:.6f}")
                self._send_vel_robot()
            if math.fabs(goal_yaw) > self._tol_err_yaw:
                #self._head_towards_goal()    
                self._robot_state = 'TWIST'

    # Funcion para detener al robot
    def _stop_robot(self):
        rospy.loginfo('Deteniendo al robot...')
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self.self._irobot_state_code = 0

    # Actionlib callback function
    def _execute(self, goal):
        pass    


def main():
    pass

if __name__ == '__main__':
    main()