#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math

class GoToPoint:
    def __init__(self):
        rospy.init_node("tb3_go2point")
        rospy.loginfo("Starting GoToPointNode as tb3_go2point.")
        self._pose_act = Pose2D()
        self._distance_to_go = 0.0
        self._goal = Pose2D()
        self._phi = 0.0
        self._tol_err_yaw = 0.0872665 # rads = 5 grados
        # 0.0349066 # rads = 2 grados
        self._tol_err_dist = 0.05       #0.001
        self._ang_vel = 0.1
        self._lin_vel = 0.1
        self._robot_state = 'STOP' # Variable de estado del robot [STOP, GO, TWIST, GOAL]
        self._goal_reached = False
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update)
        self._cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _on_odometry_update(self, msg):
        pose_act = msg.pose.pose
        quaternion = [
            pose_act.orientation.x,
            pose_act.orientation.y,
            pose_act.orientation.z,
            pose_act.orientation.w
        ]
        ang_euler = transformations.euler_from_quaternion(quaternion) # -> tupla(roll, pitch, yaw)
        self._pose_act.x = pose_act.position.x # en metros
        self._pose_act.y = pose_act.position.y # en metros
        self._pose_act.theta = ang_euler[2]  # Solo tomo el angulo de rot en 'Z' (yaw) en radianes

    def _compute_goal(self):
        dx = (self._goal.x - self._pose_act.x)
        dy = (self._goal.y - self._pose_act.y)
        phi = math.atan2(dy, dx)
        dif_dist = math.hypot(dx, dy)
        dyaw = phi - self._pose_act.theta
        return dyaw, dif_dist

    def _head_towards_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        rospy.loginfo(f'HEADING: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            # mandar el comando de giro al robot
            self._send_vel_robot(vel_ang=ang_vel, robot_state='TWIST')
        else:
            self._robot_state = 'GO'    

    def _go_staight(self):
        goal_yaw, dist_to_goal = self._compute_goal()    
        rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if self._robot_state not in ['GOAL','STOP']:
            if dist_to_goal > self._tol_err_dist:
                self._send_vel_robot(vel_lin=self._lin_vel, robot_state='GO')
            else: # Llegamos a la meta
                self._robot_state = 'GOAL'
                rospy.loginfo(f"GOAL!, yaw error: {goal_yaw:.6f}, dist error: {dist_to_goal:.6f}")
                self._goal_reached = True
                self._send_vel_robot()
            if math.fabs(goal_yaw) > self._tol_err_yaw:
                #self._head_towards_goal()    
                self._robot_state = 'TWIST'

    def set_goal(self, x, y, theta):
        self._goal.x = x
        self._goal.y = y
        self._goal.theta = theta

    def _send_vel_robot(self, vel_ang = 0.0, vel_lin = 0.0, robot_state='STOP'):
        self._robot_state = robot_state
        cmd_twist =  Twist()
        cmd_twist.linear.x = vel_lin
        cmd_twist.angular.z = vel_ang

        self._cmdvel_pub.publish(cmd_twist)

    def start(self):
        self._robot_state = 'GO'

    def getRobotState(self):
        return self._robot_state

    def stop(self):
        self._send_vel_robot()
        rospy.sleep(1)

    def is_goal_reched(self):
        return self._goal_reached

if __name__ == "__main__":
    tb3_go2point = GoToPoint()
    tb3_go2point.set_goal(3, 2, 0)
    rate = rospy.Rate(1)
    start_time = rospy.Time.now()
    if tb3_go2point.getRobotState() == 'STOP':
        tb3_go2point.start()
    while (not rospy.is_shutdown()):
        rospy.loginfo(f"Estado actual {tb3_go2point.getRobotState()}")
        if tb3_go2point.getRobotState() == 'TWIST':
            tb3_go2point._head_towards_goal()
        elif tb3_go2point.getRobotState() == 'GO':
            tb3_go2point._go_staight()
        elif tb3_go2point.is_goal_reched():      # Cond. anterior: tb3_go2point.getRobotState() == 'GOAL'
            end_time = rospy.Time.now()
            elapse_time = end_time - start_time  
            tb3_go2point.stop()
            rospy.loginfo(f"On GOAL, posicion act x: {tb3_go2point._pose_act.x:.6f}, y: {tb3_go2point._pose_act.y:.6f}, theta: {tb3_go2point._pose_act.theta:.6f} rads.")
            rospy.loginfo(f"Elpsed time: {elapse_time.to_sec():.4f} seg.")
            break

    rospy.loginfo(f"Proceso concluido.")
    
    
    