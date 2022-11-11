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
        self._tol_err_yaw = 0.0349066 # rads = 2 grados
        self._tol_err_dist = 0.001
        self._ang_vel = 0.1
        self._lin_vel = 0.1
        self._robot_state = 'STOP' # Variable de estado del robot
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
        dyaw = math.atan2(dy, dx)
        dif_dist = math.hypot(dx, dy)

        return dyaw, dif_dist

    def _head_towards_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        rospy.loginfo(f'HEAD: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            # mandar el comando de giro al robot
            self._send_vel_robot(vel_ang=ang_vel)

    def _go_staight(self):
        goal_yaw, dist_to_goal = self._compute_goal()    
        rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f}, dist to go: {dist_to_goal:.6f}')
        if dist_to_goal > self._tol_err_dist:
            self._send_vel_robot(vel_lin=self._lin_vel)
        else:
            self._send_vel_robot()
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            self._head_towards_goal()    

    def set_goal(self, x, y, theta):
        self._goal.x = x
        self._goal.y = y
        self._goal.theta = theta

    
    def _send_vel_robot(self, vel_ang = 0.0, vel_lin = 0.0):
        cmd_twist =  Twist()
        cmd_twist.linear.x = vel_lin
        cmd_twist.angular.z = vel_ang

        self._cmdvel_pub(cmd_twist)

    def start(self):
        self._go_staight()    

if __name__ == "__main__":
    tb3_go2point = GoToPoint()
    tb3_go2point.set_goal(2, 3.5, 0)
    tb3_go2point.start()
    rospy.spin()