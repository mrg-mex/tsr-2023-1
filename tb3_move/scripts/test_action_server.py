#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from tb3_move.msg import TB3MoveAction, TB3MoveGoal
import sys

def get_state_name(state_code):
    state_name = ''
    if state_code == 0: state_name = 'PENDING'
    if state_code == 1: state_name = 'ACTIVE'
    if state_code == 2: state_name = 'PREEMPTED'
    if state_code == 3: state_name = 'SUCCEEDED'
    if state_code == 4: state_name = 'ABORTED'
    if state_code == 5: state_name = 'REJECTED'
    if state_code == 6: state_name = 'PREEMPTING'
    if state_code == 7: state_name = 'RECALLING'
    if state_code == 8: state_name = 'RECALLED'
    if state_code == 9: state_name = 'LOST'

    return state_name


if __name__ == '__main__':
    param_x = float(sys.argv[1])
    param_y = float(sys.argv[2])

    rospy.init_node('tb3_test_client_node')
    # Inicializamos el cliente de Actionlib
    tb3_move_client = actionlib.SimpleActionClient('TB3_Move_server', TB3MoveAction)
    new_goal = TB3MoveGoal()
    # Valores de los campos de std_msgs/Header
    new_goal.header.seq = 1
    new_goal.header.frame_id = 'map'
    new_goal.header.stamp = rospy.Time().now()
    # Valores de los campos de geometry_msgs/Pose2D
    new_goal.target.x = param_x
    new_goal.target.y = param_y
    new_goal.target.theta = 0

    rospy.loginfo("Connecting to server...")
    if tb3_move_client.wait_for_server():
        rospy.loginfo("Done!")
        result = tb3_move_client.send_goal_and_wait(new_goal, rospy.Duration(60))
        rospy.loginfo(f"Completed ['{get_state_name(result)}' ({result})].")
    else:
        rospy.logwarn("FAIL: ActionServer connection timeout!")