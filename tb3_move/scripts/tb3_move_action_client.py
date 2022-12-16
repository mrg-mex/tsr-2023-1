#! /usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from tb3_move.msg import TB3MoveAction, TB3MoveGoal
import sys

class TB3MoveActionClient():
    def __init__(self):
        self._goal = TB3MoveGoal()
        self._action_client = actionlib.SimpleActionClient('TB3_Move_server', TB3MoveAction)
        rospy.loginfo("TB3MoveActionClient: Inicializada...")

    
    def send_goal(self, x, y, preempt_timeout = 0.0):
        self._goal.header.seq = 1
        self._goal.header.frame_id = 'map'
        self._goal.header.stamp = rospy.Time().now()
        # Valores de los campos de geometry_msgs/Pose2D
        self._goal.target.x = x
        self._goal.target.y = y
        self._goal.target.theta = 0

        rospy.loginfo("Connecting to server...")
        self._action_client.wait_for_server()
        # actionlib_client.send_goal_and_wait(new_goal, rospy.Duration())
        # actionlib_client.send_goal(new_goal, [callback funtions]) + wait_for_result(rospy.Duration())
        rospy.loginfo("Sending new goal:")
        self._action_client.send_goal(self._goal, self._on_done, self._on_active, self._on_feedback)
        rospy.loginfo(f"New goal message:\n{self._goal}")
        return self._action_client.wait_for_result(rospy.Duration(preempt_timeout))

    def _on_active(self):
        rospy.loginfo("ActionClient: New goal active!")
        state = self._action_client.get_state()
        rospy.loginfo(f"Goal [{self.get_state_name(state)}]: {self._action_client.get_goal_status_text()}")

    def _on_feedback(self, feedback_msg):
        state = self._action_client.get_state()
        formated_msg = ("GOAL [{} - {}]: current position({:.6f}, {:.6f}, {:.6f}), DError({:.6f}), YError({:.6f})."
            .format(
                self.get_state_name(state),
                feedback_msg.state_name,
                feedback_msg.position.x,
                feedback_msg.position.y,
                feedback_msg.position.theta,
                feedback_msg.distance_error,
                feedback_msg.yaw_error
            )
        )
        rospy.loginfo(formated_msg)

    def _on_done(self, success_code, msg):
        elapsed_time = msg.header.stamp - self._goal.header.stamp
        formated_msg = ("Proceso terminado con estatus [{}] '{}': posicion final({:.6f}, {:.6f}), theta: {:.6f}, Derr:({:.6f}), Yerr:({:.6f})"
            .format(
                self.get_state_name(success_code),
                msg.state_name,
                msg.position.x,
                msg.position.y,
                msg.position.theta,
                msg.distance_error,
                msg.yaw_error
            )
        )
        status_msg = f"{msg.goal_message} terminado con estado {self.get_state_name(success_code)}, elapsed time {elapsed_time.to_sec():.6f}"

        if success_code == GoalStatus.SUCCEEDED:
            rospy.loginfo(status_msg)
            rospy.loginfo(formated_msg)
        else:
            rospy.logwarn(status_msg)
            rospy.logwarn(formated_msg)       

    def cancel_goal(self):
        self._action_client.cancel_goal()


    def get_state_name(self, state_code):
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


def main():
    param_x = float(sys.argv[1])
    param_y = float(sys.argv[2])

    rospy.init_node('tb3_actionlib_client_node') 
    tb3_move_client = TB3MoveActionClient()

    rospy.loginfo(f"Goal({param_x:.6f}, {param_y:.6f})")

    goal_completed = tb3_move_client.send_goal(param_x, param_y) # Sin tomar en cuenta el tiempo = timeout
    # goal_completed = tb3_move_client.send_goal(param_x, param_y, 60) Con restriccion de timeout

    if not goal_completed:
        rospy.logerr(f"PROCCESS FAIL: {tb3_move_client._action_client.get_goal_status_text()}")
        tb3_move_client.cancel_goal()
        rospy.logerr("Goal cancelada")
        sys.exit(1)
    


if __name__ == '__main__':
    main()