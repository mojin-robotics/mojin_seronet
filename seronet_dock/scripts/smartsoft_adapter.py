#!/usr/bin/env python

import actionlib
import rospy

from actionlib_msgs.msg import GoalStatus
from cob_actions.msg import DockAction, DockGoal
from cob_msgs.msg import PowerState
from std_msgs.msg import String, Bool

class SmartsoftAdapter(object):
    def __init__(self):
        rospy.Subscriber("/docker_control/dock_seronet/goal", String, self.dock_callback)
        rospy.Subscriber("/docker_control/undock_seronet/goal", String, self.undock_callback)
        rospy.Subscriber("/charging_seronet", Bool, self.charging_callback)

        self.pub_dock_result_   = rospy.Publisher('/docker_control/dock_seronet/result', String, queue_size=1)
        self.pub_undock_result_ = rospy.Publisher('/docker_control/undock_seronet/result', String, queue_size=1)
        self.pub_power_state_   = rospy.Publisher('/power_state_seronet', PowerState, queue_size=1)

        self.ac_dock_   = actionlib.SimpleActionClient('/docker_control/dock_poses', DockAction)
        self.ac_undock_ = actionlib.SimpleActionClient('/docker_control/undock_poses', DockAction)


    def dock_callback(self, msg):
        rospy.loginfo("SmartsoftAdapter: docking to %s", str(msg.data))
        message = String()
        if not self.ac_dock_.wait_for_server(rospy.Duration(1)):
            message.data = 'WARN: Dock not available'
            self.pub_dock_result_.publish(message)
            return
        goal = DockGoal()
        goal.frame_id = msg.data
        goal_state = self.ac_dock_.send_goal_and_wait(goal)
        if goal_state == GoalStatus.SUCCEEDED:
            message.data = 'INFO: Dock successful'
        else:
            message.data = 'WARN: Dock failed'

        rospy.loginfo(message.data)
        self.pub_dock_result_.publish(message)
        return

    def undock_callback(self, msg):
        rospy.loginfo("SmartsoftAdapter: undocking from %s", str(msg.data))
        message = String()
        if not self.ac_undock_.wait_for_server(rospy.Duration(1)):
            message.data = 'WARN: Undock not available'
            self.pub_undock_result_.publish(message)
            return
        goal = DockGoal()
        goal.frame_id = msg.data
        goal_state = self.ac_undock_.send_goal_and_wait(goal)
        if goal_state == GoalStatus.SUCCEEDED:
            message.data = 'INFO: Undock successful'
        else:
            message.data = 'WARN: Undock failed'

        rospy.loginfo(message.data)
        self.pub_undock_result_.publish(message)
        return

    def charging_callback(self, msg):
        power_state = PowerState()
        power_state.header.stamp = rospy.Time.now()
        power_state.charging = msg.data
        self.pub_power_state_.publish(power_state)

if __name__ == '__main__':
    rospy.init_node('docking_smartsoft_adapter')
    SmartsoftAdapter()
    rospy.loginfo("docking_smartsoft_adapter started")
    rospy.spin()
