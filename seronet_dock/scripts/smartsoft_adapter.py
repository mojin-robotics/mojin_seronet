#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from cob_actions.msg import SetStringAction, SetStringGoal

class SmartsoftAdapter(object):
    def __init__(self):
        rospy.Subscriber("/docker_control/dock_seronet/goal", String, self.dock_callback)
        rospy.Subscriber("/docker_control/undock_seronet/goal", String, self.undock_callback)

        self.pub_dock_result_ = rospy.Publisher('/docker_control/dock_seronet/result', String, queue_size=1)
        self.pub_undock_result_ = rospy.Publisher('/docker_control/undock_seronet/result', String, queue_size=1)

        self.ac_dock_ = actionlib.SimpleActionClient('/docker_control/dock', SetStringAction)
        self.ac_undock_ = actionlib.SimpleActionClient('/docker_control/undock', SetStringAction)


    def dock_callback(self, msg):
        print ("docking to {}".format(msg.data))
        message = String()
        if not self.ac_dock_.wait_for_server(rospy.Duration(1)):
            message.data = 'WARN: Dock not available'
            self.pub_dock_result_.publish(message)
            return
        goal = SetStringGoal()
        goal.data = msg.data
        goal_state = self.ac_dock_.send_goal_and_wait(goal)
        if goal_state == GoalStatus.SUCCEEDED:
            message.data = 'INFO: Dock successful'
        else:
            message.data = 'WARN: Dock failed'

        print (message.data)
        self.pub_dock_result_.publish(message)
        return

    def undock_callback(self, msg):
        print ("undocking from {}".format(msg.data))
        message = String()
        if not self.ac_undock_.wait_for_server(rospy.Duration(1)):
            message.data = 'WARN: Undock not available'
            self.pub_undock_result_.publish(message)
            return
        goal = SetStringGoal()
        goal.data = msg.data
        goal_state = self.ac_undock_.send_goal_and_wait(goal)
        if goal_state == GoalStatus.SUCCEEDED:
            message.data = 'INFO: Undock successful'
        else:
            message.data = 'WARN: Undock failed'

        print (message.data)
        self.pub_undock_result_.publish(message)
        return

if __name__ == '__main__':
    rospy.init_node('docking_smartsoft_adapter')
    SmartsoftAdapter()
    rospy.loginfo("docking_smartsoft_adapter started")
    rospy.spin()
