#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('init_grasper', anonymous=True)

hand_commander = moveit_commander.MoveGroupCommander("hand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


def open_hand():
    hand_commander.set_named_target("open")
    plan = hand_commander.plan()
    if not hand_commander.execute(plan, wait=True):
        return False   
    return True


def close_hand():
    hand_commander.set_named_target("closed")
    plan = hand_commander.plan()
    if not hand_commander.execute(plan, wait=True):
        return False
    return True



while(True):
    print("----------Opening hand------------")
    open_hand()

    print("----------Closing hand-------------")
    close_hand()
    rospy.sleep(3)
