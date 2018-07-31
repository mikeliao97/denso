#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


while(True):
    rand_pose = group.get_random_pose()
    group.set_pose_target(rand_pose)

    plan = group.plan()
    group.go(wait=True)
    print("----------Executing a new Pose------------")
    print('Pose: ', rand_pose)
    rospy.sleep(3)