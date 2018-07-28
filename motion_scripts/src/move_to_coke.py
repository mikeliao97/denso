#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


#Service for spawn model
rospy.wait_for_service("gazebo/get_model_state")
mcoord_fetch = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

name = "coke_can"
body_name = "link"
resp_coke = mcoord_fetch(name, body_name) 

rob_pose = group.get_current_pose()
print("Robot Pose", rob_pose)
print("Coke Pose", resp_coke)
print("Random Pose", group.get_random_pose())

#Move the robot arm to the coke response
group.set_pose_target(resp_coke.pose)
group.set_planning_time(10)
group.set_goal_tolerance(0.3)
plan = group.plan()
group.go(wait=True)