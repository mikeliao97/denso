#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import SpawnModel


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


#Service for spawn model
rospy.wait_for_service("gazebo/spawn_sdf_model")
spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

rob_pose = groupt.get_current_pose()
print("Reference frame", group.get_planning_frame())

print("Current Pose:", group.get_current_pose())
print("Robot State", robot.get_current_state())
print("--------------------------------------------------")
print("--------------------------------------------------")
rospy.sleep(3)
