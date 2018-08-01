#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SpawnModel, DeleteModel
import time
from tf_conversions import posemath, toMsg
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from threading import Timer
import PyKDL

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_coke', anonymous=True)

arm_commander = moveit_commander.MoveGroupCommander("arm")
arm_commander.set_planning_time(5)
arm_commander.set_goal_tolerance(0.1)

hand_commander = moveit_commander.MoveGroupCommander("hand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


#Service for spawn model
# rospy.wait_for_service("gazebo/get_model_state")
mcoord_fetch = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

#switch controller
# rospy.wait_for_service('/controller_manager/switch_controller')
__switch_ctrl = rospy.ServiceProxy('/vs060/controller_manager/switch_controller', SwitchController)

#pause physics
# rospy.wait_for_service("/gazebo/pause_physics")
__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

#unpause phyiscs
__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)


#set model service
# rospy.wait_for_service("/gazebo/set_model_configuration")
__set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

#reset world
__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

#delete model
__delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

#spawn model
__spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)


PATH_TO_MODELS = "/home/aquifi/.gazebo/models/"
MODEL_NAME = 'coke_can'

def __start_ctrl():
    rospy.loginfo("STARTING CONTROLLERS")
    __switch_ctrl.call(start_controllers=["hand_position_trajectory_controller", "arm_position_trajectory_controller", "joint_state_controller"],
                            stop_controllers=[], 
                            strictness=SwitchControllerRequest.BEST_EFFORT)


def spawn_extras():
    try:
        __delete_model(MODEL_NAME)
    except:
        rospy.logwarn("Failed to delete model: ", MODEL_NAME)
    sdf = None
    initial_pose = Pose()
    initial_pose.position.y = -0.5
    with open(PATH_TO_MODELS +  MODEL_NAME + "/model.sdf", "r") as model:
        sdf = model.read()
    res = __spawn_model(MODEL_NAME, sdf, "", initial_pose, "world")
    rospy.loginfo(res)



def reset_world():
    """
    Resets the object poses in the world and the robot joint angles.
    """
    __switch_ctrl.call(start_controllers=[],
                            stop_controllers=["hand_position_trajectory_controller", "arm_position_trajectory_controller", "joint_state_controller"],
                            strictness=SwitchControllerRequest.BEST_EFFORT)
    __pause_physics.call()

    joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'flange','H1_F1J1', 'H1_F1J2',
    'H1_F1J3', 'H1_F2J1', 'H1_F2J2', 'H1_F2J3','H1_F3J1', 'H1_F3J2', 'H1_F3J3']
    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

    __set_model.call(model_name="denso",
                            urdf_param_name="robot_description",
                            joint_names=joint_names,
                            joint_positions=joint_positions)

    timer = Timer(0.0, __start_ctrl)
    timer.start()

    time.sleep(0.1)
    __unpause_physics.call()

    #__reset_world.call()
    spawn_extras()

def get_object_pose():
    name = "coke_can"
    body_name = "link"
    resp_coke = mcoord_fetch(name, body_name) 
    return resp_coke


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

def move_tip_absolute(target):
    arm_commander.set_start_state_to_current_state()
    arm_commander.set_pose_targets([target])
    plan = arm_commander.plan()
    if not arm_commander.execute(plan):
        return False
    return True

def get_tip_pose():
    return arm_commander.get_current_pose(arm_commander.get_end_effector_link()).pose

def move_tip(x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
    """
    Moves the tooltip in the world frame by the given x,y,z / roll,pitch,yaw.
    @return True on success
    """
    transform = PyKDL.Frame(PyKDL.Rotation.RPY(pitch, roll, yaw),
                            PyKDL.Vector(-x, -y, -z))

    tip_pose = get_tip_pose()
    tip_pose_kdl = posemath.fromMsg(tip_pose)
    final_pose = toMsg(tip_pose_kdl * transform)

    arm_commander.set_start_state_to_current_state()
    arm_commander.set_pose_targets([final_pose])
    plan = arm_commander.plan()
    if not arm_commander.execute(plan):
        return False
    return True


def pick():
    rospy.loginfo("Moving to Pregrasp")
    open_hand()
    time.sleep(0.1)

    coke_pose = get_object_pose().pose
    coke_pose.position.z += 0.55

    # setting an absolute orientation  (from the top)
    quaternion = quaternion_from_euler(0, pi, 0)
    coke_pose.orientation.x = quaternion[0]
    coke_pose.orientation.y = quaternion[1]
    coke_pose.orientation.z = quaternion[2]
    coke_pose.orientation.w = quaternion[3]

    print('Coke pose: ', coke_pose)
    rospy.loginfo("---Moving to Coke---")
    move_tip_absolute(coke_pose)
    time.sleep(0.1)

    rospy.loginfo("---Moving Down---")
    move_tip(z=-0.1)

    rospy.loginfo("---Grasping---")
    close_hand()

    rospy.loginfo("---Lifting---")
    time.sleep(2)
    for _ in range(5):
        move_tip(z=0.05)
        time.sleep(0.1)

if __name__ == '__main__':
    rospy.loginfo('Starting')
    spawn_extras()
    pick()
    reset_world()