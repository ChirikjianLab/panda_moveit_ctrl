# Class to provide service to control Panda with MoveIt
# Author: Hongtao Wu, Johns Hopkins University
# Jan 21, 2021

from __future__ import print_function

import numpy as np

import rospy
import tf2_ros

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import franka_interface

from panda_moveit_ctrl.srv import *

class PandaRobotService(object):
 
    def __init__(self, 
                 vel, 
                 acc, 
                 ee="panda_hand", 
                 move_joint=True, 
                 move_cartesian=True, 
                 load_gripper=True
        ):
        """Class used to set up service to interact with the Panda robot.
        move_joint and move_cartesian are set true if you want to
        move the robot by joint position and cartesian pose
        """

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = self.group.get_planning_frame()
        rospy.loginfo("============ Reference frame: %s" % planning_frame)

        self.group.set_end_effector_link(ee)
        eef_link = self.group.get_end_effector_link()
        rospy.loginfo("============ End effector: %s" % eef_link)

        group_names = self.robot.get_group_names()
        rospy.loginfo("============ Robot Groups: %s" % group_names)
        
        # Set up velocity and acceleration
        self.group.set_max_velocity_scaling_factor(vel)
        self.group.set_max_acceleration_scaling_factor(acc)
        
        # Set up tolerance
        self.joint_tol = 0.0001
        self.orn_tol   = 0.001
        self.pos_tol   = 0.001
        self.group.set_goal_joint_tolerance(self.joint_tol)
        self.group.set_goal_orientation_tolerance(self.orn_tol)
        self.group.set_goal_position_tolerance(self.pos_tol)

        # Set up MoveToJoint server
        if move_joint:
            self.setupMoveToJointServer()
            rospy.sleep(0.5)
        
        # Set up MoveToCartesian server
        if move_cartesian:
            self.setupMoveToCartesianServer()
            rospy.sleep(0.5)

        # Set up the gripper
        if load_gripper:
            gripper_group_name = "hand"
            self.configure_gripper(gripper_group_name)
            # self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
            rospy.loginfo("Gripper loaded")
            self.setupGripperServer()
            rospy.sleep(0.5)
        
        rospy.loginfo("Ready!")
            
    def configure_gripper(self, gripper_joint_names):
        """Initialize gripper."""

        self.gripper = franka_interface.GripperInterface()
        if not self.gripper.exists:
            self.gripper = None
            return
    
    def gripper_state(self):
        """Return Gripper state {'position', 'force'}. 
        Only available if Franka gripper is connected.

        Returns:
            gripper_state (dict): dict of position and force
        """
        gripper_state = {}

        if self.gripper:
            gripper_state['position'] = self.gripper.joint_ordered_positions()
            gripper_state['force'] = self.gripper.joint_ordered_efforts()

        return gripper_state

    def setupGripperServer(self):
        rospy.loginfo("Setting up Gripper server...")
        self.gripper_server = rospy.Service("move_gripper", MoveGripper, self.handleMoveGripper)
        rospy.loginfo("Finish setting up Gripper server...")
    
    def handleMoveGripper(self, req):
        """Move Gripper

        Args:
            req.width: float
        """

        rospy.loginfo("move_gripper service receive: %s", req.width)
        self.gripper.move_joints(req.width)
        rospy.loginfo(self.gripper_state())
        rospy.sleep(0.2)
        rospy.loginfo(self.gripper_state())

        return MoveGripperResponse("Successfully move gripper!")

    def setupMoveToJointServer(self):
        rospy.loginfo("Setting up MoveToJoint server...")
        self.move_joint_server = rospy.Service("move_to_joint", MoveToJoint, self.handleMoveToJoint)
        rospy.loginfo("Finish setting up MoveToJoint server...")
        
    def handleMoveToJoint(self, req):
        """Move to joint position
        
        Args:
            req.joint_config (7, numpy array) joint position
        """
        
        rospy.loginfo("move_to_joint service receive: %s", req.joint_config.data)
        move_success = self.group.go(req.joint_config.data, wait=True)
        self.group.stop()

        rospy.sleep(0.5)

        if move_success:
            success = 1
            rospy.loginfo("Success in cartesian motion...")
        else:
            success = 0
            rospy.loginfo("Failure in cartesian motion...")

        return MoveToJointResponse(success)

    def setupMoveToCartesianServer(self):
        rospy.loginfo("Setting up MoveToCartesian server...")
        self.move_catesian_server = rospy.Service("move_to_cartesian", MoveToCartesian, self.handleMoveToCartesian)
        rospy.loginfo("Finish setting up MoveToCartesian server...")    

    def handleMoveToCartesian(self, req):
        """Move to cartesian position
        
        Args:
            req.pos (3, numpy array) position of end effector
            req.quat (4, numpy array) quaternion (x, y, z, w)
        """

        rospy.loginfo("move_to_cartesian receive quat: %s", req.quat.data)
        rospy.loginfo("move_to_cartesian receive pos: %s", req.pos.data)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = req.quat.data[0]
        pose_goal.orientation.y = req.quat.data[1]
        pose_goal.orientation.z = req.quat.data[2]
        pose_goal.orientation.w = req.quat.data[3]
        pose_goal.position.x = req.pos.data[0]
        pose_goal.position.y = req.pos.data[1]
        pose_goal.position.z = req.pos.data[2]

        self.group.set_pose_target(pose_goal)
        move_success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        rospy.sleep(0.5)

        if move_success:
            success = 1
            rospy.loginfo("Success in cartesian motion...")
        else:
            success = 0
            rospy.loginfo("Failure in cartesian motion...")

        return MoveToCartesianResponse(success)