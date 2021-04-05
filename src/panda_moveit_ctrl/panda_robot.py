# Class to interact with Panda
# Author: Hongtao Wu, Johns Hopkins University
# Jan 25, 2021

from __future__ import print_function

import numpy as np

import rospy
import tf2_ros

import std_msgs.msg 

from panda_moveit_ctrl.srv import *

class PandaRobot(object):
    """
    Class used to interact with the Panda robot
    """
    def __init__(self, base_frame="panda_link0", ee_frame="panda_hand", goHome=True, load_gripper=False):

        rospy.loginfo("Start setting up the panda robot...")

        self.home_config = [0.0, -np.pi/4, 0.0, -2*np.pi/3, 0.0, np.pi/3, np.pi/4]
        self.monitor_config = [1.86361,-0.224915,0.0542945,-0.419028,-0.00761639,2.47066,0.888406]

        # Set up MoveToJoint client
        self.setupMoveToJointClient()

        # Set up MoveToCartesian client
        self.setupMoveToCartesianClient()

        if goHome:
            # Move robot home
            self.goHome()
        else:
            self.goMonitor()

        # Set up the gripper client
        if load_gripper:
            self.open_width = 0.04
            self.close_width = 0.01
            self.setupMoveGripperClient()
            rospy.sleep(0.5)

            self.open_gripper()
            self.close_gripper()
            self.open_gripper()

        # Set up TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.base_frame = base_frame
        self.ee_frame   = ee_frame

        rospy.loginfo("Finish setting up the panda robot...")
    
    def setupMoveToJointClient(self):
        rospy.loginfo("Start setting up MoveToJoint client...")
        rospy.wait_for_service("move_to_joint")
        self.move_joint_client = rospy.ServiceProxy("move_to_joint", MoveToJoint)
        rospy.loginfo("Finish setting up MoveToJoint client...")

    def setupMoveToCartesianClient(self):
        rospy.loginfo("Start setting up MoveToCartesian client...")
        rospy.wait_for_service("move_to_cartesian")
        self.move_cartesian_client = rospy.ServiceProxy("move_to_cartesian", MoveToCartesian)
        rospy.loginfo("Finish setting up MoveToCartesian client...")

    def setupMoveGripperClient(self):
        rospy.loginfo("Start setting up MoveGripper client...")
        rospy.wait_for_service("move_gripper")
        self.move_gripper_client = rospy.ServiceProxy("move_gripper", MoveGripper)
        rospy.loginfo("Finish setting up MoveGripper client...")

    def moveToJointPosition(self, joint_config):
        """
        Call the move_to_joint client to move the robot
        
        joint_config (list of 7): joint configuration
        """
        ros_joint_config = std_msgs.msg.Float64MultiArray()
        ros_joint_config.data = joint_config
        try:
            resp = self.move_joint_client(ros_joint_config)
        except rospy.ServiceException, e:
            rospy.login("Service call failed: %s" % e)

    def moveToCartesianPosition(self, pos, quat):
        """
        Call the move_to_cartesian client to move the robot
        
        pos (list of 7): posiiton
        quat (list of 7): quaternion
        """
        ros_pos  = std_msgs.msg.Float64MultiArray()
        ros_quat = std_msgs.msg.Float64MultiArray()
        ros_pos.data  = pos
        ros_quat.data = quat
        try:
            resp = self.move_cartesian_client(ros_pos, ros_quat)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def goHome(self):
        self.moveToJointPosition(self.home_config)
    
    def goMonitor(self):
        self.moveToJointPosition(self.monitor_config)

    def getEEXform(self):
        xform = self.tfBuffer.lookup_transform(self.base_frame, self.ee_frame, rospy.Time())
        
        pos  = np.array([0.0, 0.0, 0.0])
        quat = np.array([0.0, 0.0, 0.0, 0.0])

        pos[0] = xform.transform.translation.x
        pos[1] = xform.transform.translation.y
        pos[2] = xform.transform.translation.z

        quat[0] = xform.transform.rotation.w
        quat[1] = xform.transform.rotation.x
        quat[2] = xform.transform.rotation.y
        quat[3] = xform.transform.rotation.z

        return pos, quat

    def open_gripper(self):
        try:
            resp = self.move_gripper_client(self.open_width)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
    
    def close_gripper(self):
        try:
            resp = self.move_gripper_client(self.close_width)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)