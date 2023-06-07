# Class to provide service to control Panda with MoveIt
# Author: Hongtao Wu, Johns Hopkins University
# Jan 21, 2021

from __future__ import print_function
from ntpath import join

import numpy as np
import os

import rospy
import tf2_ros

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import franka_msgs.msg
import franka_interface
from franka_tools.collision_behaviour_interface import CollisionBehaviourInterface

from panda_moveit_ctrl.srv import *

class PandaRobotService(object):
 
    def __init__(self, 
                 vel, 
                 acc, 
                 ee="panda_hand", 
                 move_joint=True, 
                 move_cartesian=True, 
                 load_gripper=True,
                 force=True,
                 joint_config=True,
                 doc_dir=None
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

        # Set up GetEEXform server
        self.setupGetEEXformServer()

        # Set up the gripper
        if load_gripper:
            gripper_group_name = "hand"
            self.configure_gripper(gripper_group_name)
            # self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
            rospy.loginfo("Gripper loaded")
            self.setupGripperServer()
            self.setupCloseGripperServer()
            rospy.sleep(0.5)
        
        # Set up external force subscriber
        self.ext_force = []
        self.ee_force = []
        self.save_force = None
        self.ee_force_list = []
        self.img_idx = None
        if force:
            # self.franka_state_sub = rospy.Subscriber("/franka_state_controller/franka_states", franka_msgs.msg.FrankaState, self.franka_state_sub_cb)
            self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped, self.ext_force_sub_cb)
            self.save_ee_force_sub = rospy.Subscriber("save_ee_force", std_msgs.msg.Int32, self.save_ee_force_sub_cb)
            self.setupGetEEForceServer()
            self.setupGetSaveEEForceServer()
        if doc_dir:
            self.img_dir = os.path.join(doc_dir, "img")
            if os.path.exists(self.img_dir):
                os.rmdir(self.img_dir)
            os.mkdir(self.img_dir)

        # Set up joint status subscriber
        self.joint_config = []
        self.joint_config_list = []
        self.save_joint_config = None
        if joint_config:
            self.joint_config_sub = rospy.Subscriber("/franka_state_controller/joint_states", sensor_msgs.msg.JointState, self.joint_config_sub_cb)
            self.save_joint_config_sub = rospy.Subscriber("save_joint_config", std_msgs.msg.Int32, self.save_joint_config_sub_cb)
            self.setupGetJointConfigServer()
            self.setupGetSaveJointConfigServer()
        # Set up TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.base_frame = "panda_link0"
        self.ee_frame   = ee

        # Set up collision behavior
        # self.CB = CollisionBehaviourInterface()
        # joint_torques = [22.0, 22.0, 20.0, 20.0, 18.0, 16.0, 14.0]
        # cartesian_forces = [22.0, 22.0, 22.0, 27.0, 27.0, 27.0]
        # self.CB.set_collision_threshold(joint_torques, cartesian_forces)
        
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
    
    def finger_position(self):
        """Return average finger position
        
        Returns:
            finger_position (float): average position of right and left finger"""

        finger_position_list = []
        finger_position = None

        for i in range(3):
            gripper_state = self.gripper_state()
            finger_position_list.append(gripper_state['position'])
            rospy.sleep(0.1)
        
        finger_position_list = np.array(finger_position_list).flatten()
        finger_position = np.average(finger_position_list)
        
        return finger_position

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
        self.gripper.stop_action()
        rospy.sleep(0.1)
        self.gripper.move_joints(req.width)
        rospy.sleep(0.5)

        finger_position = self.finger_position()
        rospy.loginfo("Finger position: %s", finger_position)
        
        return finger_position

    def setupCloseGripperServer(self):
        rospy.loginfo("Setting up CloseGripper server...")
        self.close_gripper_server = rospy.Service("close_gripper", CloseGripper, self.handleCloseGripper)
        rospy.loginfo("Finish setting up CloseGripper server...")
    
    def handleCloseGripper(self, req):
        """Close Gripper
        """

        rospy.loginfo("close gripper")
        self.gripper.move_joints(0, wait_for_result=False)
        # self.gripper.grasp(0, 100)
        rospy.sleep(3)
        # self.gripper.stop_action()

        finger_position = self.finger_position()
        rospy.loginfo("Finger position: %s", finger_position)
        
        return finger_position

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
    
    def setupGetEEXformServer(self):
        rospy.loginfo("Setting up GetEEXform server...")
        self.GetEEXformServer = rospy.Service("get_ee_xform", GetEEXform, self.handleGetEEXform)
        rospy.loginfo("Finish setting up GetEEXform server")
    
    def handleGetEEXform(self, req):
        """Get EEXform

        Returns:
            pos (list of 3 float): position
            quat(list of 4 float): quaternion [w, x, y, z]
        """

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
    
    #################################
    # EE force
    #################################
    def setupGetEEForceServer(self):
        rospy.loginfo("Setting up GetEEForce server...")
        self.GetEEForceServer = rospy.Service("get_ee_force", GetEEForce, self.handleGetEEForceServer)
        rospy.loginfo("Finish setting up GetEEForce server")
    
    def handleGetEEForceServer(self, req):
        if len(self.ext_force) == 0:
            return [], [], []
        
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
        ee_force = np.matmul(self.quat2rotm(quat), self.ext_force)
        rospy.loginfo("[Panda robot] F_ext: {}".format(self.ext_force))
        rospy.loginfo("[Panda robot] F_ee: {}".format(ee_force))

        return ee_force, pos, quat
    
    def setupGetSaveEEForceServer(self):
        rospy.loginfo("Setting up StartSaveEEForce server...")
        self.GetSaveEEForceServer = rospy.Service("get_save_ee_force", GetSaveEEForce, self.handleGetSaveEEForceServer)
        rospy.loginfo("Finish setting up GetEEForce server")

    def handleGetSaveEEForceServer(self, req):
        self.save_force = False
        # import ipdb; ipdb.set_trace()
        self.save_ee_status_list(req.txt, self.ee_force_list, self.ee_pos_list)
        return self.img_dir

    def ext_force_sub_cb(self, msg):
        self.ext_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        if self.save_force:
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

            ee_force = np.matmul(self.quat2rotm(quat), self.ext_force)
            self.ee_force_list.append(ee_force)
            self.ee_pos_list.append(pos)

            if self.img_idx is not None:
                self.get_cam_image(self.img_dir, self.img_idx)
    
    def save_ee_force_sub_cb(self, msg):
        if msg.data == 1:
            rospy.loginfo("Start saving ee force...")
            self.save_force = True
            self.ee_force_list = []
            self.ee_pos_list = []
            self.img_idx = 0
            if os.path.exists(self.img_dir):
                os.rmdir(self.img_dir)
            os.mkdir(self.img_dir)
    
    def get_cam_image(self, data_dir, id):
        try:
            get_cam_image_client = rospy.ServiceProxy(
                'get_cam_image', 
                GetCamImage)
            rgb_path = os.path.join(data_dir, "rgb_%i.png" % id)
            depth_path = os.path.join(data_dir, "depth_%i.png" % id)
            pc_path = os.path.join(data_dir, "pc_%i.ply" % id)
            resp = get_cam_image_client(rgb_path, depth_path, pc_path)

        except rospy.ServiceException as e:
            rospy.loginfo("[Main] Service call failed: {}".format(e))
    
    # def franka_state_sub_cb(self, msg):
    #     f_x_cload = msg.F_x_Cload
    #     f_x_ctotal = msg.F_x_Ctotal
    #     print(msg)
    #################################
    # Joint config
    #################################

    def joint_config_sub_cb(self, msg):
        joint_config = msg.position
        self.joint_config = joint_config[:7]
        # [joint_config[i] for i in range(7)]

        if self.save_joint_config:
            self.joint_config_list.append(self.joint_config)

    def save_joint_config_sub_cb(self, msg):
         if msg.data == 1:
            rospy.loginfo("Start saving joint config...")
            self.save_joint_config = True
            self.joint_config_list = []
    
    def setupGetJointConfigServer(self):
        rospy.loginfo("Setting up GetJointConfig server...")
        self.GetJointConfigServer = rospy.Service("get_joint_config", GetJointConfig, self.handleGetJointConfigServer)
        rospy.loginfo("Finish setting up GetJointConfig server")
    
    def handleGetJointConfigServer(self, req):
        if len(self.joint_config) == 0:
            return []
        # import ipdb; ipdb.set_trace()
        return [self.joint_config]
    
    def setupGetSaveJointConfigServer(self):
        rospy.loginfo("Setting up StartSaveJointConfig server...")
        self.GetSaveJointConfigServer = rospy.Service("get_save_joint_config", GetSaveJointConfig, self.handleGetSaveJointConfigServer)
        rospy.loginfo("Finish setting up GetSaveJointConfig server")

    def handleGetSaveJointConfigServer(self, req):
        self.save_joint_config = False
        # import ipdb; ipdb.set_trace()
        self.save_joint_config_list(req.txt, self.joint_config_list)
        return GetSaveJointConfigResponse("Success")


    @staticmethod
    def quat2rotm(quat):
        """Quaternion to rotation matrix.

        Args:
            quat (4, numpy array): quaternion w, x, y, z
        Returns:
            rotm: (3x3 numpy array): rotation matrix
        """
        w = quat[0]
        x = quat[1]
        y = quat[2]
        z = quat[3]

        s = w * w + x * x + y * y + z * z

        rotm = np.array([
        [1 - 2 * (y * y + z * z) / s, 2 * (x * y - z * w) / s, 2 * (x * z + y * w) / s],
        [2 * (x * y + z * w) / s, 1 - 2 * (x * x + z * z) / s, 2 * (y * z - x * w) / s],
        [2 * (x * z - y * w) / s, 2 * (y * z + x * w) / s, 1 - 2 * (x * x + y * y) / s]])

        return rotm

    @staticmethod
    def save_ee_status_list(txt, force_list, pos_list):
        if os.path.exists(txt):
            os.remove(txt)
        
        with open(txt, 'w') as f:
            if len(force_list) == len(pos_list):
                for i in range(len(force_list)):
                    pos = pos_list[i]
                    force = force_list[i]
                    f.write(
                        "{:.6e}".format(pos[0]) + " " +
                        "{:.6e}".format(pos[1]) + " " +
                        "{:.6e}".format(pos[2]) + "\n"
                    )
                    f.write(
                        "{:.6e}".format(force[0]) + " " +
                        "{:.6e}".format(force[1]) + " " +
                        "{:.6e}".format(force[2]) + "\n"
                    )
    
    @staticmethod
    def save_joint_config_list(txt, joint_config_list):
        if os.path.exists(txt):
            os.remove(txt)
        
        with open(txt, 'w') as f:
            for i in range(len(joint_config_list)):
                joint_config = joint_config_list[i]
                for j in range(len(joint_config)):
                    f.write(
                        "{:.6e}".format(joint_config[j]) + " " 
                    )
                f.write( "\n")
