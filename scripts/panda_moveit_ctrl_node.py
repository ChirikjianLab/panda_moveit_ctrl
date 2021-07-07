#! /usr/bin/python

# ROS node to publish the service
# Author: Hongtao Wu
# Johns Hopkins University
# National University of Singapore
# Date: Mar 05, 2021

import rospy
from panda_moveit_ctrl.panda_robot import PandaRobot

if __name__ == "__main__":
    rospy.init_node("panda_move")

    load_gripper = True
    
    robot = PandaRobot(load_gripper=load_gripper)