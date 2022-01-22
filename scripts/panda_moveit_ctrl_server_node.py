#! /usr/bin/python

# ROS node to start the service for controlling the robot
# Author: Hongtao Wu
# Date: Mar 05

import rospy
from panda_moveit_ctrl.panda_robot_service import PandaRobotService

if __name__ == "__main__":
    rospy.init_node("panda_robot_service")

    vel = 0.3
    acc = 0.3
    load_gripper = True
    ee = "panda_link8"
    PRS = PandaRobotService(vel, acc, ee=ee, load_gripper=load_gripper)

    
    rospy.spin()