#! /usr/bin/python

# ROS node to start the service for controlling the robot
# Author: Hongtao Wu
# Date: Mar 05

import rospy
from panda_moveit_ctrl.panda_robot_service import PandaRobotService

if __name__ == "__main__":
    rospy.init_node("panda_robot_service")

    vel = 0.4
    acc = 0.4
    load_gripper = True
    PRS = PandaRobotService(vel, acc, load_gripper=load_gripper)

    rospy.spin()