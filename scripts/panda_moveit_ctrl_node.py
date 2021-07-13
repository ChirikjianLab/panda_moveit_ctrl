#! /usr/bin/python

# ROS node to publish the service
# Author: Hongtao Wu
# Date: Mar 05, 2021

import rospy
import time
from panda_moveit_ctrl.panda_robot import PandaRobot

if __name__ == "__main__":
    rospy.init_node("panda_move")

    load_gripper = True
    
    robot = PandaRobot(load_gripper=load_gripper)
    # robot.goMonitor()
    # robot.goHome()
    time.sleep(3.0)
    robot.open_gripper()
    time.sleep(3.0)
    robot.close_gripper()
    