# Panda MoveIt Control
Control Panda robot with MoveIt

Author: Hongtao Wu

Date: Mar 05, 2021

# Installation
------

# Usage
------
* Initialize MoveIt and Rviz
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>
```
* Run the panda control server node
```
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py
```
* Control the robot by calling the service **move_to_joint**, **move_to_cartesian**, and **move_gripper**. The sample code to control is in *scripts/panda_moveit_ctrl_node.py* and *src/panda_moveit_ctrl/panda_robot.py*.

# TODO
------
- [ ] Add installation guide in README