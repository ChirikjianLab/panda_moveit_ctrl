# Panda MoveIt Control

Author: Hongtao Wu

ROS package to control Panda robot with MoveIt via ROS service.
The Panda robot requires a realtime kernel to work with.
We set up a realtime kernel on a Ubuntu 16.04 desktop and advertise the ROS service to interact with the robot via [ROS MultipleMachine](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

## Installation
- [libfranka](https://frankaemika.github.io/docs/installation_linux.html)
- [franka_ros](https://frankaemika.github.io/docs/installation_linux.html)

## ROS service
- ```move_to_joint```: control the robot to a joint configuration
- ```move_to_cartesian```: control the robot to a cartesian pose
- ```move_gripper```: control the gripper

## Usage
* Initialize MoveIt and Rviz
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>
```
* Run the panda control server node
```
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py
```
* Control the robot by calling the service. 
A sample code to control is in ```scripts/panda_moveit_ctrl_node.py```.