# Teleoperation UR robot

Author: Robert van de Ven  
Contact: [robert.vandeven@wur.nl](mailto:robert.vandeven@wur.nl)

Code to collect demonstrations using Teleoperation with a UR3, controlled through OptiTrack and RelaxedIK. 
The codes has the following dependencies which need to be installed: 
* [Relaxed_IK](https://github.com/uwgraphics/relaxed_ik)
* [Universal_Robots_ROS_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
* [Robotiq](https://github.com/ros-industrial/robotiq)\
Specificially, the following packages are needed:\
``robotiq_2f_140_gripper_visualization``\
``robotiq_2f_gripper_control``\
``robotiq_ethercat``\
``robotiq_modbus_rtu``

## Using teleoperation (TO)
Make sure the pose-tracking device is in the zero pose and the robot arm is in the initial condition.  
The script can be launched by running the file `to.launch` which is the `launch` folder.