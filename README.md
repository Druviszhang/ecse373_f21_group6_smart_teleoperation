# ecse373_f21_group6_smart_teleoperation Instructions
###### ECSE 373 Lab 3 for group 6

In order to start up with single robot type this command:

```
roslaunch smart_teleoperation launch_file.launch robot_name:=robot0
```

This command is defaulted to robot0 for the first robot name. Add a new robot to the field by creating a new robot, adding lasers to it, and then placing it on the field. Change the topic of rqt_gui into `/<robot_name>/des_vel`, then give the robot an initial velocity_x.

In order to run multiple robots run the following commands:
```
roslaunch smart_teleoperation smart_teleoperation_node.launch robot_name:=robot1
rosrun rqt_gui rqt_gui
```
robot_name is the parameter for the new robot name. A new rqt_gui has to be created and set to change the velocities for the new robot. Remember to change the defaulted robot name to the name of the robot you just created.
