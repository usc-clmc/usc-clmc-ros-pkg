This stack contains an implementation of the Dynamic Movement Primitive (DMP) framework. The code is robot-independent. To make it work with your robot you will have to look into the robot_info package and load corresponding parameters that defines your robot.

To run a simple demo on a simulated/real PR2 you need to have ros diamondback installed. Also, as of now, the rosdep are only specified for Ubuntu.

##############################################
Disclaimer: If you run this code on your robot causing it to break, you can keep all the parts. That is, I am not liable to anything that happens when you run this code on your robot.

##############################################

To download and make the packages you need to:

1) download the code and add it to your ROS_PACKAGE_PATH. Make sure that it overlays the ros installation, i.e. make sure that the directory containing the usc-clmc-ros-pkg folder is *before* the ros install folder.

2) $ rosdep install pr2_launch

3) $ rosmake pr2_launch

##############################################

It is recommended to change the default publishing rate of the joint states to 1000Hz. To do so, you need to overlay the pr2_mechanism stack. Thus, copy the pr2_mechanism stack from the ros intall into a local directory and add the path to this directory *before* the ros install path in the $ROS_PACKAGE_PATH. Then change the rate of the joint_state_publisher in pr2_mechanism/pr2_controller_manager/controller_manager.launch from 100.0 to 1000.0.

##############################################

To run the demo in simulation you need to:

1) $ roslaunch pr2_dynamic_movement_primitive_controller pr2_empty_world.launch
(this brings up the pr2 in gazebo and load the dmp ik controllers)

2) $ roslaunch pr2_dynamic_movement_primitive_gui pr2_dynamic_movement_primitive_gui.launch 
(this pops up 2 guis which allow you to record, generate, and execute dmps for both arms.

As of now, only the IK controller are tested.

##############################################

To run the demo on the PR2 you will need to bring up the robot and first run 

1) $ roslaunch pr2_dynamic_movement_primitive_gui pr2_dynamic_movement_primitive_gui.launch 

on the base machine. The gui allows you to start/stop/reload controllers. Before starting the dmp_ik_controllers you need to stop the default controllers r_arm_controller and l_arm_controllers. Just select them and click on stop.

Then, you can start the dmp_ik_controllers on the robot using

2) $ roslaunch pr2_dynamic_movement_primitive_controller dmp_ik_controller_loader.launch

If everything worked out you should be able to record, generate, and execute dmps. As of now, the goal of the dmp cannot be changed using the gui. However, you can publish to /r_arm_dmp_ik_controller/goal which changes the goal (of the right arm) online. Check out pr2_dynamic_movement_primitive_controller/test/cmd_line_goal_change.sh.

You may want to check out the visualization markers that are published by the dmp_ik_controllers and add them to rviz.

The recorded data and the generated dmps are stored in pr2-data/pr2_dmp_data/demonstrations and pr2-data/pr2_dmp_data/dmp_data/.

There are a bunch of things that may be fixed or added. Feel free to check out the dmp/TODO.txt
