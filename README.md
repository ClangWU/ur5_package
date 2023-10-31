# UR5_Package
UR5 + Robotiq85 gripper Moveit! &amp; Gazebo packge

**1 -** Create a ros workspace 

`cd workspace/src`

`git clone https://github.com/ClangWU/ur5_package.git`

`cd .. && catkin_make`

**2 -** Run this command to launch moveit & gazebo. 

`source devel/setup.bash`

`roslaunch ur5_gripper_moveit_config demo_gazebo.launch`

**3 -** You can change the initial pose for the UR5 by revising the demo_gazebo.launch.

**Please** just ignore the error: `No p gain specified for pid.  Namespace:/***`
