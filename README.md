# UR5_Package
UR5 + Robotiq85 gripper moveit &amp; gazebo packge

**1 -** Create a ros workspace 

`cd workspace/src`

`git clone https://github.com/ClangWU/ur5_package.git`

`cd .. & catkin_make`

**2 -** Run this command to launch moveit & gazebo. 

(You should click the start button of the gazebo, or you can revise the config in demo_gazebo.launch)

`source devel/setup.bash`

`roslaunch ur5_gripper_moveit_config demo_gazebo.launch`

