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

## Troubleshooting

If your ros environment is installed by Robostack within a conda environment. You may encounter the error: 

```bash
[Err] [Plugin.hh:212] Failed to load plugin libgazebo_xxxxxxx.so: ...
```

This is probably because the ros-in-conda environment will not set `LD_LIBRARY_PATH` correctly. You can fix this by adding the path `/path/to/catkin_ws/devel/lib` to the `LD_LIBRARY_PATH` variable, e.g. adding the following line to `demo_gazebo.launch` roslaunch file:

```bash
   <env name="LD_LIBRARY_PATH" value=/path/to/catkin_ws/devel/lib:$(optenv LD_LIBRARY_PATH)"/>
```
