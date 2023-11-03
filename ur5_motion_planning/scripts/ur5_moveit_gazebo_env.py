#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_gripper(action_gripper, value):
    # Initialize the goal as a FollowJointTrajectoryGoal
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    # Create a trajectory point
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = [value]  # Set the gripper position
    trajectory_point.time_from_start = rospy.Duration(3)  # Move to the position in 3 seconds

    # Create a trajectory and add the trajectory point to it
    trajectory = JointTrajectory()
    trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]  # This should be the name of your gripper joint
    trajectory.points.append(trajectory_point)

    # Set the trajectory in the goal
    goal.trajectory = trajectory

    # Send the goal
    action_gripper.send_goal(goal)
    # Wait for the server to finish performing the action
    action_gripper.wait_for_result(rospy.Duration(10))

    # Return the result of executing the action
    return action_gripper.get_result()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_moveit_script', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name_arm = "ur5_arm"  # 修改为你的机械臂组名
    group_name_gripper = "gripper"  # 修改为你的抓手组名
    group_name_manipulator = "ur5_manipulator"  # 修改为你的tcp组名

    move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
    move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
    move_group_manipulator = moveit_commander.MoveGroupCommander(group_name_manipulator)

    rospy.loginfo("Planning frame: %s" % move_group_manipulator.get_planning_frame())
    rospy.loginfo("End effector link: %s" % move_group_manipulator.get_end_effector_link())
    rospy.loginfo("Robot Groups: %s" % robot.get_group_names())
    rospy.loginfo("Robot State:")
    rospy.loginfo(robot.get_current_state())

    action_gripper = actionlib.SimpleActionClient(
      '/gripper_controller/follow_joint_trajectory',
      control_msgs.msg.FollowJointTrajectoryAction
    )
    print("Waiting for action of gripper controller")
    _ans = action_gripper.wait_for_server(rospy.Duration(5))

    if _ans:
        rospy.loginfo("Action server started")
    elif not _ans:
        rospy.loginfo("Action server not started") 
    # Planning to a Pose goal
    pose_goal = Pose()
    pose_goal.position.x = 0.0
    pose_goal.position.y = 0.4
    pose_goal.position.z = 0.4
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    move_group_manipulator.set_pose_target(pose_goal)

    rospy.loginfo("Planning to move to pose goal")
    plan = move_group_manipulator.go(wait=True)
    move_group_manipulator.stop()
    move_group_manipulator.clear_pose_targets()

    if plan:
        rospy.loginfo("Motion to pose goal succeeded!")
    else:
        rospy.loginfo("Motion to pose goal failed.")


    # Closing the gripper
    rospy.loginfo("Closing the gripper")
    set_gripper(action_gripper, 0.8)
        # Opening the gripper
    rospy.loginfo("Opening the gripper")
    set_gripper(action_gripper, 0.0)
    # Closing the gripper
    rospy.loginfo("Closing the gripper")
    set_gripper(action_gripper, 0.8)
        # Opening the gripper
    rospy.loginfo("Opening the gripper")
    set_gripper(action_gripper, 0.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
