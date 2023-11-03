#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_moveit_script', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name_arm = "ur5_arm"  # 修改为你的机械臂组名
    group_name_gripper = "gripper"  # 修改为你的抓手组名
    move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
    move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    rospy.loginfo("Planning frame: %s" % move_group_arm.get_planning_frame())
    rospy.loginfo("End effector link: %s" % move_group_arm.get_end_effector_link())
    rospy.loginfo("Robot Groups: %s" % robot.get_group_names())
    rospy.loginfo("Robot State:")
    rospy.loginfo(robot.get_current_state())

    # Planning to a Pose goal
    pose_goal = Pose()
    pose_goal.position.x = 0.3
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    pose_goal.orientation.w = 1.0

    move_group_arm.set_pose_target(pose_goal)

    rospy.loginfo("Planning to move to pose goal")
    plan = move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()

    if plan:
        rospy.loginfo("Motion to pose goal succeeded!")
    else:
        rospy.loginfo("Motion to pose goal failed.")

    # Control the gripper
    rospy.loginfo("Controlling the gripper...")
    gripper_joint_goal = [0.04]  # The gripper joint values, adjust as needed
    move_group_gripper.go(gripper_joint_goal, wait=True)
    move_group_gripper.stop()

    rospy.loginfo("Gripper control completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
