#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped



# Workaround to use the gripper action client with unified gripper interface
class GripperCommanderGroup:

    def __init__(self) -> None:    

        self.init_clients()
        self.gripper_status_pub = rospy.Publisher('gripper_status', String, queue_size=10)
        print("Gripper action clients ready")

    def init_clients(self):
        self.action_gripper = actionlib.SimpleActionClient(
        '/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction
        )
        print("Waiting for action of gripper controller")
        _ans = self.action_gripper.wait_for_server(rospy.Duration(5))
        if _ans:
            rospy.loginfo("Action server started")
        elif not _ans:
            rospy.loginfo("Action server not started") 

    def open_gripper(self, value=0.08):
        self.set_gripper(value)
        self.gripper_status_pub.publish("open") # Publish the status of the gripper

    def close_gripper(self, value=0.03):
        self.set_gripper(value)
        self.gripper_status_pub.publish("close") # Publish the status of the gripper

    def set_gripper(self, value):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # Create a trajectory point
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [(0.08 - value)*10]  # Set the gripper position
        trajectory_point.time_from_start = rospy.Duration(2)  # Move to the position in 3 seconds

        trajectory = JointTrajectory()
        trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]  # This should be the name of your gripper joint
        trajectory.points.append(trajectory_point)
        goal.trajectory = trajectory

        self.action_gripper.send_goal(goal)
        self.action_gripper.wait_for_result(rospy.Duration(10))
        return self.action_gripper.get_result()

def all_close(goal, actual, tolerance=0.01):
    """
    Convenience method for testing if the values in two lists are within a tolerance range of each other.
    :param: goal: A list of floats, the length of the list must match that of actual.
    :param: actual: A list of floats, the length of the list must match that of goal.
    :param: tolerance: A float, tolerance for equality testing.
    :returns: bool
    """
    all_equal = True
    for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
            return False

    return True

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
    GripperCommander = GripperCommanderGroup()

    rospy.loginfo("Planning frame: %s" % move_group_manipulator.get_planning_frame())
    rospy.loginfo("End effector link: %s" % move_group_manipulator.get_end_effector_link())
    rospy.loginfo("Robot Groups: %s" % robot.get_group_names())
    rospy.loginfo("Robot State:")
    rospy.loginfo(robot.get_current_state())

    rospy.loginfo("Closing the gripper")
    GripperCommander.close_gripper()

    current_pose_stamped = move_group_manipulator.get_current_pose()
    current_pose = current_pose_stamped.pose
    move_group_manipulator.set_start_state_to_current_state()
    move_group_manipulator.clear_pose_targets()
    # Log the current pose
    rospy.loginfo("Current Pose: %s" % current_pose)

    # Define a new pose goal
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = move_group_manipulator.get_planning_frame()
    pose_goal.pose.position.x = current_pose.position.x 
    pose_goal.pose.position.y = current_pose.position.y
    pose_goal.pose.position.z = current_pose.position.z + 0.10 # Move up by 5 cm
    pose_goal.pose.orientation = current_pose.orientation

    # Now, you can set this new pose as the goal
    move_group_manipulator.set_pose_target(pose_goal)

    # Proceed with planning and moving as before
# Plan and execute the trajectory
    success = move_group_manipulator.go(wait=True)
    move_group_manipulator.stop()  # Ensure that there is no residual movement
    move_group_manipulator.clear_pose_targets()

    # Moving the gripper
    rospy.sleep(2)
    GripperCommander.open_gripper()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
