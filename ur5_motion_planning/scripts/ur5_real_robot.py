#!/usr/bin/env python3

import sys
import rospy
import argparse
import urx
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

def parse_arguments():
    parser = argparse.ArgumentParser(description='UR5 Robot Controller')
    parser.add_argument('--robot_ip', type=str, help='IP address of the UR robot', required=True)
    args = parser.parse_args()
    return args


class UR5Controller:
    def __init__(self, robot_ip):
        # IP 地址
        
        self.robot = None
        self.robot_ip = "192.168.1.1"
        try:
            self.robot = urx.Robot(self.robot_ip)
        except Exception as e:
            rospy.logerr("Failed to connect to UR5 at IP %s: %s", self.robot_ip, str(e))
            sys.exit(1)
        # Set the robot home position，这里的home_position是关节角度， 必须符合demo_gazebo.launch中的home_position
        home_position = [-1.544, -1.544, -1.54, -1.579, 1.579, 0]
        # Move to home position
        self.robot.movej(home_position, acc=0.1, vel=0.1, wait=True)

        self.gripper = Robotiq_Two_Finger_Gripper(self.robot)
        # sub joint_states
        self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_recv_callback)
        self.gripper_states_sub = rospy.Subscriber('gripper_states', String, self.gripper_states_recv_callback)

        rospy.loginfo("UR5 Controller initialized")

    def joint_states_recv_callback(self, msg):
        rospy.loginfo("Current Joint States: %s", msg.position)
        if len(msg.position) >= 6:
            try:
                self.robot.movej(msg.position[:6], acc=0.1, vel=0.1, wait=False)
            except Exception as e:
                rospy.logerr("Failed to move UR5: %s", str(e))

    def gripper_states_recv_callback(self, msg):
        if msg.data == "open":
          rospy.loginfo("Opening gripper")
          self.gripper.open_gripper()
        elif msg.data == "close":
          rospy.loginfo("Closing gripper")
          self.gripper.close_gripper()

    def shutdown(self):
        self.robot.close()
        rospy.loginfo("UR5 Controller shutdown")

def main():
    args = parse_arguments()
    rospy.init_node('ur5_real_robot', anonymous=True)
    ur5_controller = UR5Controller(robot_ip=args.robot_ip)
    rospy.on_shutdown(ur5_controller.shutdown)  
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, Exception) as e:
        rospy.logerr("Unhandled exception: %s", str(e))