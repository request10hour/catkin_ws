#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import moveit_commander
import threading

default_xyz = [0.0, -0.2, 0.35]
x, y, d = 0.0, 0.0, 0.0

def sub_init():
    rospy.init_node('arm_controller_node', anonymous=True)
    rospy.Subscriber('xyd_topic', String, xyd_callback)
    rospy.spin()

def xyd_callback(msg):
    print(f"Received new position: {msg.data}")
    xyd_str = msg.data
    x_str, y_str, d_str = xyd_str.split('y')[0], xyd_str.split('y')[1].split('d')[0], xyd_str.split('d')[1]
    x, y, d = float(x_str[1:]), float(y_str), float(d_str)

if __name__ == "__main__":
    threading.Thread(target=sub_init).start()

    arm = moveit_commander.MoveGroupCommander("arm_group")
    # arm.set_goal_joint_tolerance(0.5)
    arm.set_goal_position_tolerance(0.03)
    arm.set_goal_orientation_tolerance(0.1)

    while 1:
        print(x, y, d)
        if input(f"press enter to move, {default_xyz}") == "q":
            break
        # print(arm.get_current_rpy("link6"))
        arm.set_pose_target(default_xyz + [-1.57, 0.0, -3.1415], end_effector_link="link6")
        arm.plan()
        arm.go()
        default_xyz[0] += x
        default_xyz[1] += y
        default_xyz[2] += d
