#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import moveit_commander

if __name__ == "__main__":
    arm = moveit_commander.MoveGroupCommander("arm_group")

    # arm.set_goal_joint_tolerance(0.5)
    arm.set_goal_position_tolerance(0.03)
    arm.set_goal_orientation_tolerance(0.001)

    arm.set_pose_target([0.0, 0.0, 0.3, 0, 1.57, 0], end_effector_link="link6")
    arm.plan()
    arm.go()
