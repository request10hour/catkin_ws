#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import moveit_commander

class ArmController:
    def __init__(self):
        # MoveIt을 초기화합니다.
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        # 관절 및 위치/자세 허용 오차를 설정합니다.
        self.arm.set_goal_position_tolerance(0.015) # 2cm
        self.arm.set_goal_orientation_tolerance(0.1) # 3 degrees

        # 이전 메시지 데이터를 저장합니다.
        self.previous_xyd = ""

        # ROS 노드를 초기화하고 토픽을 구독합니다.
        rospy.init_node('arm_controller_node', anonymous=True)
        rospy.Subscriber('xyd_topic', String, self.xyd_callback)

    def xyd_callback(self, msg):
        if msg.data != self.previous_xyd:
            self.previous_xyd = msg.data
            self.update_arm_position(msg.data)

    def update_arm_position(self, xyd_str):
        try:
            # xyd_str을 파싱하여 목표 위치를 설정합니다.
            x_str, y_str, d_str = xyd_str.split('y')[0], xyd_str.split('y')[1].split('d')[0], xyd_str.split('d')[1]
            x, y, d = float(x_str[1:]), float(y_str), float(d_str)
            print(f"Received new position: x={x}, y={y}, d={d}")

            # 목표 위치를 설정합니다.
            pose_target = [0.2 + d, 0.0 - x, 0.4 - y, 0, 1.57, 0]
            self.arm.set_pose_target(pose_target, end_effector_link="link6")

            # 계획을 수립하고 실행합니다.
            plan = self.arm.plan()
            self.arm.go(wait=True)

            # 목표를 초기화합니다.
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr(f"Failed to update arm position: {e}")

if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        rospy.spin()  # ROS 노드가 종료될 때까지 대기
    except rospy.ROSInterruptException:
        pass
