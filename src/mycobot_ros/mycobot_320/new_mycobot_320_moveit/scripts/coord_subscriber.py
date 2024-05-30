#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

class ArmController:
    def __init__(self):
        # MoveIt을 초기화합니다.
        moveit_commander.roscpp_initialize([])
        self.arm = MoveGroupCommander("arm_group")
        self.planningscene = PlanningSceneInterface()
        self.robotcommander = RobotCommander()
        self.planningscene.remove_world_object("box")
        self.arm.detach_object("sphere")
        self.planningscene.remove_world_object("sphere")

        # 관절 및 위치/자세 허용 오차를 설정합니다.
        self.arm.set_goal_position_tolerance(0.01) # 2cm
        self.arm.set_goal_orientation_tolerance(0.05) # 3 degrees

        self.default_xyz = [0.0, -0.1, 0.4]

        # ROS 노드를 초기화하고 토픽을 구독합니다.
        rospy.init_node('arm_controller_node', anonymous=True)
        rospy.Subscriber('xyd_topic', String, self.xyd_callback)
        self.callbacking = True

        self.arm.set_pose_target(self.default_xyz + [-1.57, 0.0, -3.14], end_effector_link="link6")
        self.arm.plan()
        self.arm.go(wait=True)
        # 목표를 초기화합니다.
        self.arm.stop()
        self.arm.clear_pose_targets()
        input("Arm is ready. Press enter to start moving.")
        self.callbacking = False

    def xyd_callback(self, msg):
        if not self.callbacking:
            self.callbacking = True
            self.update_arm_position(msg.data)

    def update_arm_position(self, xyd_str):
        try:
            # xyd_str을 파싱하여 목표 위치를 설정합니다.
            x_str, y_str, d_str, p_str = xyd_str.split('y')[0], xyd_str.split('y')[1].split('d')[0], xyd_str.split('d')[1].split('p')[0], xyd_str.split('p')[1]
            x, y, d, p = float(x_str[1:]), float(y_str), float(d_str), float(p_str)
            print(f"Received new position: x={x}, y={y}, d={d}, p={p}")
            self.default_xyz[0] += x
            self.default_xyz[1] -= (d - 0.15)
            self.default_xyz[2] += y

            if input(f"press enter to move, {self.default_xyz}") == "q":
                rospy.signal_shutdown("User requested shutdown")
            # print(arm.get_current_rpy("link6"))
            p_rad = p * 3.1415 / 180
            minus = -1 if p_rad < 0 else 1
            move_rad = (3.1415 - abs(p_rad)) * minus

            posestamped = moveit_commander.PoseStamped()
            posestamped.header.frame_id = self.robotcommander.get_planning_frame()
            posestamped.pose.position.x = self.default_xyz[0]
            posestamped.pose.position.y = self.default_xyz[1]
            posestamped.pose.position.z = self.default_xyz[2]
            self.planningscene.add_box("box", posestamped, [1.0, 0.1, 1.0])
            self.arm.set_pose_target(self.default_xyz + [-1.57, 0.0, move_rad], end_effector_link="link6")
            self.arm.plan()
            self.arm.go(wait=True)

            # 목표를 초기화합니다.
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr(f"Failed to update arm position: {e}")
        finally:
            input("I checked the position. Press enter to start moving.")
            self.callbacking = False

if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        rospy.spin()  # ROS 노드가 종료될 때까지 대기
    except rospy.ROSInterruptException:
        pass
