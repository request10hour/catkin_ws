#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pymycobot import MyCobot
import rospy
from std_msgs.msg import String
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from rosgraph.names import ns_join
from math import sin, cos, radians
import copy

class ArmController:
    def __init__(self):
        # MoveIt을 초기화합니다.
        moveit_commander.roscpp_initialize([])
        self.arm = MoveGroupCommander("arm_group")
        self.planningscene = PlanningSceneInterface()
        self.robotcommander = RobotCommander()
        self.rospub = rospy.Publisher(
            ns_join("", "collision_object"), CollisionObject, queue_size=100
        )

        self.planningscene.remove_attached_object()
        self.planningscene.remove_world_object()

        self.box_co = None
        self.a4_co = None
        self.sphere_co = None

        # 관절 및 위치/자세 허용 오차를 설정합니다.
        self.arm.set_goal_position_tolerance(0.01) # 2cm
        self.arm.set_goal_orientation_tolerance(0.05) # 3 degrees
        self.distance_from_socket = 0.11

        self.default_xyz = [0.0, -0.1, 0.35]

        # ROS 노드를 초기화하고 토픽을 구독합니다.
        self.callbacking = True
        rospy.init_node('arm_controller_node', anonymous=True)
        rospy.Subscriber('xyd_topic', String, self.xyd_callback)

        self.arm.set_pose_target(self.default_xyz + [-1.57, 0.0, -3.14], end_effector_link="link6")
        self.arm.plan()
        self.arm.go(wait=True)
        # 목표를 초기화합니다.
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.sleep(10)
        self.callbacking = False

    def xyd_callback(self, msg):
        print("Callback")
        if not self.callbacking:
            self.callbacking = True
            self.update_arm_position(msg.data)

    def update_arm_position(self, xyd_str):
        try:
            # xyd_str을 파싱하여 목표 위치를 설정합니다.
            x_str, y_str, d_str, p_str = xyd_str.split('y')[0], xyd_str.split('y')[1].split('d')[0], xyd_str.split('d')[1].split('p')[0], xyd_str.split('p')[1]
            x, y, d, p = float(x_str[1:]), float(y_str), float(d_str), float(p_str)
            print(f"Received new position: x={x}, y={y}, d={d}, p={p}")

            pose_all = self.object_control(self.robotcommander, self.box_co, self.a4_co, self.sphere_co, x, d, y, -radians(p))

            # print(arm.get_current_rpy("link6"))
            p_rad = p * 3.1415 / 180
            minus = -1 if p_rad < 0 else 1
            move_rad = (3.1415 - abs(p_rad)) * minus

            print(p)

            self.default_xyz[0] = float(pose_all.pose.position.x) + self.distance_from_socket * sin(radians(p))
            self.default_xyz[1] = ((-0.1) / cos(radians(p))) + (float(pose_all.pose.position.y) + self.distance_from_socket) * cos(radians(p))
            self.default_xyz[2] = float(pose_all.pose.position.z)
            print(self.default_xyz)

            self.arm.set_pose_target(self.default_xyz + [-1.57, 0.0, move_rad], end_effector_link="link6")
            self.arm.plan()
            self.arm.go(wait=True)

            # 목표를 초기화합니다.
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr(f"Failed to update arm position: {e}")
        finally:
            rospy.sleep(5)
            rospy.signal_shutdown("User requested shutdown")

    def make_object(self, posestamped : moveit_commander.PoseStamped, shape_type, shape_dimensions, id):
        co = CollisionObject()
        co.operation = co.ADD
        co.id = id
        co.header = posestamped.header
        co.pose = posestamped.pose
        shape = SolidPrimitive()
        shape.type = shape_type
        shape.dimensions = shape_dimensions
        co.primitives = [shape]
        self.rospub.publish(co)
        return co

    def move_object(self, co : CollisionObject, posestamped : moveit_commander.PoseStamped):
        co.operation = co.MOVE
        co.pose = posestamped.pose
        self.rospub.publish(co)

    def object_control(self, robotcommander : RobotCommander, box_co, a4_co, sphere_co, x, d, y, p):

        posestamped = moveit_commander.PoseStamped()
        posestamped.header.frame_id = robotcommander.get_planning_frame()

        default_xyz = [0.0, -0.1, 0.35]

        posestamped.pose.position.x = default_xyz[0] + x - (x * sin(p))
        posestamped.pose.position.y = default_xyz[1] - d * cos(-p)
        posestamped.pose.position.z = default_xyz[2] + y

        # convert euler angles to quaternion
        r, y = -1.57, -3.14

        cy = cos(y * 0.5)
        sy = sin(y * 0.5)
        cr = cos(r * 0.5)
        sr = sin(r * 0.5)
        cp = cos(p * 0.5)
        sp = sin(p * 0.5)

        posestamped.pose.orientation.x = cy * sr * cp - sy * cr * sp   # x
        posestamped.pose.orientation.y = sy * cr * cp + cy * sr * sp   # y
        posestamped.pose.orientation.z = sy * cr * cp - cy * sr * sp   # z
        posestamped.pose.orientation.w = cy * cr * cp + sy * sr * sp   # w
        print(posestamped.pose.position)
        print(posestamped.pose.orientation)

        posestamped_add_distance = copy.deepcopy(posestamped)
        posestamped_add_distance.pose.position.y -= 0.01

        if (box_co is None or a4_co is None or sphere_co is None):
            box_co = self.make_object(posestamped_add_distance, SolidPrimitive.BOX, [0.297 * 2, 0.21 * 2, 0.001], "box")
            a4_co = self.make_object(posestamped, SolidPrimitive.BOX, [0.297, 0.21, 0.001], "a4")
            sphere_co = self.make_object(posestamped, SolidPrimitive.SPHERE, [0.025], "sphere")
        else:
            self.move_object(box_co, posestamped_add_distance)
            self.move_object(a4_co, posestamped)
            self.move_object(sphere_co, posestamped)

        return posestamped

if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        rospy.spin()  # ROS 노드가 종료될 때까지 대기
    except rospy.ROSInterruptException:
        pass
