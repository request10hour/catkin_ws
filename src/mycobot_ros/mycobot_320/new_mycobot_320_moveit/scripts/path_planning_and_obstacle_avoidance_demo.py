#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from rosgraph.names import ns_join
from math import sin, cos, radians
import copy

x, y, d, p = 0.0, 0.0, 0.0, 0.0
rospub = rospy.Publisher(
            ns_join("", "collision_object"), CollisionObject, queue_size=100
        )

def xydp_callback(data):
    global x, y, d, p
    x_str, y_str, d_str, p_str = data.data.split('y')[0], data.data.split('y')[1].split('d')[0], data.data.split('d')[1].split('p')[0], data.data.split('p')[1]
    x, y, d, p = float(x_str[1:]), float(y_str), float(d_str), float(p_str)
    p = -radians(p)

def make_object(posestamped, shape_type, shape_dimensions, id):
    co = CollisionObject()
    co.operation = co.ADD
    co.id = id
    co.header = posestamped.header
    co.pose = posestamped.pose
    shape = SolidPrimitive()
    shape.type = shape_type
    shape.dimensions = shape_dimensions
    co.primitives = [shape]
    rospub.publish(co)
    return co

def move_object(co, posestamped):
    co.operation = co.MOVE
    co.pose = posestamped.pose
    rospub.publish(co)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    arm = MoveGroupCommander("arm_group")
    planningscene = PlanningSceneInterface()
    robotcommander = RobotCommander()

    planningscene.remove_attached_object()
    planningscene.remove_world_object()

    box_co = None
    a4_co = None
    sphere_co = None

    while 1 :
        rospy.init_node('arm_controller_node', anonymous=True)
        rospy.Subscriber('xyd_topic', String, xydp_callback)
        rospy.sleep(0.5)

        posestamped = moveit_commander.PoseStamped()
        posestamped.header.frame_id = robotcommander.get_planning_frame()

        default_xyz = [0.0, -0.1, 0.4]

        posestamped.pose.position.x = default_xyz[0] + x
        posestamped.pose.position.y = default_xyz[1] - d
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

        posestamped_add_distance = copy.deepcopy(posestamped)
        posestamped_add_distance.pose.position.y -= 0.01

        if (box_co is None or a4_co is None or sphere_co is None):
            box_co = make_object(posestamped_add_distance, SolidPrimitive.BOX, [0.297 * 2, 0.21 * 2, 0.001], "box")
            a4_co = make_object(posestamped, SolidPrimitive.BOX, [0.297, 0.21, 0.001], "a4")
            sphere_co = make_object(posestamped, SolidPrimitive.SPHERE, [0.025], "sphere")
        else:
            move_object(box_co, posestamped_add_distance)
            move_object(a4_co, posestamped)
            move_object(sphere_co, posestamped)

        rospy.sleep(0.5)