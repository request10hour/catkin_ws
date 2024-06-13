#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time
import math
import subprocess
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from pymycobot.mycobot import MyCobot


mc = None
ee_option = 1

def ee_callback(data):
    global ee_option
    ee_option = data.data

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)

    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    mc.send_angles(data_list, 40)

    mc.set_basic_output(6, int(ee_option))


def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)
    rospy.Subscriber('ee_topic', Int32, ee_callback)

    # scan the port
    res = None
    while True:
        try:
            res = subprocess.check_output("ls /dev/ttyACM*", shell=True)
            if res:
                break
        except Exception as e:
            rospy.logerr("No port found, retrying...")
            time.sleep(1)

    port = rospy.get_param("~port", res.decode().strip())
    baud = rospy.get_param("~baud", 115200)

    mc = MyCobot(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    listener()
