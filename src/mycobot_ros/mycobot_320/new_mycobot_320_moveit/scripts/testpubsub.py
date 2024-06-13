#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

def pub_robotarm_callback(data):
	rospy.loginfo('pub_robotarm: %s', data.data)

def client_message_callback(data):
    rospy.loginfo('client_message: %s', data.data)

if __name__ == "__main__":
	rospy.init_node('testpubsub', anonymous=True)
	pub0 = rospy.Publisher('control_robotarm', Int32, queue_size=10)
	sub0 = rospy.Subscriber('pub_robotarm', Int32, pub_robotarm_callback)
	sub1 = rospy.Subscriber('client_message', Int32, client_message_callback)

	pub0.publish(-400)

	rospy.spin()
