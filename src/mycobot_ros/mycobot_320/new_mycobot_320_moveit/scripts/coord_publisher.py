#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import threading

class StringPublisher:
    def __init__(self):
        # 노드를 초기화합니다.
        rospy.init_node('string_publisher_node', anonymous=True)

        # 퍼블리셔를 생성합니다.
        self.xyd_pub = rospy.Publisher('xyd_topic', String, queue_size=10)

        # 초기 메시지 설정
        self.xyd_msg_data = "x0.001y0.002d0.003"

        # 퍼블리시 빈도를 설정합니다 (예: 1초에 한 번 퍼블리시).
        self.rate = rospy.Rate(1) # 1 Hz

        # 종료 이벤트 생성
        self.stop_event = threading.Event()

        # 스레드 시작
        self.publish_thread = threading.Thread(target=self.publish_strings)
        self.publish_thread.start()

    def publish_strings(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            # 발행할 메시지를 생성합니다.
            xyd_msg = String()
            xyd_msg.data = self.xyd_msg_data

            # 토픽에 메시지를 발행합니다.
            self.xyd_pub.publish(xyd_msg)

            # 주기를 유지합니다.
            self.rate.sleep()

    def update_message(self, new_msg):
        self.xyd_msg_data = new_msg

    def stop(self):
        self.stop_event.set()
        self.publish_thread.join()

if __name__ == '__main__':
    try:
        publisher = StringPublisher()

        while not rospy.is_shutdown():
            # 사용자 입력을 받아 메시지를 업데이트합니다.
            user_input = input("Enter new message (format: x0.12y0.34d0.56) or type 'exit' to stop: ")
            if user_input.lower() == "exit":
                publisher.stop()
                break
            else:
                publisher.update_message(user_input)

    except rospy.ROSInterruptException:
        pass
