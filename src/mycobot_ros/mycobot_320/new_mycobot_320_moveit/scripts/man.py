#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import cv2
import time
import threading
import rospy
from std_msgs.msg import String
import math

class StringPublisher:
    def __init__(self):
        # 노드를 초기화합니다.
        rospy.init_node('string_publisher_node', anonymous=True)
        # 퍼블리셔를 생성합니다.
        self.xyd_pub = rospy.Publisher('xyd_topic', String, queue_size=10)
        self.xyd_data = "x0.001y0.002d0.003p0.004"
        self.frontrear_data = "f0.001r0.002"
        # 퍼블리시 빈도를 설정합니다 (예: 1초에 한 번 퍼블리시).
        self.rate = rospy.Rate(1) # 1 Hz
        # 종료 이벤트 생성
        self.stop_event = threading.Event()
        # 스레드 시작
        self.publish_thread = threading.Thread(target=self.publish_strings, daemon=True)
        self.publish_thread.start()

    def calculate_t(self, p, x, d):
        # Convert p to absolute value
        p = abs(p)
        # Calculate theta
        theta = 90 - p
        # Convert theta to radians
        theta_rad = math.radians(theta)
        # Convert x to absolute value
        # Calculate alpha
        sin_alpha = (x * math.sin(theta_rad)) / d
        alpha_rad = math.asin(sin_alpha)
        # Calculate T
        t = d * math.cos(alpha_rad + theta_rad)

        front = t * math.cos(theta_rad)
        rear = t * math.sin(theta_rad)
        self.frontrear_data = f"f{front}r{rear}"

        print("옆", t * math.sin(math.radians(theta)))
        print("앞", t * math.cos(math.radians(theta)))

        return t

    def publish_strings(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            # 토픽에 메시지를 발행합니다.
            self.xyd_pub.publish(String(self.xyd_data + self.frontrear_data))
            # 주기를 유지합니다.
            self.rate.sleep()

    def calc_average_recv(self, value, newvalue):
        if value == 0 or float('nan'):
            return newvalue
        else:
            return math.sqrt(value * newvalue)

    def open_socket_for_receiving(self):
        while True:
            try:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.connect(('192.168.137.1', 8001))
                x, y, d, p = 0, 0, 0, 0
                while True:
                    data = server_socket.recv(1024)
                    if not data:
                        break
                    print(data.decode())
                    try:
                        x_str, y_str, d_str, p_str = self.xyd_data.split('y')[0], self.xyd_data.split('y')[1].split('d')[0], self.xyd_data.split('d')[1].split('p')[0], self.xyd_data.split('p')[1]
                        newx, newy, newd, newp = float(x_str[1:]), float(y_str), float(d_str), float(p_str)
                        x = self.calc_average_recv(x, newx)
                        y = self.calc_average_recv(y, newy)
                        d = self.calc_average_recv(d, newd)
                        p = self.calc_average_recv(p, newp)
                        print(p,x,d)
                        self.calculate_t(p, x, d)
                    except:
                        pass
                    self.xyd_data = data.decode()

                server_socket.close()
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(5)  # 5초 대기 후 다시 시도

    def send_video(self, fps=10, compression_quality=70):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)  # Increase socket buffer size
        client_socket.connect(('192.168.137.1', 8000))
        connection = client_socket.makefile('wb')

        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)

        frame_interval = 1.0 / fps

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), compression_quality]

        try:
            while True:
                start_time = time.time()

                ret, frame = cap.read()
                if not ret:
                    break

                # Convert frame to grayscale
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                result, gray_frame = cv2.imencode('.jpg', gray_frame, encode_param)
                data = gray_frame.tobytes()
                size = len(data)

                connection.write(struct.pack('<L', size) + data)

                elapsed_time = time.time() - start_time
                time_to_wait = frame_interval - elapsed_time
                if time_to_wait > 0:
                    time.sleep(time_to_wait)
        finally:
            cap.release()
            connection.close()
            client_socket.close()


if __name__ == "__main__":
    try:
        video_to_publish = StringPublisher()
        # Start the thread to receive processed data
        threading.Thread(target=video_to_publish.open_socket_for_receiving, daemon=True).start()
        # Call the function with the desired FPS and compression quality
        video_to_publish.send_video(fps=10, compression_quality=50)
    except rospy.ROSInterruptException:
        pass
