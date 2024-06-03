#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import torch
import time
import threading
import rospy
from std_msgs.msg import String

class MarkerDistanceCalculator:
    def __init__(self, cam_num="/dev/video0", distance_adjustment=1, screen_width=640, screen_height=480, p0=5, q0=5):
        self.cam_num = cam_num
        self.distance_adjustment = distance_adjustment
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.p0 = p0
        self.q0 = q0

        self.camera_matrix = np.array([[600, 0, screen_width/2], [0, 600, screen_height/2], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.1, -0.05, 0, 0, 0], dtype=np.float32)

        self.marker_length_m = 0.044
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters = cv2.aruco.DetectorParameters()

        half_marker_length = self.marker_length_m / 2
        self.marker_corners_3d = np.array([[-half_marker_length, half_marker_length, 0],
                                           [half_marker_length, half_marker_length, 0],
                                           [half_marker_length, -half_marker_length, 0],
                                           [-half_marker_length, -half_marker_length, 0]])

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/juna/foraddy/catkin_ws/src/mycobot_ros/mycobot_320/new_mycobot_320_moveit/scripts/best8.pt')

        self.cap = cv2.VideoCapture(self.cam_num)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.screen_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.screen_height)

        self.real_distance_x = 0.0
        self.real_distance_y = 0.0
        self.distance_average = 0.0
        self.pitch_average = 0.0

    def pixel_to_real_distance(self, pixel_distance, z, fx):
        real_distance = pixel_distance * z / fx
        return real_distance

    def rotation_matrix_to_euler_angles(self, R):
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.degrees(x), np.degrees(y), np.degrees(z)

    def calculate_distances(self):
        target_fps = 15.0
        frame_time = 1.0 / target_fps

        while True:
            start_time = time.time()

            ret, frame = self.cap.read()
            if not ret:
                print("카메라에서 프레임을 읽을 수 없습니다.")
                break

            frame_copy = frame.copy()
            frame_copy = cv2.addWeighted(frame_copy, 0.7, np.zeros(frame_copy.shape, frame_copy.dtype), 0, 0)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            distances = []
            yaws = []
            pitches = []
            rolls = []

            corners, ids, rejectedImgPoints = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters).detectMarkers(gray)

            if ids is not None:
                for i in range(len(ids)):
                    marker_corners_2d = corners[i][0]
                    success, rvec, tvec = cv2.solvePnP(self.marker_corners_3d, marker_corners_2d, self.camera_matrix, self.dist_coeffs)

                    if success:
                        # cv2.drawFrameAxes(frame_copy, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.02)
                        cv2.aruco.drawDetectedMarkers(frame_copy, corners)

                        distance = np.linalg.norm(tvec) * self.distance_adjustment
                        distances.append(distance)

                        R, _ = cv2.Rodrigues(rvec)
                        yaw, pitch, roll = self.rotation_matrix_to_euler_angles(R)
                        yaws.append(yaw)
                        pitches.append(pitch)
                        rolls.append(roll)

                        cv2.putText(frame_copy, f'{distance:.2f}m', (int(marker_corners_2d[0][0]), int(marker_corners_2d[0][1]) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        # cv2.putText(frame_copy, f'Yaw: {yaw:.1f}', (int(marker_corners_2d[0][0]), int(marker_corners_2d[0][1]) - 30),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        # cv2.putText(frame_copy, f'Pitch: {pitch:.1f}', (int(marker_corners_2d[0][0]), int(marker_corners_2d[0][1]) - 50),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                        # cv2.putText(frame_copy, f'Roll: {roll:.1f}', (int(marker_corners_2d[0][0]), int(marker_corners_2d[0][1]) - 70),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            distance_average = np.mean(distances)
            yaw_average = np.mean(yaws)
            pitch_average = np.mean(pitches)
            self.pitch_average = pitch_average
            roll_average = np.mean(rolls)
            self.distance_average = distance_average
            print(f'Average Distance: {distance_average:.2f} meters, Average Yaw: {yaw_average:.2f} degrees, Average Pitch: {pitch_average:.2f} degrees, Average Roll: {roll_average:.2f} degrees')

            results = self.model(frame)
            df = results.pandas().xyxy[0]
            fx = self.camera_matrix[0, 0]

            center_x = -1
            center_y = -1

            df = df[df['name'] == 'Socket']
            if len(df) > 0:
                highest_confidence_object = df.loc[df['confidence'].idxmax()]
                x1, y1, x2, y2, confidence, class_id, name = highest_confidence_object
                label = f'{name} {confidence:.2f}'
                cv2.rectangle(frame_copy, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame_copy, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                q1 = 10 - self.q0
                center_x = int((q1 * x1 + self.q0 * x2) / 10)
                p1 = 10 - self.p0
                center_y = int((p1 * y1 + self.p0 * y2) / 10)

            cam_center_x = self.screen_width // 2
            cam_center_y = self.screen_height // 2
            if center_x != -1 and center_y != -1:
                cv2.circle(frame_copy, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.circle(frame_copy, (cam_center_x, cam_center_y), 5, (255, 0, 0), -1)
                pixel_distance_x = cam_center_x - center_x
                pixel_distance_y = cam_center_y - center_y

                real_distance_x = self.pixel_to_real_distance(pixel_distance_x, distance_average, fx) * (1 / self.distance_adjustment)
                real_distance_y = self.pixel_to_real_distance(pixel_distance_y, distance_average, fx) * (1 / self.distance_adjustment)
                cv2.putText(frame_copy, f'X: {real_distance_x:.3f}m, Y: {real_distance_y:.3f}m', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                real_distance_x_turned = real_distance_x * np.cos(np.radians(roll_average)) - real_distance_y * np.sin(np.radians(roll_average))
                real_distance_y_turned = real_distance_x * np.sin(np.radians(roll_average)) + real_distance_y * np.cos(np.radians(roll_average))

                self.real_distance_x = real_distance_x_turned
                self.real_distance_y = real_distance_y_turned
                cv2.putText(frame_copy, f'X: {real_distance_x_turned:.3f}m, Y: {real_distance_y_turned:.3f}m', (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.putText(frame_copy, f'Average Distance = {distance_average:.2f} meters', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame_copy, f'Average Pitch = {pitch_average:.2f} degrees', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.imshow('YOLOv5 Object Detection', frame_copy)

            elapsed_time = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed_time)
            time.sleep(sleep_time)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

class StringPublisher:
    def __init__(self, calculator):
        # 노드를 초기화합니다.
        rospy.init_node('string_publisher_node', anonymous=True)

        # 퍼블리셔를 생성합니다.
        self.xyd_pub = rospy.Publisher('xyd_topic', String, queue_size=10)

        # 초기 메시지 설정
        self.calculator = calculator
        self.xyd_msg_data = "x0.001y0.002d0.003p0.004"

        # 퍼블리시 빈도를 설정합니다 (예: 1초에 한 번 퍼블리시).
        self.rate = rospy.Rate(1) # 1 Hz

        # 종료 이벤트 생성
        self.stop_event = threading.Event()

        # 스레드 시작
        self.publish_thread = threading.Thread(target=self.publish_strings)
        self.publish_thread.start()

    def publish_strings(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            # 실시간으로 업데이트된 메시지를 생성합니다.
            x = self.calculator.real_distance_x
            y = self.calculator.real_distance_y
            d = self.calculator.distance_average
            p = self.calculator.pitch_average
            self.xyd_msg_data = f"x{x:.3f}y{y:.3f}d{d:.3f}p{p:.3f}"

            xyd_msg = String()
            xyd_msg.data = self.xyd_msg_data

            # 토픽에 메시지를 발행합니다.
            self.xyd_pub.publish(xyd_msg)

            # 주기를 유지합니다.
            self.rate.sleep()

    def stop(self):
        self.stop_event.set()
        self.publish_thread.join()

if __name__ == "__main__":
    try:
        calculator = MarkerDistanceCalculator()
        publisher = StringPublisher(calculator)

        yolothread = threading.Thread(target=calculator.calculate_distances)
        yolothread.start()

        while yolothread.is_alive():
            time.sleep(0.5)
            print(f'Real Distance X: {calculator.real_distance_x:.3f}m, Real Distance Y: {calculator.real_distance_y:.3f}m, Average Distance: {calculator.distance_average:.3f}m')

        publisher.stop()
    except rospy.ROSInterruptException:
        pass
