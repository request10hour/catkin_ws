import socket
import time

def open_socket_for_receiving():
    while True:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.connect(('192.168.0.7', 8001))

            while True:
                data = server_socket.recv(1024)
                if not data:
                    break
                print(data.decode())

            server_socket.close()
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(5)  # 5초 대기 후 다시 시도

open_socket_for_receiving()
