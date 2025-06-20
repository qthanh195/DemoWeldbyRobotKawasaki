import socket
import time
import json
import os

class RobotController:
    def __init__(self, config_path="E:/2. GE/24. Kawasaki Robot\code/robot_weld_vision_demo\src/robot/robot_config.json"):
        self.config = self.load_config(config_path)
        self.bot = None

    def load_config(self, config_path):
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Không tìm thấy file cấu hình: {config_path}")
        with open(config_path, 'r') as f:
            return json.load(f)

    def initialize_robot(self):
        print("Robot initialized with config:")
        print(self.config)

    def connect_bot(self):
        try:
            self.bot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.bot.settimeout(5)  # Timeout để tránh treo khi không phản hồi
            self.bot.connect((self.config['ip'], self.config['port']))
            self.send_cmd("as")  # login
            print("Đã kết nối với robot.")
        except Exception as e:
            print(f"Lỗi khi kết nối robot: {e}")
            self.bot = None

    def disconnect_bot(self):
        if self.bot:
            self.bot.close()
            self.bot = None
            print("Đã ngắt kết nối với robot.")

    def send_cmd(self, cmd, wait_time=0.2):
        if not self.bot:
            raise ConnectionError("Chưa kết nối với robot.")
        try:
            print(f"Gửi: {cmd}")
            self.bot.sendall((cmd + '\r\n').encode())
            time.sleep(wait_time)
            response = self.bot.recv(4096).decode(errors='ignore')
            print(f"Phản hồi: {response.strip()}")
            return response
        except socket.timeout:
            print("Hết thời gian chờ phản hồi từ robot.")
        except Exception as e:
            print(f"Lỗi khi gửi lệnh: {e}")

    def load_program(self, program_name):
        # xoa chuowng trinh
        self.send_cmd("DELETE/P test2006")
        self.send_cmd("1")
        
        self.send_cmd(f"load {program_name}")

        # while True:
        #     res = "2"
        #     # res = self.send_cmd(f'type EXISTPGM ("{program_name}")')
        #     res=self.send_cmd(f'type EXISTPGM ("test2006")')
        #     # res = self.bot.recv(4096).decode(errors='ignore')
        #     print(res)
        #     if res.strip() == "-1":
        #         break

    def execute_program(self, program_name):
        return self.send_cmd(f"execute {program_name}")

    def hold_program(self):
        return self.send_cmd("hold")

    def continue_program(self):
        return self.send_cmd("continue")

    def stop_program(self):
        return self.send_cmd("ABORT")
    
  