import socket
import time
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

###◦
# 0: Chương trình không trong quá trình thực thi.
# ◦
# 1: Chương trình đang chạy.
# ◦
# 2: Chương trình đang ở trạng thái giữ (held).
# ◦
# 3: Việc thực thi bước (stepper) đã hoàn thành; đang chờ hoàn thành chuyển động của robot.


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

    def recv_response(self):
        """Nhận phản hồi từ robot."""
        # Trong thực tế, bạn sẽ nhận dữ liệu từ socket
        response = ""
        while True:
            response = self.bot.recv(4096).decode(errors='ignore')
            if response == ">":
                    break
        # Vì đây là ví dụ, chúng ta sẽ giả lập phản hồi
        # return self.send_cmd("SIMULATED_RECEIVE")
    
    def connect_bot(self):
        try:
            self.bot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.bot.settimeout(5)  # Timeout để tránh treo khi không phản hồi
            self.bot.connect((self.config['ip'], self.config['port']))
            self.send_cmd("as")  # login

            # Đợi đến khi nhận được dấu nhắc '>'
            buffer = ""
            while True:
                response = self.bot.recv(4096).decode(errors='ignore')
                buffer += response
                if ">" in response:
                    break

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
        # program_name = "test1806"

        # Bước 1: Vào chế độ soạn thảo cho chương trình
        self.send_cmd(f"edit {program_name}")
        time.sleep(0.1) # Đợi robot phản hồi và vào Editor Mode

        # Bước 2: Xóa tất cả các dòng hiện có trong chương trình
        # Chuyển đến dòng đầu tiên
        self.send_cmd("S 1")
        time.sleep(0.05)
        # Xóa một số lượng lớn các bước từ dòng hiện tại trở đi để làm sạch hoàn toàn chương trình
        self.send_cmd("D 9999") # D 9999 sẽ xóa 9999 dòng từ dòng hiện tại. [5.1, 98]
        time.sleep(0.1) # Đợi quá trình xóa hoàn tất

        # Bước 3: Đọc và chèn các dòng chương trình mới
        file_path = "E:/2. GE/24. Kawasaki Robot/code/robot_weld_vision_demo/gerobot.pg"
        with open(file_path, "r") as file:
            lines = file.readlines()
            # Bỏ dòng đầu tiên (.PROGRAM tên_chương_trình) và dòng cuối cùng (.END)
            # vì lệnh EDIT và E đã xử lý phần này
            program_content_lines = []
            for line in lines[1:-1]:
                stripped_line = line.strip()
                if stripped_line: # Chỉ thêm các dòng không trống
                    program_content_lines.append(stripped_line)

        # Chèn từng dòng nội dung mới vào chương trình trống
        # Editor mode sẽ tự động tăng số bước sau mỗi lần gửi lệnh
        for line_content in program_content_lines:
            self.send_cmd(line_content)
            time.sleep(0.01) # Thêm độ trễ nhỏ giữa các lệnh để robot xử lý

        # Bước 4: Thoát chế độ soạn thảo
        self.send_cmd("e")
        time.sleep(0.1) # Đợi robot thoát Editor Mode

        print(f"Chương trình '{program_name}' đã được tải thành công với {len(program_content_lines)} dòng.")

    
    def execute_program(self, program_name):
        return self.send_cmd(f"execute {program_name}")
    
    def delete_program(self, program_name):
        return self.send_cmd(f"delete/P {program_name}")

    def hold_program(self):
        return self.send_cmd("hold")

    def continue_program(self):
        return self.send_cmd("continue")

    def stop_program(self):
        return self.send_cmd("ABORT")
    
    def status_robot(self):
        response = self.send_cmd("TYPE TASK(1)")
        # Tách các dòng trong phản hồi
        lines = response.splitlines()
        for line in lines:
            line = line.strip()
            # Bỏ qua dòng lệnh và dấu nhắc, chỉ lấy dòng là số
            if line.isdigit():
                return int(line)
        print("Không tìm thấy trạng thái robot hợp lệ trong phản hồi.")
        return None
