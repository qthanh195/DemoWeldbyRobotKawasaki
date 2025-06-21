import socket
import time
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

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
    
def calibrate_kawasaki_base_from_3_points(p0, p1, p2):
    """
    Tính base TRANS(x, y, z, o, a, t) từ 3 điểm robot: p0 (origin), p1 (trục X), p2 (trục XY)

    Args:
        p0: gốc hệ tọa độ mới, dạng (x, y, z)
        p1: điểm trên trục X
        p2: điểm trên mặt phẳng X-Y

    Returns:
        Chuỗi TRANS(x, y, z, o, a, t)
    """

    # Chuyển về vector numpy
    p0 = np.array(p0)
    p1 = np.array(p1)
    p2 = np.array(p2)

    # Vector trục X mới
    x_vec = p1 - p0
    x_vec = x_vec / np.linalg.norm(x_vec)

    # Vector phụ
    temp_vec = p2 - p0

    # Trục Z = X × temp_vec
    z_vec = np.cross(x_vec, temp_vec)
    z_vec = z_vec / np.linalg.norm(z_vec)

    # Trục Y = Z × X
    y_vec = np.cross(z_vec, x_vec)

    # Ma trận quay: cột là [x, y, z]
    R_mat = np.column_stack((x_vec, y_vec, z_vec))

    # Chuyển thành OAT (Euler ZYZ)
    r = R.from_matrix(R_mat)
    o, a, t = r.as_euler('ZYZ', degrees=True)

    # Lấy vị trí gốc
    x, y, z = p0

    return f"TRANS({x:.3f}, {y:.3f}, {z:.3f}, {o:.3f}, {a:.3f}, {t:.3f})"

def calibrate_transformation_from_3points(camera_points, robot_points):
    """
    Tính TRANS(x, y, z, o, a, t) từ 3 điểm tương ứng giữa hệ camera và hệ robot.

    Args:
        camera_points: List 3 điểm camera [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2]]
        robot_points: List 3 điểm robot  [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2]]

    Returns:
        (x, y, z, o, a, t): tọa độ robot theo chuẩn Kawasaki
    """

    cam = np.array(camera_points, dtype=np.float64)
    rob = np.array(robot_points, dtype=np.float64)

    # 1. Gốc tọa độ
    cam_origin = cam[0]
    rob_origin = rob[0]

    # 2. Trục X: từ P0 → P1
    cam_x = cam[1] - cam[0]
    rob_x = rob[1] - rob[0]

    # 3. Trục Z: pháp tuyến mặt P0-P1-P2
    cam_z = np.cross(cam[1] - cam[0], cam[2] - cam[0])
    rob_z = np.cross(rob[1] - rob[0], rob[2] - rob[0])

    # 4. Trục Y: vuông góc X-Z
    cam_y = np.cross(cam_z, cam_x)
    rob_y = np.cross(rob_z, rob_x)

    # Chuẩn hóa trục
    def normalize(v):
        return v / np.linalg.norm(v)

    R_cam = np.column_stack([normalize(cam_x), normalize(cam_y), normalize(cam_z)])
    R_rob = np.column_stack([normalize(rob_x), normalize(rob_y), normalize(rob_z)])

    # 5. Ma trận quay từ camera → robot
    R_transform = R_rob @ R_cam.T  # Vì: R_cam * V_cam = V_local => V_rob = R_rob * V_local

    # 6. Vector tịnh tiến
    T = rob_origin - R_transform @ cam_origin

    # 7. Chuyển thành góc OAT (Euler ZYZ)
    r = R.from_matrix(R_transform)
    o, a, t = r.as_euler('ZYZ', degrees=True)

    x, y, z = T
    return x, y, z, o, a, t