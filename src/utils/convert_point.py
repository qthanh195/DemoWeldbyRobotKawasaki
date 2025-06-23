import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation as R

def generate_points_on_line(p1, p2, step=70):
    """
    Tạo danh sách các điểm nằm trên đoạn thẳng giữa p1 và p2 với khoảng cách đều nhau.

    Parameters:
        p1 (tuple): Điểm đầu (x1, y1)
        p2 (tuple): Điểm cuối (x2, y2)
        step (float): Khoảng cách giữa các điểm

    Returns:
        list of (x, y): Các điểm tạo ra
    """
    x1, y1 = p1
    x2, y2 = p2

    # Vector chỉ phương
    dx = x2 - x1
    dy = y2 - y1
    length = np.hypot(dx, dy)

    # Số bước chia
    num_steps = int(length // step)
    points = []

    for i in range(num_steps + 1):
        ratio = i * step / length
        x = x1 + ratio * dx
        y = y1 + ratio * dy
        points.append(tuple(map(int,(x, y))))

    return points

def pixel_to_robot(p, pixel_per_mm=7, image_size=(3848, 2748), invert_x=False, invert_y=False):
    """
    Chuyển từ tọa độ pixel sang tọa độ robot (mm), với tuỳ chọn đảo trục X hoặc Y.

    Args:
        px, py: Tọa độ pixel trên ảnh
        pixel_per_mm: 7 px = 1 mm
        image_size: (width, height) của ảnh
        invert_x: nếu True, đảo trục X (robot X ngược với camera X)
        invert_y: nếu True, đảo trục Y

    Returns:
        (x_mm, y_mm): Tọa độ robot tính theo mm
    """
    px, py = p[0], p[1]
    cx, cy = image_size[0] // 2, image_size[1] // 2

    dx = px - cx
    dy = py - cy

    if invert_x:
        dx = -dx
    if invert_y:
        dy = -dy

    x_mm = dx / pixel_per_mm
    y_mm = dy / pixel_per_mm

    return x_mm, y_mm


def get_line_directions(p1, p2, center):
    """
    Tính vector đơn vị hướng X (dọc đường thẳng) và Y (vuông góc hướng vào center)
    
    Args:
        p1: tuple (x1, y1) - điểm bắt đầu
        p2: tuple (x2, y2) - điểm kết thúc
        center: tuple (cx, cy) - tâm để xác định hướng Y
        
    Returns:
        dict với các giá trị ux, uy, vx, vy
    """
    # Vector từ p1 đến p2
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = np.hypot(dx, dy)

    if length == 0:
        raise ValueError("p1 và p2 trùng nhau, không xác định được hướng.")

    # Hướng X: dọc theo đoạn thẳng
    ux = dx / length
    uy = dy / length

    # Hướng Y: vuông góc với X
    vx = -uy
    vy = ux

    # Kiểm tra hướng Y có hướng vào center chưa
    to_center = np.array([center[0] - p1[0], center[1] - p1[1]])
    y_dir = np.array([vx, vy])

    if np.dot(to_center, y_dir) < 0:
        vx, vy = -vx, -vy

    return {
        'ux': ux, 'uy': uy,
        'vx': vx, 'vy': vy
    }

def draw_points_with_xy_direction(img, points_data, scale=30, color_x=(0, 0, 255), color_y=(0, 255, 0)):
    """
    Vẽ điểm và mũi tên X (đỏ) và Y (xanh lá) lên ảnh.
    
    Parameters:
        img: ảnh gốc (numpy array, BGR)
        points_data: list chứa {'point': (x, y), 'dx': ..., 'dy': ...}
        scale: độ dài mũi tên
    """
    for item in points_data:
        x, y = item['point']
        
        ux, uy, vx, vy = item['ux'], item['uy'], item['vx'], item['vy']

        p = (int(x), int(y))

        # Điểm kết thúc mũi tên X
        p_x = (int(x + ux * scale), int(y + uy * scale))

        # Điểm kết thúc mũi tên Y
        p_y = (int(x + vx * scale), int(y + vy * scale))

        cv2.circle(img, p, 3, (255, 255, 255), -1)  # vẽ điểm trắng
        cv2.arrowedLine(img, p, p_x, color_x, 2, tipLength=0.3)  # mũi tên X (đỏ)
        cv2.arrowedLine(img, p, p_y, color_y, 2, tipLength=0.3)  # mũi tên Y (xanh)
        
def get_rx_ry_rz(dir_line):
    ux = dir_line['ux']
    uy = dir_line['uy']
    vx = dir_line['vx']
    vy = dir_line['vy']
    # Vector X, Y
    from scipy.spatial.transform import Rotation as R
    X = np.array([ux, uy, 0])
    Y = np.array([vx, vy, 0])
    Z = np.cross(X, Y)

    # Ma trận xoay 3x3 (các vector là cột)
    R_mat = np.column_stack((X, Y, Z))

    # Tạo rotation object từ ma trận
    rot = R.from_matrix(R_mat)

    # Chuyển thành Euler angles (deg)
    return rot.as_euler('xyz', degrees=True)

def angle_between_two_lines(p1, p2):
    p1_start, p1_end = np.array(p1[0]), np.array(p1[1])
    p2_start, p2_end = np.array(p2[0]), np.array(p2[1])
    # Tính vector chỉ phương
    v1 = np.array([p1_end[0] - p1_start[0], p1_end[1] - p1_start[1]])
    v2 = np.array([p2_end[0] - p2_start[0], p2_end[1] - p2_start[1]])
    
    # Chuẩn hóa
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    
    # Tính dot product
    dot = np.dot(v1_norm, v2_norm)
    
    # Tránh lỗi do sai số máy
    dot = np.clip(dot, -1.0, 1.0)
    
    # Góc (rad → độ)
    angle_rad = np.arccos(dot)
    angle_deg = np.degrees(angle_rad)
    if p2[0][1]> p2[1][1]:
        angle_deg += 180
    
    
    
    return angle_deg

def euler_xyz_to_oat(rx, ry, rz):
    """
    Convert từ góc Euler XYZ sang Euler ZYZ (o,a,t) dùng cho robot Kawasaki

    rx, ry, rz: góc quay theo trục X, Y, Z (đơn vị: độ)

    Returns:
        o, a, t: tương ứng với TRANS(x, y, z, o, a, t)
    """
    # Tạo rotation theo thứ tự Euler XYZ (roll, pitch, yaw)
    r = R.from_euler('xyz', [rx, ry, rz], degrees=True)

    # Chuyển sang Euler ZYZ (dạng của Kawasaki)
    o, a, t = r.as_euler('ZYZ', degrees=True)
    return o, a, t

def create_program(file_path, program_name, line1, line2, rz_line1, rz_line2, center):
    with open(file_path, 'w') as file:
        file.write(f".PROGRAM {program_name}()\n")
        file.write(f"   SPEED 30\n")
        file.write(f"   ACCURACY 1\n")
        file.write(f"   BASE TRANS(434.862, 151.696, -285.698, 0.823, 179.381, -89.907)\n")
        file.write(f"   JMOVE TRANS({center[0]:.3f},{center[1]:.3f},-300,0,0,0)\n")
        # file.write(f"   TDRAW 0,0,0,0,0,{rz_line1:.3f}\n")
        # file.write(f"   TDRAW 0,0,0,-20,0,0\n")
        file.write(f"   TDRAW 0,0,0,{rz_line1[0]:.3f},{rz_line1[1]:.3f},{rz_line1[2]:.3f}\n")

        file.write(f"   POINT current_pose = HERE\n")
        file.write(f"   current_O = DEXT(current_pose, 4)\n")
        file.write(f"   current_A = DEXT(current_pose, 5)\n")
        file.write(f"   current_T = DEXT(current_pose, 6)\n")
        
        for point in line1:
            x, y = point
            file.write(f"   LMOVE TRANS({x:.3f},{y:.3f},-10.000,current_O,current_A,current_T)\n")
            
        file.write(f"   JMOVE TRANS({center[0]:.3f},{center[1]:.3f},-300,0,0,0)\n")
        # file.write(f"   TDRAW 0,0,0,0,0,{rz_line2:.3f}\n")
        # file.write(f"   TDRAW 0,0,0,-20,0,0\n")
        file.write(f"   TDRAW 0,0,0,{rz_line2[0]:.3f},{rz_line2[1]:.3f},{rz_line2[2]:.3f}\n")
        
        file.write(f"   POINT current_pose = HERE\n")
        file.write(f"   current_O = DEXT(current_pose, 4)\n")
        file.write(f"   current_A = DEXT(current_pose, 5)\n")
        file.write(f"   current_T = DEXT(current_pose, 6)\n")
        
        for point in line2:
            x, y = point
            file.write(f"   LMOVE TRANS({x:.3f},{y:.3f},-10.000,current_O,current_A,current_T)\n")
        file.write(f"   JMOVE TRANS({center[0]:.3f},{center[1]:.3f},-300,0,0,0)\n")
        file.write(f"   JMOVE TRANS(-126,192,-266,0,0,0)\n")
        file.write(f".END\n")
        
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

def build_tcp_orientation_from_points(p1, p2, center, tilt_angle_deg):
    """
    Tạo hướng TCP (OAT) từ hai điểm đầu-cuối và tâm của vật thể.
    
    Args:
        p1: np.array([x, y, z]) - Điểm đầu của cạnh
        p2: np.array([x, y, z]) - Điểm cuối của cạnh
        center: np.array([x, y, z]) - Tâm vật thể (để xác định hướng nghiêng)
        tilt_angle_deg: float - Góc nghiêng (độ) của trục Z tool so với pháp tuyến
    
    Returns:
        (o, a, t): tuple[float, float, float] - Góc OAT cho robot Kawasaki
    """

    # 1. Tính vector cạnh (trục X)
    edge_vec = p2 - p1
    x = edge_vec / np.linalg.norm(edge_vec)

    # 2. Vector hướng từ cạnh đến tâm
    to_center = center - (p1 + p2) / 2
    to_center = to_center / np.linalg.norm(to_center)

    # 3. Góc nghiêng (rad)
    tilt_rad = np.radians(tilt_angle_deg)

    # 4. Trục Z nghiêng hướng về phía ngoài (lệch tilt so với hướng vào tâm)
    z = np.cos(tilt_rad) * to_center + np.sin(tilt_rad) * np.cross(to_center, x)
    z = z / np.linalg.norm(z)

    # 5. Trục Y vuông góc với Z và X
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)

    # 6. Tạo ma trận quay
    R_mat = np.column_stack((x, y, z))  # [X Y Z] cột

    # 7. Chuyển sang Euler Z-Y-Z (OAT)
    r = R.from_matrix(R_mat)
    o, a, t = r.as_euler('ZYZ', degrees=True)
    
    return o, a, t