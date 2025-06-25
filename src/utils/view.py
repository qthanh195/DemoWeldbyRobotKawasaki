import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def draw_robot_points(points, mode='2d', show_axes=True):
    """
    Vẽ các điểm robot (x, y, z) trong không gian 2D hoặc 3D.

    Args:
        points: danh sách các điểm [(x, y, z), ...]
        mode: '2d' hoặc '3d'
        show_axes: nếu True, hiển thị trục tọa độ

    Returns:
        Hiển thị hình ảnh matplotlib
    """
    if mode == '2d':
        plt.figure(figsize=(8, 6))
        for i, (x, y, _) in enumerate(points):
            plt.scatter(x, y, color='blue')
            plt.text(x + 2, y + 2, f"P{i}", fontsize=8)

        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.title("Robot Coordinate Map (2D XY View)")
        plt.grid(True)
        if show_axes:
            plt.axhline(0, color='black', linewidth=0.5)
            plt.axvline(0, color='black', linewidth=0.5)
        plt.axis('equal')
        plt.show()

    elif mode == '3d':
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        xs, ys, zs = zip(*points)

        ax.scatter(xs, ys, zs, c='red', marker='o')
        for i, (x, y, z) in enumerate(points):
            ax.text(x, y, z + 5, f"P{i}", size=8)
            
        # nối các điểm
        for i in range(len(points) - 1):
            ax.plot([points[i][0], points[i + 1][0]],
                    [points[i][1], points[i + 1][1]],
                    [points[i][2], points[i + 1][2]], color='blue', linewidth=0.5)

        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title("Robot Workspace View (3D)")
        ax.grid(True)
        plt.show()
    else:
        raise ValueError("mode phải là '2d' hoặc '3d'")


def normalize(v):
    return v / np.linalg.norm(v)

def rotate_around_x(vec, angle_rad):
    R = np.array([
        [1, 0, 0],
        [0, np.cos(angle_rad), -np.sin(angle_rad)],
        [0, np.sin(angle_rad),  np.cos(angle_rad)]
    ])
    return R @ vec

def visualize_points_3d_with_tcp_frame(points, center=(0, 0, 0), rx_deg=0):
    points = np.array(points)
    points_3d = np.hstack([points, np.zeros((len(points), 1))])  # Thêm z=0

    start = points_3d[0]
    end = points_3d[-1]

    # Vector trục X: từ điểm đầu đến điểm cuối
    x_vec = normalize(end - start)

    # Vector từ điểm đầu đến tâm (dùng cho Y)
    center = np.array(center)
    y_vec = normalize(center - start - np.dot(center - start, x_vec) * x_vec)  # Vuông góc với X

    # Vector trục Z từ X x Y
    z_vec = normalize(np.cross(x_vec, y_vec))

    # Áp dụng xoay quanh X (tilt)
    rx_rad = np.deg2rad(rx_deg)
    y_vec = rotate_around_x(y_vec, rx_rad)
    z_vec = rotate_around_x(z_vec, rx_rad)

    # Vẽ
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], label='Path')

    # Vẽ khung TCP tại điểm đầu
    scale = 10
    ax.quiver(*start, *(x_vec*scale), color='r', label='X (path)')
    ax.quiver(*start, *(y_vec*scale), color='g', label='Y (to center)')
    ax.quiver(*start, *(z_vec*scale), color='b', label='Z')

    ax.scatter(*center, color='purple', s=50, label='Center')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'TCP Frame at Start with Rx = {rx_deg}°')
    ax.legend()
    ax.axis('auto')
    plt.show()
    
    
def draw_tcp_frame(ax, origin, x_dir, y_dir, z_dir=None, length=10, label='TCP'):
    """
    Vẽ một khung TCP tại vị trí origin với các hướng x, y, z.

    Parameters:
        ax     : trục 3D matplotlib
        origin : tuple/list/np.array (3,) - tọa độ điểm gốc
        x_dir  : hướng trục X (unit vector)
        y_dir  : hướng trục Y (unit vector)
        z_dir  : hướng trục Z (unit vector), nếu None sẽ tính bằng X x Y
        length : độ dài vector hiển thị
        label  : tên khung tọa độ
    """
    origin = np.array(origin)
    x_dir = np.array(x_dir)
    y_dir = np.array(y_dir)
    if z_dir is None:
        z_dir = np.cross(x_dir, y_dir)

    # Normalize các vector
    x_dir = x_dir / np.linalg.norm(x_dir)
    y_dir = y_dir / np.linalg.norm(y_dir)
    z_dir = z_dir / np.linalg.norm(z_dir)

    # Vẽ các vector
    ax.quiver(origin[0], origin[1], origin[2],
              x_dir[0]*length, x_dir[1]*length, x_dir[2]*length, color='r', label=f'{label} X')
    ax.quiver(origin[0], origin[1], origin[2],
              y_dir[0]*length, y_dir[1]*length, y_dir[2]*length, color='g', label=f'{label} Y')
    ax.quiver(origin[0], origin[1], origin[2],
              z_dir[0]*length, z_dir[1]*length, z_dir[2]*length, color='b', label=f'{label} Z')
    ax.text(*(origin + x_dir * length * 1.1), f'{label}_X', color='r')
    ax.text(*(origin + y_dir * length * 1.1), f'{label}_Y', color='g')
    ax.text(*(origin + z_dir * length * 1.1), f'{label}_Z', color='b')
    



def to_3d(pt):
    pt = np.array(pt)
    if pt.shape[0] == 2:
        return np.array([pt[0], pt[1], 0])
    return pt

def show_points_with_xyz_axes(points, x_dirs, y_dirs, z_dirs=None, length=2):
    """
    Hiển thị các điểm 3D và vẽ khung XYZ tại mỗi điểm.
    points: list of (x, y, z)
    x_dirs, y_dirs: list of vector định hướng X, Y tại mỗi điểm (phải cùng chiều dài với points)
    z_dirs: list of vector định hướng Z (nếu None sẽ tự tính bằng X x Y)
    length: độ dài các trục
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = zip(*[to_3d(p) for p in points])
    ax.scatter(xs, ys, zs, c='red', marker='o')

    for i, (origin, x_dir, y_dir) in enumerate(zip(points, x_dirs, y_dirs)):
        origin = to_3d(origin)
        x_dir = to_3d(x_dir)
        y_dir = to_3d(y_dir)
        if z_dirs is not None:
            z_dir = to_3d(z_dirs[i])
        else:
            z_dir = np.cross(x_dir, y_dir)
        draw_tcp_frame(ax, origin, x_dir, y_dir, z_dir, length=length, label=f'P{i}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Points with XYZ Axes')
    ax.legend()
    plt.show()
    
def show_points_3d(points, color='red', marker='o'):
    """
    Hiển thị các điểm 3D đơn giản, không vẽ hướng.
    points: list of (x, y, z)
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys,zs = zip(*points)
    ax.scatter(xs, ys, zs, c=color, marker=marker)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Points 3D')
    ax.set_xlim(-250, 150)
    ax.set_ylim(-250, 150)
    ax.set_zlim(-200, 200)
    plt.show()