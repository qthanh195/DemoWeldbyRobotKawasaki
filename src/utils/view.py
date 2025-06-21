import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
