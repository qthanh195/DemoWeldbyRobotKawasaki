from src.camera.camera_controller import BaslerCamera
from src.robot.robot_controller import RobotController
from src.utils.process_model import ProcessModel
from src.utils.convert_point import *
from src.utils.view import *

import threading
import numpy as np

def Process():
    z = -10
    # Khởi tạo camera và robot
    # camera = BaslerCamera()
    # camera.open_camera()
    
    # image = camera.get_image()
    image = cv2.imread("E:/2. GE/24. Kawasaki Robot/image1/img_20250614_181118.png")
    cv2.imwrite("test.png", image)
    # camera.close_camera()
    if image is None:
        print("Không thể lấy ảnh từ camera.")
        return
    
    center_part, line = ProcessModel().r_d(image)
    if center_part is None or line is None:
        print("Không tìm thấy vật.")
        return
    
    points1_robot = [pixel_to_robot(p) for p in generate_points_on_line(line[0][0], line[0][1], 70)]
    points2_robot = [pixel_to_robot(p) for p in generate_points_on_line(line[1][0], line[1][1], 70)]
    
    line1_oat = build_tcp_orientation_from_points(np.array([line[0][0][0],line[0][0][1],z]), 
                                                  np.array([line[0][1][0],line[0][1][1],z]), 
                                                  np.array([center_part[0],center_part[1],z]), 30)
    line2_oat = build_tcp_orientation_from_points(np.array([line[1][0][0],line[1][0][1],z]), 
                                                  np.array([line[1][1][0],line[1][1][1],z]), 
                                                  np.array([center_part[0],center_part[1],z]), 30)
    
    # line1_oat = euler_xyz_to_oat(-30, 0, angle_between_two_lines(((0,0),(500,0)), (line[0][0], line[0][1])))
    # line2_oat = euler_xyz_to_oat(-30, 0, angle_between_two_lines(((0,0),(500,0)), (line[1][0], line[1][1])))
    
    create_program("gerobot.pg", "gerobot",line1 = points1_robot,
                   line2 = points2_robot, rz_line1 = line1_oat, 
                   rz_line2 = line2_oat, center = pixel_to_robot(center_part))
    
    # thread_robot = threading.Thread(target=process_robot)
    # thread_robot.start()
    # thread_robot.join()
    
def process_robot():
    import time
    bot = RobotController()
    bot.initialize_robot()
    bot.connect_bot()
    #kiểm tra trạng thái robot trước khi edit chương trình
    status_robot = bot.status_robot()

    print(f"Trạng thái robot: {status_robot}")
    if status_robot == 0:
        print("Robot đang ở trạng thái có thể thực hiện chương trình")
        bot.load_program("gerobot")
    # bot.execute_program("gerobot,1")
    bot.disconnect_bot()
    
if __name__ == "__main__":
    Process()
    # process_robot()
