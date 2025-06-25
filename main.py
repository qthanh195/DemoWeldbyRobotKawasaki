from src.camera.camera_controller import BaslerCamera
from src.robot.robot_controller import RobotController
from src.utils.process_model import ProcessModel
from src.utils.convert_point import *
from src.utils.view import *

import threading
import numpy as np

def Process():
    while True:
        z = -5
        # Khởi tạo camera và robot
        camera = BaslerCamera()
        camera.open_camera()
        
        image = camera.get_image()
        # image = cv2.imread("E:/2. GE/24. Kawasaki Robot/image1/img_20250614_181118.png")
        cv2.imwrite("test.png", image)
        camera.close_camera()
        if image is None:
            print("Không thể lấy ảnh từ camera.")
            return
        
        center_part, linea, lineb = ProcessModel().r_d(image)
        if center_part is None or linea is None or lineb is None:
            print("Không tìm thấy vật.")
            return
        
        points1_robot = [pixel_to_robot(p) for p in generate_points_on_line(linea[0][0], linea[0][1], 70)]
        points2_robot = [pixel_to_robot(p) for p in generate_points_on_line(linea[1][0], linea[1][1], 70)]
        
        #kiểm tra points1_robot và points2_robot xem cạnh nào nằm bên phải thì nó là points1_robot
        if points1_robot[0][0] < points2_robot[0][0]:
            points1_robot, points2_robot = points2_robot, points1_robot
        
        line1_oat = build_tcp_orientation_from_points(np.array([linea[0][0][0],linea[0][0][1],z]), 
                                                    np.array([linea[0][1][0],linea[0][1][1],z]), 
                                                    np.array([center_part[0],center_part[1],z]), 160)
        line2_oat = build_tcp_orientation_from_points(np.array([linea[1][0][0],linea[1][0][1],z]), 
                                                    np.array([linea[1][1][0],linea[1][1][1],z]), 
                                                    np.array([center_part[0],center_part[1],z]), -40)
        
        points3_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[0][0], lineb[0][1], 70)]
        points4_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[1][0], lineb[1][1], 70)]
        
        #kiểm tra points3_robot và points4_robot xem cạnh nào nằm trên thì nó là points3_robot
        if points3_robot[0][0] > points4_robot[0][0]:
            points3_robot, points4_robot = points4_robot, points3_robot
        
        line3_oat = build_tcp_orientation_from_points(np.array([lineb[0][0][0],lineb[0][0][1],z]),
                                                    np.array([lineb[1][0][0],lineb[1][0][1],z]),
                                                    np.array([center_part[0],center_part[1],z]), 20)
        line4_oat = build_tcp_orientation_from_points(np.array([lineb[1][0][0],lineb[1][0][1],z]),
                                                    np.array([lineb[1][1][0],lineb[1][1][1],z]),
                                                    np.array([center_part[0],center_part[1],z]), 20)

        rz_line1 = angle_between_two_lines(((0,0),(500,0)), (linea[0][0], linea[0][1]))
        rz_line2 = angle_between_two_lines(((0,0),(500,0)), (linea[1][0], linea[1][1]))
        rz_line3 = angle_between_two_lines(((0,0),(500,0)), (lineb[0][0], lineb[0][1]))
        rz_line4 = angle_between_two_lines(((0,0),(500,0)), (lineb[1][0], lineb[1][1]))
        
        # line1_oat = euler_xyz_to_oat(0,0,-30)
        # line2_oat = euler_xyz_to_oat(0,0,30)
        # line3_oat = euler_xyz_to_oat(0,0,-30)
        # line4_oat = euler_xyz_to_oat(0,0,30)
        
        # create_program("gerobot.pg", "gerobot",line1 = points1_robot,
        #                line2 = points2_robot, line3 = points3_robot, line4 = points4_robot,
        #                rz_line1 = line1_oat, rz_line2 = line2_oat,rz_line3 = line3_oat, rz_line4 = line4_oat,
        #                center = pixel_to_robot(center_part))
        create_program("gerobot.pg", "gerobot",line1 = points1_robot,
                    line2 = points2_robot, line3 = points3_robot, line4 = points4_robot,
                    rz_line1 = rz_line1, rz_line2 = rz_line2,rz_line3 = rz_line3, rz_line4 = rz_line4,
                    center = pixel_to_robot(center_part))
        



        # thread_robot = threading.Thread(target=process_robot)
        # thread_robot.start()
        # thread_robot.join()
        
        

        
        point_line5 = points3_robot[0]
        line5 = []
        # line là danh sách điểm có x,y cố đinh = point_line5 và z thay đổi từ 5 đến 40 cách nhau 5
        for i in range(5, 40, 7):
            line5.append((point_line5[0], point_line5[1], i))
            
        point_line6 = points3_robot[-1]
        line6 = []
        # line là danh sách điểm có x,y cố đinh = point_line5 và z thay đổi từ 5 đến 40 cách nhau 5
        for i in range(5, 40, 7):
            line6.append((point_line6[0], point_line6[1], i))

        point_line7 = points4_robot[0]
        line7 = []
        # line là danh sách điểm có x,y cố đinh = point_line5 và z thay đổi từ 5 đến 40 cách nhau 5
        for i in range(5, 40, 7):
            line7.append((point_line7[0], point_line7[1], i))

        point_line8 =   points4_robot[-1]
        line8 = []
        # line là danh sách điểm có x,y cố đinh = point_line5 và z thay đổi từ 5 đến 40 cách nhau 5
        for i in range(5, 40, 7):
            line8.append((point_line8[0], point_line8[1], i))

        # print
        
        
        # danh sach cac diem
        points = []
        for p in points1_robot:
            points.append((p[0], p[1], 5))
        for p in points2_robot:
            points.append((p[0], p[1], 5))
        for p in points3_robot:
            points.append((p[0], p[1], 5))
        for p in points4_robot:
            points.append((p[0], p[1], 5))
        for p in line5:
                points.append(p)
        for p in line6:
                points.append(p)
        for p in line7:
                points.append(p)
        for p in line8:
                points.append(p)
        
        
        points1 = generate_points_on_line(linea[0][0], linea[0][1], 70)
        points2 = generate_points_on_line(linea[1][0], linea[1][1], 70)
        points3 = generate_points_on_line(lineb[0][0], lineb[0][1], 70)
        points4 = generate_points_on_line(lineb[1][0], lineb[1][1], 70)
        
        line1 = get_line_directions(points1[0], points1[-1], center_part)
        line2 = get_line_directions(points2[0], points2[-1], center_part)
        line3 = get_line_directions(points3[0], points3[-1], center_part)
        line4 = get_line_directions(points4[0], points4[-1], center_part)
        
        line1_test = []
        for point in points1:
            line1_test.append({
                "point": (point[0],point[1]),
                "ux": line1['ux'],
                "uy": line1['uy'],
                "vx": line1['vx'],
                "vy": line1['vy'],
            })

        line2_test = []
        for point in points2:
            line2_test.append({
                "point": (point[0],point[1]),
                "ux": line2['ux'],
                "uy": line2['uy'],
                "vx": line2['vx'],
                "vy": line2['vy'],
            })
        line3_test = []
        for point in points3:
            line3_test.append({
                "point": (point[0],point[1]),
                "ux": line3['ux'],
                "uy": line3['uy'],
                "vx": line3['vx'],
                "vy": line3['vy'],
        })
        line4_test = []
        for point in points4:
            line4_test.append({
                "point": (point[0],point[1]),
                "ux": line4['ux'],
                "uy": line4['uy'],
                "vx": line4['vx'],
                "vy": line4['vy'],
            })
            
        img = cv2.imread("test.png")
        draw_points_with_xy_direction(img, line1_test, 30, (0, 0, 255), (0, 255, 0))
        draw_points_with_xy_direction(img, line2_test, 30, (0, 0, 255), (0, 255, 0))
        draw_points_with_xy_direction(img, line3_test, 30, (0, 0, 255), (0, 255, 0))
        draw_points_with_xy_direction(img, line4_test, 30, (0, 0, 255), (0, 255, 0))
        # cv2.line(image, (linea[0][0][0],linea[0][0][1]), (linea[0][1][0],linea[0][1][1]), (0, 255, 0), 2)
        # cv2.line(image, (linea[1][0][0],linea[1][0][1]), (linea[1][1][0],linea[1][1][1]), (0, 255, 0), 2)
        
        # cv2.line(image, (lineb[1][0][0],lineb[1][0][1]), (lineb[1][1][0],lineb[1][1][1]), (0, 255, 0), 2)
        # cv2.line(image, (lineb[0][0][0],lineb[0][0][1]), (lineb[0][1][0],lineb[0][1][1]), (0, 255, 0), 2)
        # cv2.circle(image, (int(center_part[0]), int(center_part[1])), 10, (0, 0, 255), 2)
        # cv2.line(image, (0, 1374), (3840, 1374), (0, 255, 0), 2)
        # cv2.line(image, (1920, 0), (1920, 2748), (0, 255, 0), 2)
        cv2.imwrite("test.png", img)
        resize = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("Image", resize)
        show_points_3d(points)
        key = cv2.waitKey(0)  # Chờ nhấn phím bất kỳ
        # plt.close()
        if key == 27:  # Nhấn ESC để thoát
            break
    cv2.destroyAllWindows()
       
    
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
        bot.execute_program("gerobot,1")
    bot.disconnect_bot()
    
if __name__ == "__main__":

    Process()
        
    # while True:
    #     Process()
    #     ti

    # process_robot()
