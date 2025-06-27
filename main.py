from src.camera.camera_controller import BaslerCamera
from src.robot.robot_controller import RobotController
from src.utils.process_model import ProcessModel
from src.utils.convert_point import *
from src.utils.view import *

import threading
import numpy as np
import time

def Process():
    fig, ax = realtime_3d_plot_init()
    z = -5
    while True:
        start_time = time.time()
        # Khởi tạo camera và robot
        camera = BaslerCamera()
        camera.open_camera()
        
        image = camera.get_image()
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
        
        points3_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[0][0], lineb[0][1], 70)]
        points4_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[1][0], lineb[1][1], 70)]
        
        #kiểm tra points3_robot và points4_robot xem cạnh nào nằm trên thì nó là points3_robot
        if points3_robot[0][0] > points4_robot[0][0]:
            points3_robot, points4_robot = points4_robot, points3_robot

        rz_line1 = angle_between_two_lines(((0,0),(3000,0)), (linea[0][0], linea[0][1]))
        rz_line2 = angle_between_two_lines(((0,0),(3000,0)), (linea[1][0], linea[1][1]))
        rz_line3 = angle_between_two_lines(((0,0),(3000,0)), (lineb[0][0], lineb[0][1]))
        rz_line4 = angle_between_two_lines(((0,0),(3000,0)), (lineb[1][0], lineb[1][1]))
        
        print("rz_line1: ", rz_line1)
        print("rz_line2: ", rz_line2)
        print("rz_line3: ", rz_line3)     
        print("rz_line4: ", rz_line4)
        
        point_line5_robot = points3_robot[0]
        point_line6_robot = points3_robot[-1]
        point_line7_robot = points4_robot[0]
        point_line8_robot = points4_robot[-1]
        
        #  đảm bảo point_line5_robot nằm trên bên phải, point_line6_robot nằm trên bên trái, point_line7_robot nằm dưới bên , point_line8_robot nằm dưới bên trái
        # Gom các điểm lại
        all_points = [points3_robot[0], points3_robot[-1], points4_robot[0], points4_robot[-1]]

        # Sắp xếp theo trục Y (từ nhỏ đến lớn => trên trước, dưới sau)
        sorted_by_y = sorted(all_points, key=lambda p: p[1])

        # 2 điểm trên cùng (Y nhỏ nhất)
        top_points = sorted_by_y[:2]
        # 2 điểm dưới cùng (Y lớn hơn)
        bottom_points = sorted_by_y[2:]

        # Với mỗi nhóm, sắp theo trục X để biết trái/phải
        top_points_sorted = sorted(top_points, key=lambda p: p[0])     # trái -> phải
        bottom_points_sorted = sorted(bottom_points, key=lambda p: p[0])  # trái -> phải

        # Gán lại đúng vị trí mong muốn
        point_line6_robot = top_points_sorted[0]  # trên trái
        point_line5_robot = top_points_sorted[1]  # trên phải
        point_line8_robot = bottom_points_sorted[0]  # dưới trái
        point_line7_robot = bottom_points_sorted[1]  # dưới phải

        
        line5_robot = []
        line6_robot = []
        line7_robot = []
        line8_robot = []
        
        # line là danh sách điểm có x,y cố đinh = point_line5 và z thay đổi từ 5 đến 8 cách nhau 1
        for i in range(5, 80, 10):
            line5_robot.append((point_line5_robot[0], point_line5_robot[1], -i))
        
        for i in range(5, 80, 10):
            line6_robot.append((point_line6_robot[0], point_line6_robot[1], -i))
        for i in range(5, 80, 10):
            line7_robot.append((point_line7_robot[0], point_line7_robot[1], -i))

        for i in range(5, 80, 10):
            line8_robot.append((point_line8_robot[0], point_line8_robot[1], -i))
        
        create_program("gerobot.pg", "gerobot",line1 = points1_robot,
                    line2 = points2_robot, line3 = points3_robot, line4 = points4_robot,
                    rz_line1 = rz_line1, rz_line2 = rz_line2,rz_line3 = rz_line3, rz_line4 = rz_line4,
                    line5 = line5_robot, line6 = line6_robot, line7 = line7_robot, line8 = line8_robot ,
                    center = pixel_to_robot(center_part))
        
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
 
        # danh sach cac diem
        points = []
        for p in points1_robot:
            points.append((p[0], p[1], z))
        for p in points2_robot:
            points.append((p[0], p[1], z))
        for p in points3_robot:
            points.append((p[0], p[1], z))
        for p in points4_robot:
            points.append((p[0], p[1], z))
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
        
        cv2.imwrite("test.png", img)
        resize = cv2.resize(img, (0, 0), fx=0.4, fy=0.4)
        points = np.array(points)
        cv2.imshow("Image", resize)
        
        realtime_3d_plot_update(ax, points)
        
        thread_robot = threading.Thread(target=process_robot)
        thread_robot.start()
        thread_robot.join()
        
        print("thoi gian = ", time.time() - start_time)
        key = cv2.waitKey(0)  # Chờ nhấn phím bất kỳ
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