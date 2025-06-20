# from src.camera.camera_controller import BaslerCamera
# import threading
# import cv2
# from src.robot.robot_controller import RobotController
# import time

# def main():
#     camera = BaslerCamera()
#     camera.open_camera()
#     thread_camera = threading.Thread(target=camera.continuous_grab)
#     thread_camera.start()
#     image = None
#     while True:
#         if not camera.image.empty():
#             print("image is not empty")
#             image = camera.image.get()
#             # vẽ đường thẳng trên ảnh
#             cv2.line(image, (0, 1474), (5000, 1474), (0, 255, 0), 2)
#             cv2.line(image, (0, 1684), (5000, 1684), (0, 255, 0), 2)
#             cv2.line(image, (2439, 0), (2439, 5000), (0, 255, 0), 2)
#             cv2.line(image, (2649, 0), (2649, 5000), (0, 255, 0), 2)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         if image is not None:
#             img_resize = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 
#             cv2.imshow("Camera", img_resize)
#     cv2.destroyAllWindows()
#     camera.continuous_grab()
#     camera.close_camera()
#     thread_camera.join()
    
def control_robot():
    # %
    from src.robot.robot_controller import RobotController
    controller = RobotController()
    controller.initialize_robot()
    controller.connect_bot()
    # %
    # controller.load_program("tepp.pg")
    # # %
    # controller.execute_program("testProgramFromPc.pg,-1")

    # %
    # controller.hold_program()

    # # %
    # controller.continue_program()

    # # %
    # controller.stop_program()
    # # %
    # time.sleep(10)
    controller.load_program("test1906.pg")
    time.sleep(10)
    controller.disconnect_bot()
    
def test():
    from src.utils.process_model import ProcessModel
    import cv2
    from src.utils.convert_point import generate_points_on_line, camera_to_robot, get_line_directions, draw_points_with_xy_direction, get_rx_ry_rz, angle_between_two_lines, create_program

    model = ProcessModel()

    center,line = model.r_d("E:/2. GE/24. Kawasaki Robot\image1\img_20250614_181038.png")
    image = cv2.imread("E:/2. GE/24. Kawasaki Robot/image1/img_20250614_181038.png")
    
    points1 = generate_points_on_line(line[0][0], line[0][1], 70)
    dir_line1 = get_line_directions(line[0][0], line[0][1],center)
    
    points2 = generate_points_on_line(line[1][0], line[1][1], 70)
    dir_line2 = get_line_directions(line[1][0], line[1][1],center)
    
    rz_line1 = angle_between_two_lines(((0,0),(500,0)), (line[0][0], line[0][1]))
    rz_line2 = angle_between_two_lines(((0,0),(500,0)), (line[1][0], line[1][1]))
    
    line1_test = []
    for point in points1:
        line1_test.append({
            "point": (point[0],point[1]),
            "ux": dir_line1['ux'],
            "uy": dir_line1['uy'],
            "vx": dir_line1['vx'],
            "vy": dir_line1['vy'],
        })
    line2_test = []
    for point in points2:
        line2_test.append({
            "point": (point[0],point[1]), 
            "ux": dir_line2['ux'],
            "uy": dir_line2['uy'],
            "vx": dir_line2['vx'],
            "vy": dir_line2['vy'],
        })
    draw_points_with_xy_direction(image, line1_test, 30, (0, 0, 255), (0, 255, 0))
    draw_points_with_xy_direction(image, line2_test, 30, (0, 0, 255), (0, 255, 0))
    
    # line1 = []
    # for point in points1:
    #     line1.append(camera_to_robot(point[0],point[1]))
    
    # line2 = []
    # for point in points2:
    #     line2.append(camera_to_robot(point[0],point[1]))
    
    line1 = [camera_to_robot(line[0][0]), camera_to_robot(line[0][1])]
    line2 = [camera_to_robot(line[1][0]), camera_to_robot(line[1][1])]

        
    center_robot = camera_to_robot(center)
    create_program("test1806.pg","test1806", line1, line2, center_robot, rz_line1, rz_line2)
    
    cv2.imwrite("test.png", image)

if __name__ == "__main__":
    import time
    from src.utils.convert_point import angle_between_two_lines
    # main()
    # control_robot()
    # start_time = time.time()
    # test()
    # print("--- %s seconds ---" % (time.time() - start_time))
    print(angle_between_two_lines(((0,0),(0,500)), ((1865,1072),(1915,1357))))

# 210 pixel = 30mm -> 1 pixel = 0.14285714285714285 mm > 1mm = 7 pixel 

# image(1819, 1502)pixel -> image(mm) = (1819*0.14285714285714285, 1502*0.14285714285714285) = camera(259.857, 214.286)
# robot(-91.826, -381.399) 
# 
# 91.826 - 259.857 = -168.031
# 381.399 - 214.286 = 167.113

# TRANS(351.747, 595.724, 0, 0, 0, -90)
# z 322.327