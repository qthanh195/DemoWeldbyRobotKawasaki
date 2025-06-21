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
    from src.utils.convert_point import generate_points_on_line, pixel_to_robot, get_line_directions, draw_points_with_xy_direction, euler_xyz_to_oat, angle_between_two_lines, create_program
    from  src.utils.view import draw_robot_points
    model = ProcessModel()

    center,line = model.r_d("E:/2. GE/24. Kawasaki Robot\image1\img_20250614_181118.png")
    image = cv2.imread("E:/2. GE/24. Kawasaki Robot/image1/img_20250614_181118.png")
    
    points1 = generate_points_on_line(line[0][0], line[0][1], 70)
    dir_line1 = get_line_directions(line[0][0], line[0][1],center)
    
    points2 = generate_points_on_line(line[1][0], line[1][1], 70)
    dir_line2 = get_line_directions(line[1][0], line[1][1],center)
    
    rz_line1 = angle_between_two_lines(((0,0),(500,0)), (line[0][0], line[0][1]))
    rz_line2 = angle_between_two_lines(((0,0),(500,0)), (line[1][0], line[1][1]))
    line1_oat = euler_xyz_to_oat(0, 0, rz_line1)
    line2_oat = euler_xyz_to_oat(0, 0, rz_line2)
    
    
    
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
    
    line1 = [pixel_to_robot(line[0][0]), pixel_to_robot(line[0][1])]
    line2 = [pixel_to_robot(line[1][0]), pixel_to_robot(line[1][1])]
    

        
    center_robot = pixel_to_robot(center)
    
    # print("line1: ", line1)
    # print("line2:", line2)
    # print("center_robot: ", center_robot)
    
    
    # poins3d = [(line1[0][0],line1[0][1],10), (line1[1][0],line1[1][1],10), (line2[0][0],line2[0][1],10), (line2[1][0],line2[1][1],10), (center_robot[0],center_robot[1],300)]
    # draw_robot_points(poins3d, "3d", True)
    create_program("test1806.pg","test1806", line1, line2, center_robot, line1_oat, line)
    

    cv2.line(image, (0, 1374), (3840, 1374), (0, 255, 0), 2)
    cv2.line(image, (1920, 0), (1920, 2748), (0, 255, 0), 2)
    cv2.imwrite("test.png", image)

if __name__ == "__main__":
    # from src.robot.robot_controller import calibrate_kawasaki_base_from_3_points, euler_xyz_to_oat
    # user_coordinates = calibrate_kawasaki_base_from_3_points((434.862,151696,-285.698),(436.323,37.110,-285.700),(367.975,37.116,-286.438))
    # print(user_coordinates)
    
    from src.utils.convert_point import euler_xyz_to_oat
    print(euler_xyz_to_oat(-30, 0, 82.409))
    
    # main()
    # control_robot()
    # import time
    # start_time = time.time()
    # test()
    # print("--- %s seconds ---" % (time.time() - start_time))

# 210 pixel = 30mm -> 1 pixel = 0.14285714285714285 mm > 1mm = 7 pixel 

# image(1819, 1502)pixel -> image(mm) = (1819*0.14285714285714285, 1502*0.14285714285714285) = camera(259.857, 214.286)
# robot(-91.826, -381.399) 
# 
# 91.826 - 259.857 = -168.031
# 381.399 - 214.286 = 167.113

# TRANS(351.747, 595.724, 0, 0, 0, -90)
# z 322.327

# 1450 pixel = 207mm -> 1 pixel = 0.14285714285714285 mm > 1mm = 7 pixel