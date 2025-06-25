from src.camera.camera_controller import BaslerCamera


def main():
    camera = BaslerCamera()
    camera.open_camera()

    
def control_robot():
    # %
    import time
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
    controller.load_program("test1806.pg")
    time.sleep(10)
    controller.disconnect_bot()
    
def test():
    z = -10
    from src.utils.process_model import ProcessModel
    import cv2
    import numpy as np
    from src.utils.convert_point import generate_points_on_line, pixel_to_robot, get_line_directions, draw_points_with_xy_direction, euler_xyz_to_oat, angle_between_two_lines, create_program, build_tcp_orientation_from_points
    from  src.utils.view import draw_robot_points
    from src.robot.robot_controller import RobotController
    model = ProcessModel()

    image = cv2.imread("E:/2. GE/24. Kawasaki Robot\code/robot_weld_vision_demo/test copy.png")
    center_part, linea, lineb = model.r_d(image)
    
    # points1_robot = [pixel_to_robot(p) for p in generate_points_on_line(linea[0][0], linea[0][1], 70)]
    # points2_robot = [pixel_to_robot(p) for p in generate_points_on_line(linea[1][0], linea[1][1], 70)]
    
    # line1_oat = build_tcp_orientation_from_points(np.array([linea[0][0][0],linea[0][0][1],z]), 
    #                                               np.array([linea[0][1][0],linea[0][1][1],z]), 
    #                                               np.array([center_part[0],center_part[1],z]), 20)
    # line2_oat = build_tcp_orientation_from_points(np.array([linea[1][0][0],linea[1][0][1],z]), 
    #                                               np.array([linea[1][1][0],linea[1][1][1],z]), 
    #                                               np.array([center_part[0],center_part[1],z]), 20)
    
    # points3_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[0][0], lineb[0][1], 70)]
    # points4_robot = [pixel_to_robot(p) for p in generate_points_on_line(lineb[1][0], lineb[1][1], 70)]
    
    # line3_oat = build_tcp_orientation_from_points(np.array([lineb[0][0][0],lineb[0][0][1],z]),
    #                                               np.array([lineb[1][0][0],lineb[1][0][1],z]),
    #                                               np.array([center_part[0],center_part[1],z]), -20)
    # line4_oat = build_tcp_orientation_from_points(np.array([lineb[1][0][0],lineb[1][0][1],z]),
                                                #   np.array([lineb[1][1][0],lineb[1][1][1],z]),
                                                #   np.array([center_part[0],center_part[1],z]), -20)
    
    # line1_test = []
    # for point in points1:
    #     line1_test.append({
    #         "point": (point[0],point[1]),
    #         "ux": dir_line1['ux'],
    #         "uy": dir_line1['uy'],
    #         "vx": dir_line1['vx'],
    #         "vy": dir_line1['vy'],
    #     })
    # line2_test = []
    # for point in points2:
    #     line2_test.append({
    #         "point": (point[0],point[1]), 
    #         "ux": dir_line2['ux'],
    #         "uy": dir_line2['uy'],
    #         "vx": dir_line2['vx'],
    #         "vy": dir_line2['vy'],
    #     })
    # draw_points_with_xy_direction(image, line1_test, 30, (0, 0, 255), (0, 255, 0))
    # draw_points_with_xy_direction(image, line2_test, 30, (0, 0, 255), (0, 255, 0))
    

    
    # line1 = [pixel_to_robot(line[0][0]), pixel_to_robot(line[0][1])]
    # line2 = [pixel_to_robot(line[1][0]), pixel_to_robot(line[1][1])]
    

    # center_robot = pixel_to_robot(center)
    
    # robot = RobotController()
    # robot.initialize_robot()
    # robot.connect_bot()
    # robot.load_program(line1, line2, center_robot, line1_oat, line2_oat)
    # robot.disconnect_bot()
    
    #vec cac duong line
    cv2.line(image, (linea[0][0][0],linea[0][0][1]), (linea[0][1][0],linea[0][1][1]), (0, 255, 0), 2)
    cv2.line(image, (linea[1][0][0],linea[1][0][1]), (linea[1][1][0],linea[1][1][1]), (0, 255, 0), 2)
    
    cv2.line(image, (lineb[1][0][0],lineb[1][0][1]), (lineb[1][1][0],lineb[1][1][1]), (0, 255, 0), 2)
    cv2.line(image, (lineb[0][0][0],lineb[0][0][1]), (lineb[0][1][0],lineb[0][1][1]), (0, 255, 0), 2)
    cv2.circle(image, (int(center_part[0]), int(center_part[1])), 10, (0, 0, 255), 2)
    cv2.line(image, (0, 1374), (3840, 1374), (0, 255, 0), 2)
    cv2.line(image, (1920, 0), (1920, 2748), (0, 255, 0), 2)
    cv2.imwrite("test.png", image)

if __name__ == "__main__":
    # from src.utils.process_model import ProcessModel
    # import cv2
    # model = ProcessModel()
    # center,line = model.r_d(cv2.imread("E:/2. GE/24. Kawasaki Robot/image1/img_20250614_180955.png"))
    test()

#     robot = RobotController()
#     robot.initialize_robot()
#     robot.connect_bot()
    
#     robot.load_program("gerobot")
#     # time.sleep(10)  
#     robot.disconnect_bot()
    
    

# 210 pixel = 30mm -> 1 pixel = 0.14285714285714285 mm > 1mm = 7 pixel 

# image(1819, 1502)pixel -> image(mm) = (1819*0.14285714285714285, 1502*0.14285714285714285) = camera(259.857, 214.286)
# robot(-91.826, -381.399) 
# 
# 91.826 - 259.857 = -168.031
# 381.399 - 214.286 = 167.113

# TRANS(351.747, 595.724, 0, 0, 0, -90)
# z 322.327

# 1450 pixel = 207mm -> 1 pixel = 0.14285714285714285 mm > 1mm = 7 pixel