import time
from camera.camera_controller import CameraController
from robot.robot_controller import RobotController

def main():
    robot = RobotController()

    robot.connect_bot()
    


if __name__ == "__main__":
    main()