import unittest
from src.robot.robot_controller import RobotController

class TestRobotController(unittest.TestCase):

    def setUp(self):
        self.robot_controller = RobotController()

    def test_initialize_robot(self):
        result = self.robot_controller.initialize_robot()
        self.assertTrue(result)

    def test_move_to_position(self):
        position = (10, 20, 30)
        result = self.robot_controller.move_to_position(position)
        self.assertTrue(result)

    def test_start_welding(self):
        result = self.robot_controller.start_welding()
        self.assertTrue(result)

if __name__ == '__main__':
    unittest.main()