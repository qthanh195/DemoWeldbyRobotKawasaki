import unittest
from src.camera.camera_controller import CameraController

class TestCameraController(unittest.TestCase):

    def setUp(self):
        self.camera_controller = CameraController()

    def test_initialize_camera(self):
        result = self.camera_controller.initialize_camera()
        self.assertTrue(result)

    def test_capture_image(self):
        self.camera_controller.initialize_camera()
        image = self.camera_controller.capture_image()
        self.assertIsNotNone(image)

    def test_release_camera(self):
        self.camera_controller.initialize_camera()
        result = self.camera_controller.release_camera()
        self.assertTrue(result)

if __name__ == '__main__':
    unittest.main()