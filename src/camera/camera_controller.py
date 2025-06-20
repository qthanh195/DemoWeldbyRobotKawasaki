from pypylon import pylon
import numpy as np
import queue

        
class BaslerCamera():
    def __init__(self):
        self.camera = None
        self.is_open = False
        self.image = queue.Queue()
        
    def get_image(self):
        return self.single_shot()

    def setup_camera(self):
        """Thiết lập cấu hình camera."""
        
        self.camera.PixelFormat.Value = "Mono8" # Đặt định dạng pixel thành Mono8
        self.camera.ExposureAuto.Value = "Off" ## Đặt chế độ tự động điều chỉnh độ sáng thành Once
        self.camera.BalanceWhiteAuto.Value = "Off" # Đặt chế độ tự động điều chỉnh màu trắng thành Once
        # self.print_camera_settings()
        
    def open_camera(self):
        """Mở camera và tải user set nếu có."""
        try:
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.Open()
            self.is_open = True
            print("Camera đã mở.")
            # self.setup_camera()
        except Exception as e:
            print(f"Lỗi khi mở camera: {e}")

    def close_camera(self):
        """Đóng camera Basler."""
        if self.is_open:
            # if self.is_continuous:
            #     self.stop_continuous_grabbing()
            self.camera.Close()
            self.is_open = False
            print("Camera đã đóng.")

    def single_shot(self):
        """Chụp một hình ảnh."""
        image = None
        if not self.is_open:
            print("Camera chưa được mở.")
            return image
        try:
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            grab_result = self.camera.RetrieveResult(20000, pylon.TimeoutHandling_ThrowException)

            if grab_result.GrabSucceeded():
                # Chuyển đổi hình ảnh sang định dạng OpenCV
                image = grab_result.Array
                grab_result.Release()
                self.camera.StopGrabbing()
                return image
            else:
                print("Lỗi khi chụp hình.")
        except Exception as e:
            print(f"Lỗi khi chụp hình: {e}")

    def continuous_grab(self):
        """Chụp liên tục hình ảnh."""
        if not self.is_open:
            print("Camera chưa được mở.")
            return
        try:
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            while self.is_open:
                grab_result = self.camera.RetrieveResult(20000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    # Chuyển đổi hình ảnh sang định dạng OpenCV
                    image = grab_result.Array
                    grab_result.Release()
                    
                    self.image.put(image)
                else:
                    print("Lỗi khi chụp hình.")
        except Exception as e:
            print(f"Lỗi khi chụp hình: {e}")

    def stop_continuous_grabbing(self):
        """Dừng chụp liên tục hình ảnh."""
        if self.is_open:
            self.camera.StopGrabbing()
            print("Chụp liên tục đã dừng.")