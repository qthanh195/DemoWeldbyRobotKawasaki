from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO("model_v2.pt")

class ProcessModel:
        
    def predict(self, image):
        return model.predict(image, conf=0.6)
    
    def r_d(self, image):
        img1 = image.copy()
        # Luôn truyền ảnh màu vào YOLO
        if len(image.shape) == 2:  # ảnh grayscale
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 1:  # ảnh có shape (H, W, 1)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        results = self.predict(image)
        for idx, result in enumerate(results):
            if result.masks is None:
                continue
            
            for i, (seg, cls) in enumerate(zip(result.masks.xy, result.boxes.cls)):
                polygon = np.array(seg, dtype=np.int32)
                
                # 1. Tìm hình chữ nhật xoay bao quanh polygon
                rect = cv2.minAreaRect(polygon)
                box = cv2.boxPoints(rect)
                box = np.int32(box)
                
                center_part_image_org, size, angle_part_image_org = rect
                size = tuple([int(s) for s in size])
                
                M = cv2.getRotationMatrix2D(center_part_image_org, angle_part_image_org, 1.0)
                
                # 3. Xoay toàn ảnh
                rotated = cv2.warpAffine(img1, M, (img1.shape[1], img1.shape[0]))
                
                # 4. Crop vùng rectangle đã xoay
                x_crop, y_crop = int(center_part_image_org[0] - size[0] / 2), int(center_part_image_org[1] - size[1] / 2)
                w, h = size
                crop = rotated[y_crop:y_crop+h, x_crop:x_crop+w]
                if len(crop.shape) == 3:
                    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                else:
                    crop = crop
                                
                _, thresh = cv2.threshold(crop, 200, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # contour có diện tích lớn hơn 855000
                for cnt in contours:
                    if cv2.contourArea(cnt) > 855000:
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        box = np.int32(box)
                        cv2.drawContours(crop, [box], 0, (0, 255, 0), 2)
                        
                        #fine line 3,4
                        crop_thresh_min, center_thresh_min, angle_thresh_min, (x_crop_thresh_min, y_crop_thresh_min), size_crop_thresh = self.crop_image_with_rect(thresh, rect)
                        h_crop, w_crop = crop_thresh_min.shape
                        center_thresh_crop =  (int(w_crop/2), int(h_crop/2))
                        
                        (line3_a, line3_b), (line4_a, line4_b) = self.get_main_lines_from_crop(crop_thresh_min, center_thresh_crop, w_crop, h_crop)

                        # Chuyển về ảnh gốc
                        image_shape = thresh.shape  # hoặc image.shape
                        line3_a = self.revert_crop_point_to_image(line3_a, center_thresh_min, angle_thresh_min, (x_crop_thresh_min, y_crop_thresh_min), image_shape)
                        line3_b = self.revert_crop_point_to_image(line3_b, center_thresh_min, angle_thresh_min, (x_crop_thresh_min, y_crop_thresh_min), image_shape)
                        line4_a = self.revert_crop_point_to_image(line4_a, center_thresh_min, angle_thresh_min, (x_crop_thresh_min, y_crop_thresh_min), image_shape)
                        line4_b = self.revert_crop_point_to_image(line4_b, center_thresh_min, angle_thresh_min, (x_crop_thresh_min, y_crop_thresh_min), image_shape)

                        cv2.line(crop, (int(line3_a[0]), int(line3_a[1])), (int(line3_b[0]), int(line3_b[1])), 0, 2)
                        cv2.line(crop, (int(line4_a[0]), int(line4_a[1])), (int(line4_b[0]), int(line4_b[1])), 0, 2)
                        cv2.imwrite("debug_lines_on_crop.png", crop)

                        def sort_box_points_clockwise(box):
                            # Chuyển sang mảng numpy nếu cần
                            box = np.array(box)

                            # Tính tổng và hiệu toạ độ từng điểm
                            s = box.sum(axis=1)         # x + y
                            diff = np.diff(box, axis=1) # x - y

                            # Xác định các điểm theo thứ tự
                            p0 = box[np.argmin(s)]      # top-left: x + y nhỏ nhất
                            p2 = box[np.argmax(s)]      # bottom-right: x + y lớn nhất
                            p1 = box[np.argmin(diff)]   # top-right: x - y nhỏ nhất
                            p3 = box[np.argmax(diff)]   # bottom-left: x - y lớn nhất

                            return np.array([p0, p1, p2, p3])
                        
                        def rotate_point(point):
                            x, y = point
                            point_np = np.array([x+x_crop, y+y_crop, 1])
                            x_new, y_new = np.dot(cv2.getRotationMatrix2D(center_part_image_org, -angle_part_image_org, 1.0), point_np)
                            return (x_new, y_new)
                               
                        box = sort_box_points_clockwise(box)

                        if np.linalg.norm(box[0] - box[1]) > np.linalg.norm(box[0] - box[3]):
                            return (center_part_image_org,(([tuple(map(int,rotate_point(box[3]))), tuple(map(int,rotate_point(box[0])))]),
                                                           ([tuple(map(int,rotate_point(box[1]))), tuple(map(int,rotate_point(box[2])))])),
                                                            (([tuple(map(int,rotate_point(line3_a))), tuple(map(int,rotate_point(line3_b)))]),
                                                            (([tuple(map(int,rotate_point(line4_a))),tuple(map(int,rotate_point(line4_b)))]))))

                        elif np.linalg.norm(box[0] - box[1]) < np.linalg.norm(box[0] - box[3]):
                            return (center_part_image_org,(([tuple(map(int,rotate_point(box[0]))), tuple(map(int,rotate_point(box[1])))]),
                                                          ([tuple(map(int,rotate_point(box[2]))), tuple(map(int,rotate_point(box[3])))])), 
                                                            (([tuple(map(int,rotate_point(line3_a))), tuple(map(int,rotate_point(line3_b)))]),
                                                            ([tuple(map(int,rotate_point(line4_a))), tuple(map(int,rotate_point(line4_b)))])))  

        return None, None, None
    
    def crop_image_with_rect(self, image, rect):
        center, size, angle = rect
        print(center, size, angle)
        size = tuple([int(s) for s in size])
        size1 = size

        # Tính toán kích thước canvas mới (lớn hơn để không bị cắt)
        height, width = image.shape[:2]
        diag = int(np.sqrt(width**2 + height**2))
        new_w, new_h = diag, diag

        # Tạo ma trận dịch chuyển để đưa ảnh vào giữa canvas mới
        M_translate = np.array([[1, 0, (new_w - width) // 2],
                                [0, 1, (new_h - height) // 2]], dtype=np.float32)
        image_padded = cv2.warpAffine(image, M_translate, (new_w, new_h))

        # Điều chỉnh lại center cho canvas mới
        center_new = (center[0] + (new_w - width) // 2, center[1] + (new_h - height) // 2)

        # Xoay trên canvas mới
        M_rotate = cv2.getRotationMatrix2D(center_new, angle, 1.0)
        rotated = cv2.warpAffine(image_padded, M_rotate, (new_w, new_h))

        # Cắt vùng chữ nhật đã xoay
        x, y = int(center_new[0] - size[0] / 2), int(center_new[1] - size[1] / 2)
        w, h = size
        crop = rotated[y:y+h, x:x+w]

        return crop, center_new, angle, (x,y),size1
    
    def get_main_lines_from_crop(self, crop_thresh, center_crop, w_crop, h_crop):
        """
        crop_thresh: ảnh nhị phân sau threshold
        center_crop: tâm ảnh crop (tính theo crop local)
        w_crop, h_crop: kích thước ảnh crop
        Trả về: (line3_a, line3_b), (line4_a, line4_b) trong ảnh crop
        """
        if h_crop > w_crop:
            print("→ Hình đứng (dọc)")
            # Chia theo trục dọc (cắt ở giữa theo chiều ngang)
            zone_a = crop_thresh[:, :w_crop // 2]
            zone_b = crop_thresh[:, w_crop // 2:]
            offset_a, offset_b = -1, 1
            width_narrow, width_wide = 368, 441 # Tuỳ chỉnh dựa vào thực nghiệm
        else:
            print("→ Hình nằm ngang")
            # Chia theo trục ngang (cắt ở giữa theo chiều dọc)
            zone_a = crop_thresh[:h_crop // 2, :]
            zone_b = crop_thresh[h_crop // 2:, :]
            offset_a, offset_b = -1, 1
            # width_narrow, width_wide = 389, 462
            width_narrow, width_wide = 368, 441

        # Đếm số pixel trắng
        zone_a_count = cv2.countNonZero(zone_a)
        zone_b_count = cv2.countNonZero(zone_b)

        if h_crop > w_crop:
            if zone_a_count > zone_b_count:
                print("→ Bên trái nhiều hơn")
                line3_a = (center_crop[0] + offset_a * width_wide, center_crop[1] - 693)
                line3_b = (center_crop[0] + offset_a * width_wide, center_crop[1] + 693)
                line4_a = (center_crop[0] + offset_b * width_narrow, center_crop[1] - 693)
                line4_b = (center_crop[0] + offset_b * width_narrow, center_crop[1] + 693)
            else:
                print("→ Bên phải nhiều hơn")
                line3_a = (center_crop[0] + offset_a * width_narrow, center_crop[1] - 693)
                line3_b = (center_crop[0] + offset_a * width_narrow, center_crop[1] + 693)
                line4_a = (center_crop[0] + offset_b * width_wide, center_crop[1] - 693)
                line4_b = (center_crop[0] + offset_b * width_wide, center_crop[1] + 693)
        else:
            if zone_a_count > zone_b_count:
                print("→ Trên nhiều hơn")
                line3_a = (center_crop[0] - 629, center_crop[1] + offset_a * width_wide)
                line3_b = (center_crop[0] + 629, center_crop[1] + offset_a * width_wide)
                line4_a = (center_crop[0] + 629, center_crop[1] + offset_b * width_narrow)
                line4_b = (center_crop[0] - 629, center_crop[1] + offset_b * width_narrow)
            else:
                print("→ Dưới nhiều hơn")
                line3_a = (center_crop[0] + 629, center_crop[1] + offset_b * width_wide)
                line3_b = (center_crop[0] - 629, center_crop[1] + offset_b * width_wide)
                line4_a = (center_crop[0] - 629, center_crop[1] + offset_a * width_narrow)
                line4_b = (center_crop[0] + 629, center_crop[1] + offset_a * width_narrow)

        return (line3_a, line3_b), (line4_a, line4_b)
    
    def revert_crop_point_to_image(self, crop_point, center_rot, angle_deg, crop_origin, original_image_shape):
        """
        crop_point: (x, y) trong ảnh crop
        center_rot: tâm xoay trong ảnh đã pad
        angle_deg: góc xoay
        crop_origin: (x_crop, y_crop) – vị trí crop trong ảnh đã xoay
        original_image_shape: ảnh gốc (chưa pad), để tính lại dịch chuyển
        """
        x_crop, y_crop = crop_origin
        h_orig, w_orig = original_image_shape[:2]

        # Bước 1: Từ crop -> ảnh đã xoay
        point_in_rotated = (crop_point[0] + x_crop, crop_point[1] + y_crop)

        # Bước 2: Xoay ngược điểm đó
        M_inv = cv2.getRotationMatrix2D(center_rot, -angle_deg, 1.0)
        point_np = np.array([[point_in_rotated]], dtype=np.float32)
        point_in_padded = cv2.transform(point_np, M_inv)[0][0]

        # Bước 3: Tính lại dịch ngược từ pad về gốc
        diag = int(np.sqrt(w_orig**2 + h_orig**2))
        dx = (diag - w_orig) // 2
        dy = (diag - h_orig) // 2

        point_in_original = (point_in_padded[0] - dx, point_in_padded[1] - dy)

        return point_in_original


