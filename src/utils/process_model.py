from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO("model_v2.pt")

class ProcessModel:
        
    def predict(self, image):
        return model.predict(image, conf=0.6)
    
    def r_d(self, image):
        img1 = cv2.imread(image)
        img = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
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
                rotated = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
                
                # 4. Crop vùng rectangle đã xoay
                x_crop, y_crop = int(center_part_image_org[0] - size[0] / 2), int(center_part_image_org[1] - size[1] / 2)
                w, h = size
                crop = rotated[y_crop:y_crop+h, x_crop:x_crop+w]
                
                _, thresh = cv2.threshold(crop, 200, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # contour có diện tích lớn hơn 855000
                for cnt in contours:
                    if cv2.contourArea(cnt) > 855000:
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        box = np.int32(box)

                        # print(w, h)
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
                            return center_part_image_org,(([tuple(map(int,rotate_point(box[3]))), tuple(map(int,rotate_point(box[0])))]),([tuple(map(int,rotate_point(box[1]))), tuple(map(int,rotate_point(box[2])))]))
                        
                        elif np.linalg.norm(box[0] - box[1]) < np.linalg.norm(box[0] - box[3]):
                            return center_part_image_org,(([tuple(map(int,rotate_point(box[0]))), tuple(map(int,rotate_point(box[1])))]),([tuple(map(int,rotate_point(box[2]))), tuple(map(int,rotate_point(box[3])))]))

        return None, None
                
                