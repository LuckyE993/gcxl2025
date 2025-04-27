import cv2
import numpy as np
import time

class CameraCalibrator:
    def __init__(self, pattern_size=(9,6), square_size=30, min_samples=15):
        # 标定参数
        self.pattern_size = pattern_size    # 棋盘格内部角点数 (cols, rows)
        self.square_size = square_size      # 棋盘格实际边长（毫米）
        self.min_samples = min_samples      # 最小有效样本数
        
        # 标定数据存储
        self.obj_points = []                # 3D物体点
        self.img_points = []                # 2D图像点
        self.calibration_done = False       # 标定完成标志
        
        # 生成物体坐标系点（Z=0）
        self.objp = np.zeros((self.pattern_size[0]*self.pattern_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1,2) * square_size
        
        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.height = None
        self.pixel_ratio = None

    def detect_corners(self, frame):
        """检测棋盘格角点"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray", gray)
        
        # 使用更宽松的参数检测棋盘格
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        
        if ret:
            # 亚像素级角点检测
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            return True, corners
        return False, None

    def collect_samples(self, cap):
        """通过摄像头采集样本"""
        print(f"需要至少 {self.min_samples} 组有效数据，按空格键采集，ESC退出")
        sample_count = 0
        
        while sample_count < self.min_samples:
            ret, frame = cap.read()
            if not ret:
                continue
                
            # 实时显示检测结果
            draw_frame = frame.copy()
            success, corners = self.detect_corners(draw_frame)
            if success:
                cv2.drawChessboardCorners(draw_frame, self.pattern_size, corners, success)
                cv2.putText(draw_frame, f"Found: {sample_count}/{self.min_samples}", 
                           (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            else:
                cv2.putText(draw_frame, "Adjust chessboard", (20,40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            
            cv2.imshow("Calibration", draw_frame)
            
            key = cv2.waitKey(1)
            if key == 32 and success:  # 空格键
                self.img_points.append(corners)
                self.obj_points.append(self.objp)
                sample_count += 1
                print(f"采集样本 {sample_count}/{self.min_samples}")
                time.sleep(0.5)  # 防止连续采集
            elif key == 27:       # ESC键
                break
        
        cv2.destroyAllWindows()
        return sample_count >= self.min_samples

    def perform_calibration(self, cap=None, img_size=None):
        """执行标定计算"""
        if len(self.obj_points) < self.min_samples:
            print("样本不足，标定失败")
            return False
        
        # 获取图像尺寸
        if img_size is None:
            if cap is not None and cap.isOpened():
                # 使用传入的摄像头对象
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("无法从摄像头获取图像")
                    return False
                h, w = frame.shape[:2]
            else:
                # 如果没有有效的图像源，使用已获取样本中的图像点估算尺寸
                if len(self.img_points) > 0:
                    points = self.img_points[0]
                    x_coords = points[:, 0, 0]
                    y_coords = points[:, 0, 1]
                    w = int(np.max(x_coords) * 1.2)  # 估计宽度
                    h = int(np.max(y_coords) * 1.2)  # 估计高度
                    print(f"使用估计图像尺寸: {w}x{h}")
                else:
                    # 使用默认尺寸
                    w, h = 640, 480
                    print(f"使用默认图像尺寸: {w}x{h}")
        else:
            # 使用传入的图像尺寸
            w, h = img_size
        
        # 执行标定
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, (w, h), None, None
        )
        
        if not ret:
            print("标定失败")
            return False
        
        # 计算重投影误差以评估标定质量
        mean_error = 0
        for i in range(len(self.obj_points)):
            imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], 
                                              self.camera_matrix, self.dist_coeffs)
            error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print(f"平均重投影误差: {mean_error/len(self.obj_points)}")
        
        # 估算摄像头高度 (假设棋盘平放且Z=0平面)
        # 使用最后一个样本的tvec[2]（Z方向距离）获取高度
        self.height = np.mean([tvec[2] for tvec in tvecs]) / 1000  # 转换为米
        
        # 计算像素比例（毫米/像素）- 使用焦距和高度关系
        fx = self.camera_matrix[0,0]
        fy = self.camera_matrix[1,1]
        # 基于焦距和高度的像素实际尺寸换算
        self.pixel_ratio = self.square_size / (fx * self.square_size / (self.height * 1000))
        
        self.calibration_done = True
        return True

    def print_results(self):
        """打印标定结果"""
        if not self.calibration_done:
            print("尚未完成标定")
            return
        
        print("\n=== 标定结果 ===")
        print(f"摄像头高度: {self.height:.3f} 米")
        print(f"像素比例: {self.pixel_ratio:.4f} 毫米/像素")
        print("内参矩阵:\n", self.camera_matrix)
        print("畸变系数:\n", self.dist_coeffs.ravel())
        print("焦距: fx={:.2f}, fy={:.2f}".format(
            self.camera_matrix[0,0], self.camera_matrix[1,1]))
        print("主点: cx={:.2f}, cy={:.2f}".format(
            self.camera_matrix[0,2], self.camera_matrix[1,2]))

if __name__ == "__main__":
    # 初始化标定器和摄像头
    calibrator = CameraCalibrator(square_size=20, min_samples=10, pattern_size=(5,4))
    
    # 尝试打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头，尝试其他摄像头索引...")
        # 尝试其他摄像头索引
        for i in range(1, 5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"成功打开摄像头索引 {i}")
                break
    
    if not cap.isOpened():
        print("无法打开任何摄像头，程序退出")
        exit(1)
        
    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 采集数据
    if calibrator.collect_samples(cap):
        # 执行标定，传入摄像头对象
        if calibrator.perform_calibration(cap=cap):
            calibrator.print_results()
        else:
            print("标定过程出现错误")
    else:
        print("数据采集中断")

    cap.release()