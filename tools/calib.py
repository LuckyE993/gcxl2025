import cv2
import numpy as np
import time
import yaml
import os

class CameraCalibrator:
    def __init__(self, pattern_size=(9,6), square_size=30, min_samples=15, save_images=True):
        # 标定参数
        self.pattern_size = pattern_size    # 棋盘格内部角点数 (cols, rows)
        self.square_size = square_size      # 棋盘格实际边长（毫米）
        self.min_samples = min_samples      # 最小有效样本数
        self.save_images = save_images      # 是否保存标定图像
        
        # 标定数据存储
        self.obj_points = []                # 3D物体点
        self.img_points = []                # 2D图像点
        self.calibration_done = False       # 标定完成标志
        self.calibration_images = []        # 保存标定用的图像
        
        # 创建保存图像的目录
        self.images_dir = "calibration_images"
        if self.save_images and not os.path.exists(self.images_dir):
            os.makedirs(self.images_dir)
            print(f"创建图像保存目录: {self.images_dir}")
        
        # 生成物体坐标系点（Z=0）
        self.objp = np.zeros((self.pattern_size[0]*self.pattern_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1,2) * square_size
        
        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.height = None
        self.pixel_ratio = None
        self.img_size = None

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
                
                # 保存标定图像
                if self.save_images:
                    img_filename = f"{self.images_dir}/calib_{sample_count:02d}.jpg"
                    cv2.imwrite(img_filename, frame)
                    print(f"保存标定图像: {img_filename}")
                    self.calibration_images.append(frame.copy())
                
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
            
        self.img_size = (w, h)
        
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
    
    def save_calibration_to_yaml(self, filename='camera_calibration.yaml'):
        """将标定结果保存到YAML文件"""
        if not self.calibration_done:
            print("尚未完成标定，无法保存结果")
            return False
            
        # 创建要保存的数据字典
        data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'image_width': self.img_size[0],
            'image_height': self.img_size[1],
            'height': float(self.height),
            'pixel_ratio': float(self.pixel_ratio),
            'calibration_time': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # 保存到YAML文件
        try:
            with open(filename, 'w') as f:
                yaml.dump(data, f)
            print(f"标定结果已保存到 {filename}")
            return True
        except Exception as e:
            print(f"保存标定结果失败: {e}")
            return False


def load_calibration_from_yaml(filename='camera_calibration.yaml'):
    """从YAML文件加载标定结果"""
    if not os.path.exists(filename):
        print(f"标定文件 {filename} 不存在")
        return None
        
    try:
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
            
        # 转换数据类型
        camera_matrix = np.array(data['camera_matrix'])
        dist_coeffs = np.array(data['dist_coeffs'])
        img_width = data['image_width']
        img_height = data['image_height']
        
        return {
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'img_size': (img_width, img_height),
            'height': data.get('height'),
            'pixel_ratio': data.get('pixel_ratio')
        }
    except Exception as e:
        print(f"加载标定结果失败: {e}")
        return None


def undistort_and_display(calibration_file='camera_calibration.yaml', camera_id=0, apply_roi_crop=False):
    """从标定文件读取参数进行实时畸变矫正显示"""
    # 加载标定数据
    calib_data = load_calibration_from_yaml(calibration_file)
    if calib_data is None:
        return False
        
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']
    img_size = calib_data['img_size']
    
    # 创建对比图像保存目录
    comparison_dir = "correction_comparison"
    if not os.path.exists(comparison_dir):
        os.makedirs(comparison_dir)
        print(f"创建对比图像保存目录: {comparison_dir}")
    
    # 计算最佳新相机矩阵
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, img_size, 0, img_size)
    
    # 计算畸变映射
    mapx, mapy = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, img_size, cv2.CV_32FC1)
    
    # 打开摄像头
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("无法打开摄像头")
        return False
    
    # 设置分辨率与标定一致
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_size[1])
    
    print("正在显示畸变矫正后的图像")
    print("按键说明:")
    print("  ESC - 退出")
    print("  s   - 保存当前原始图像")
    print("  c   - 保存当前矫正图像")
    print("  b   - 保存原始和矫正图像对比")
    
    save_counter = 0
    
    # 显示畸变矫正后的图像
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取图像")
            break
            
        # 使用畸变映射进行矫正
        undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        
        # 裁剪ROI区域 (可选)
        x, y, w, h = roi
        if apply_roi_crop and w > 0 and h > 0:
            if w > 50 and h > 50:  # 确保ROI合理
                undistorted = undistorted[y:y+h, x:x+w]
            else:
                print(f"跳过ROI裁剪，ROI太小: {roi}")
        
        # 显示原始和矫正后的图像
        cv2.imshow("Original", frame)
        cv2.imshow("Undistorted", undistorted)
        
        # 检查按键
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键
            break
        elif key == ord('s'):  # 's'键 - 保存原始图像
            original_filename = f"{comparison_dir}/original_{save_counter:03d}.jpg"
            cv2.imwrite(original_filename, frame)
            print(f"保存原始图像: {original_filename}")
            save_counter += 1
        elif key == ord('c'):  # 'c'键 - 保存矫正图像
            corrected_filename = f"{comparison_dir}/corrected_{save_counter:03d}.jpg"
            cv2.imwrite(corrected_filename, undistorted)
            print(f"保存矫正图像: {corrected_filename}")
            save_counter += 1
        elif key == ord('b'):  # 'b'键 - 保存对比图像
            # 创建对比图像
            if undistorted.shape[:2] != frame.shape[:2]:
                # 如果尺寸不同，调整矫正图像尺寸
                undistorted_resized = cv2.resize(undistorted, (frame.shape[1], frame.shape[0]))
            else:
                undistorted_resized = undistorted
            
            # 水平拼接原始和矫正图像
            comparison = np.hstack((frame, undistorted_resized))
            
            # 添加标签
            cv2.putText(comparison, "Original", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(comparison, "Corrected", (frame.shape[1] + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            comparison_filename = f"{comparison_dir}/comparison_{save_counter:03d}.jpg"
            cv2.imwrite(comparison_filename, comparison)
            print(f"保存对比图像: {comparison_filename}")
            save_counter += 1
    
    cap.release()
    cv2.destroyAllWindows()
    return True

if __name__ == "__main__":
    # 定义参数
    calibration_file = 'camera_calibration.yaml'
    camera_id = 2
    
    # 检查是否要加载现有标定文件
    if os.path.exists(calibration_file):
        print(f"检测到标定文件 {calibration_file}")
        choice = input("选择操作: [r]重新标定 或 [u]使用现有标定显示矫正结果? ").lower()
        if choice == 'u':
            undistort_and_display(calibration_file, camera_id)
            exit(0)
    
    # 初始化标定器和摄像头 - 启用图像保存
    calibrator = CameraCalibrator(square_size=20, min_samples=10, pattern_size=(5,4), save_images=True)
    
    # 尝试打开摄像头
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("无法打开摄像头，尝试其他摄像头索引...")
        # 尝试其他摄像头索引
        for i in range(1, 5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"成功打开摄像头索引 {i}")
                camera_id = i
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
            
            # 保存标定结果到YAML文件
            if calibrator.save_calibration_to_yaml(calibration_file):
                # 关闭摄像头
                cap.release()
                
                # 询问是否立即显示矫正效果
                choice = input("是否立即查看矫正效果? [y/n]: ").lower()
                if choice == 'y':
                    undistort_and_display(calibration_file, camera_id)
        else:
            print("标定过程出现错误")
    else:
        print("数据采集中断")
    
    cap.release()