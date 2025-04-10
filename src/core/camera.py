import cv2
import numpy as np
import time
from typing import Optional, Tuple, Dict, List, Union, Any
import sys
from pathlib import Path

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(project_root))

from src.utils.helpers import log_message
from src.config.settings import get_config

class Camera:
    """摄像头操作的封装类，提供摄像头读取和配置的功能"""
    
    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (640, 480)):
        """
        初始化摄像头对象
        
        Args:
            camera_id (int): 摄像头ID，默认为0（通常是内置摄像头）
            resolution (Tuple[int, int]): 分辨率，格式为(宽, 高)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.cap = None
        self.is_open = False
    
    def open(self) -> bool:
        """
        打开摄像头
        
        Returns:
            bool: 打开成功返回True，否则返回False
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                log_message(f"无法打开摄像头ID: {self.camera_id}", level="error")
                return False
                
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            self.is_open = True
            log_message(f"成功打开摄像头ID: {self.camera_id}, 分辨率: {self.resolution}")
            return True
        except Exception as e:
            log_message(f"打开摄像头异常: {str(e)}", level="error")
            return False
    
    def close(self) -> None:
        """
        关闭摄像头
        """
        if self.cap and self.is_open:
            self.cap.release()
            self.is_open = False
            log_message(f"关闭摄像头ID: {self.camera_id}")
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        读取一帧图像
        
        Returns:
            Optional[np.ndarray]: 成功返回图像帧(BGR格式)，失败返回None
        """
        if not self.is_open or not self.cap:
            log_message("摄像头未打开", level="warning")
            return None
            
        ret, frame = self.cap.read()
        if not ret:
            log_message("读取帧失败", level="warning")
            return None
            
        return frame
    
    def get_properties(self) -> Dict[str, float]:
        """
        获取摄像头属性
        
        Returns:
            Dict[str, float]: 摄像头属性字典，包括分辨率、帧率等
        """
        if not self.is_open or not self.cap:
            log_message("摄像头未打开", level="warning")
            return {}
            
        properties = {
            "width": self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
            "height": self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            "fps": self.cap.get(cv2.CAP_PROP_FPS),
            "brightness": self.cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": self.cap.get(cv2.CAP_PROP_CONTRAST),
            "saturation": self.cap.get(cv2.CAP_PROP_SATURATION),
            "hue": self.cap.get(cv2.CAP_PROP_HUE),
            "exposure": self.cap.get(cv2.CAP_PROP_EXPOSURE)
        }
        return properties
    
    def set_property(self, property_id: int, value: float) -> bool:
        """
        设置摄像头属性
        
        Args:
            property_id (int): 属性ID，使用cv2.CAP_PROP_*常量
            value (float): 属性值
            
        Returns:
            bool: 设置成功返回True，否则返回False
        """
        if not self.is_open or not self.cap:
            log_message("摄像头未打开", level="warning")
            return False
            
        return self.cap.set(property_id, value)



class FPSCounter:
    """用于计算和显示FPS的工具类"""
    
    def __init__(self, update_interval=10):
        """
        初始化FPS计数器
        
        Args:
            update_interval (int): 更新FPS的帧间隔
        """
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        self.update_interval = update_interval
    
    def update(self) -> float:
        """
        更新FPS计数
        
        Returns:
            float: 当前FPS值
        """
        self.frame_count += 1
        
        # 达到更新间隔后计算FPS
        if self.frame_count >= self.update_interval:
            end_time = time.time()
            time_elapsed = end_time - self.start_time
            
            # 避免除零错误
            if time_elapsed > 0:
                self.fps = self.frame_count / time_elapsed
            
            # 重置计数
            self.frame_count = 0
            self.start_time = time.time()
            
        return self.fps
    
    def draw_fps(self, image, position=(10, 30), font_scale=1, color=(0, 255, 0), thickness=2):
        """
        在图像上绘制FPS信息
        
        Args:
            image: 要绘制的图像
            position (tuple): 文本位置 (x, y)
            font_scale (float): 字体大小
            color (tuple): 文本颜色 (B, G, R)
            thickness (int): 文本粗细
            
        Returns:
            图像: 带有FPS信息的图像
        """
        cv2.putText(
            image,
            f"FPS: {self.fps:.1f}",
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            color,
            thickness
        )
        return image

def list_available_cameras() -> List[int]:
    """
    列出系统中可用的摄像头
    
    Returns:
        List[int]: 可用摄像头ID列表
    """
    available_cameras = []
    for i in range(10):  # 通常检查前10个ID应该足够
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    
    return available_cameras


def get_camera_from_config() -> Optional[Camera]:
    """
    从配置文件中读取摄像头设置并返回摄像头对象
    
    Returns:
        Optional[Camera]: 配置好的摄像头对象，配置失败返回None
    """
    try:
        config = get_config()
        camera_config = config.get("camera", {})
        camera_id = camera_config.get("id", 0)
        width = camera_config.get("width", 640)
        height = camera_config.get("height", 480)
        
        camera = Camera(camera_id=camera_id, resolution=(width, height))
        if camera.open():
            return camera
        return None
    except Exception as e:
        log_message(f"从配置文件读取摄像头设置失败: {str(e)}", level="error")
        return None


def capture_image(camera: Camera) -> Optional[np.ndarray]:
    """
    使用给定的摄像头捕获一张图像
    
    Args:
        camera (Camera): 摄像头对象
    
    Returns:
        Optional[np.ndarray]: 捕获的图像(BGR格式)，失败返回None
    """
    return camera.read_frame()


def save_image(image: np.ndarray, file_path: str) -> bool:
    """
    保存图像到文件
    
    Args:
        image (np.ndarray): 图像数据(BGR格式)
        file_path (str): 保存路径
        
    Returns:
        bool: 保存成功返回True，否则返回False
    """
    try:
        cv2.imwrite(file_path, image)
        log_message(f"图像已保存到: {file_path}")
        return True
    except Exception as e:
        log_message(f"保存图像失败: {str(e)}", level="error")
        return False


def convert_bgr_to_rgb(image: np.ndarray) -> np.ndarray:
    """
    将BGR格式图像转换为RGB格式
    
    Args:
        image (np.ndarray): BGR格式图像
        
    Returns:
        np.ndarray: RGB格式图像
    """
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


# 测试代码
if __name__ == "__main__":
    # 列出可用摄像头
    available_cameras = list_available_cameras()
    print(f"可用摄像头: {available_cameras}")
    
    # 创建并测试摄像头
    camera = Camera(camera_id=0, resolution=(640, 480))
    if camera.open():
        print("摄像头属性:", camera.get_properties())
        
        # 读取并显示3秒视频流
        start_time = time.time()
        while True:
            frame = camera.read_frame()
            if frame is not None:
                cv2.imshow("Camera Test", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        # 捕获一张图像并保存
        image = capture_image(camera)
        if image is not None:
            save_image(image, "test_capture.jpg")
        
        # 关闭摄像头
        camera.close()
        cv2.destroyAllWindows()