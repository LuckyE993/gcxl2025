import sys
import os
from pathlib import Path
from typing import List, Dict, Optional, Union, Any

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(project_root))

from ultralytics import YOLO
import numpy as np
from src.config.settings import get_config
from src.utils.helpers import log_message


class Detector:
    """对象检测功能的封装类"""
    
    def __init__(self, model_path: Optional[str] = None):
        """
        初始化检测器
        
        Args:
            model_path (Optional[str]): 模型路径，如果为None则从配置文件读取
        """
        log_message("Detector类初始化")
        self.model = None
        if model_path:
            self.load_model(model_path)
    
    def load_model(self, custom_model_path: Optional[str] = None) -> bool:
        """
        加载YOLO模型
        
        Args:
            custom_model_path (Optional[str]): 自定义模型路径，如果提供则覆盖配置文件中的路径
            
        Returns:
            bool: 加载成功返回True，否则返回False
        """
        try:
            # 获取配置
            config = get_config()
            
            # 确定模型路径（优先使用自定义路径，其次是配置文件中的路径）
            if custom_model_path:
                model_path = custom_model_path
            else:
                model_path = config.get("model", {}).get("path", "models/default.pt")
            
            # 确保路径是相对于项目根目录的
            absolute_model_path = project_root / model_path
            
            log_message(f"加载模型: {absolute_model_path}")
            
            # 加载模型
            self.model = YOLO(str(absolute_model_path))
            return True
        except Exception as e:
            log_message(f"模型加载失败: {str(e)}", level="error")
            return False
    
    def detect(self, image: np.ndarray, confidence: float = 0.25) -> List[Dict[str, Any]]:
        """
        使用模型检测图像中的物体
        
        Args:
            image (np.ndarray): 输入图像
            confidence (float): 置信度阈值，默认为0.25
        
        Returns:
            List[Dict[str, Any]]: 检测到的物体列表，每个物体包含类别、置信度和边界框信息
        """
        if self.model is None:
            log_message("模型未加载，无法执行检测", level="error")
            return []
        
        try:
            # 使用模型检测物体
            results = self.model.predict(image, conf=confidence)[0]
            
            # 解析结果
            detections = []
            
            # 获取边界框、置信度和类别ID
            boxes = results.boxes.xyxy.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy().astype(int)
            
            # 获取类别名称
            class_names = results.names
            
            # 组织检测结果
            for i, box in enumerate(boxes):
                detection = {
                    "class_id": int(class_ids[i]),
                    "class_name": class_names[int(class_ids[i])],
                    "confidence": float(confidences[i]),
                    "bbox": {
                        "x1": float(box[0]),
                        "y1": float(box[1]),
                        "x2": float(box[2]),
                        "y2": float(box[3])
                    }
                }
                detections.append(detection)
            
            return detections
        except Exception as e:
            log_message(f"检测过程发生错误: {str(e)}", level="error")
            return []
    
    def visualize_detections(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """
        在图像上可视化检测结果，使用固定的颜色方案
        
        Args:
            image (np.ndarray): 原始图像
            detections (List[Dict[str, Any]]): 检测结果列表
            
        Returns:
            np.ndarray: 带有检测结果标注的图像
        """
        import cv2
        
        # 创建图像副本以避免修改原始图像
        output_image = image.copy()
        
        # 定义固定的颜色映射，BGR格式
        color_map = {
            "circle-blue": (255, 0, 0),    # 蓝色
            "circle-green": (0, 255, 0),   # 绿色
            "circle-red": (0, 0, 255),     # 红色
            "object-blue": (255, 128, 0),  # 浅蓝色
            "object-green": (0, 255, 128), # 浅绿色
            "object-red": (128, 0, 255)    # 浅红色/紫色
        }
        
        # 如果检测到未知类别，则使用黄色作为默认颜色
        default_color = (0, 255, 255)  # 黄色
        
        # 在图像上绘制检测结果
        for detection in detections:
            # 获取边界框坐标
            x1 = int(detection["bbox"]["x1"])
            y1 = int(detection["bbox"]["y1"])
            x2 = int(detection["bbox"]["x2"])
            y2 = int(detection["bbox"]["y2"])
            
            # 获取类别和置信度
            class_name = detection["class_name"]
            confidence = detection["confidence"]
            
            # 获取对应的颜色，如果类别不在颜色映射中则使用默认颜色
            color = color_map.get(class_name, default_color)
            
            # 绘制边界框
            cv2.rectangle(output_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签
            label = f"{class_name}: {confidence:.2f}"
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(output_image, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), color, -1)
            cv2.putText(output_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return output_image


# 便捷函数，用于快速创建检测器
def create_detector(model_path: Optional[str] = None) -> Detector:
    """
    创建并返回一个初始化好的检测器实例
    
    Args:
        model_path (Optional[str]): 模型路径，如果为None则从配置文件读取
        
    Returns:
        Detector: 初始化好的检测器实例
    """
    detector = Detector(model_path)
    if detector.model is None:
        detector.load_model()
    return detector


# 如果直接运行此模块，则执行测试
if __name__ == "__main__":
    # 创建检测器
    detector = create_detector()
    
    # 加载测试图像(如果有)
    test_image_path = project_root / "20250309_112823_094.jpg"
    if test_image_path.exists():
        import cv2
        
        # 读取图像
        image = cv2.imread(str(test_image_path))
        
        # 执行检测
        detections = detector.detect(image)
        print(f"检测到 {len(detections)} 个物体:")
        for i, detection in enumerate(detections):
            print(f"  物体 {i+1}: {detection['class_name']} (置信度: {detection['confidence']:.2f})")
        
        # 可视化结果
        result_image = detector.visualize_detections(image, detections)
        
        # 显示结果
        cv2.imshow("检测结果", result_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"测试图像不存在: {test_image_path}")
        print("请将测试图像放置在 data/test/test_image.jpg 路径下")