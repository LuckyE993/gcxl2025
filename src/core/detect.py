import os
import time
from src.utils.helpers import log_message
from src.config.settings import get_config
from src.core.camera import Camera
from typing import List, Tuple, Dict, Optional

import cv2
import numpy as np
import onnxruntime as ort

# Add OpenVINO import
try:
    import openvino as ov
    OPENVINO_AVAILABLE = True
except ImportError:
    OPENVINO_AVAILABLE = False
    log_message("OpenVINO not available. Install with 'pip install openvino'", level="warning")

from ultralytics.utils import yaml_load
from ultralytics.utils.checks import check_yaml


class Detector:
    """
    Base class for object detection models.

    This class provides the interface and common functionality for object
    detection implementations.

    Attributes:
        confidence_thres (float): Confidence threshold for filtering detections.
        iou_thres (float): IoU threshold for non-maximum suppression.
    """

    def __init__(self, confidence_thres: float = 0.5, iou_thres: float = 0.5):
        """
        Initialize the base Detector class.

        Args:
            confidence_thres (float): Confidence threshold for filtering detections.
            iou_thres (float): IoU threshold for non-maximum suppression.
        """
        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres
        log_message("初始化检测器")

    def detect(self, image: np.ndarray, confidence: Optional[float] = None) -> List[Dict]:
        """
        Detect objects in the input image.

        Args:
            image (np.ndarray): Input image for detection
            confidence (float, optional): Override default confidence threshold

        Returns:
            List[Dict]: List of detection results with keys:
                - 'box': [x, y, w, h] format bounding box
                - 'score': confidence score
                - 'class_id': class identifier
        """
        raise NotImplementedError("子类必须实现检测方法")

    def visualize_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        Draw detection results on the image.

        Args:
            image (np.ndarray): Input image
            detections (List[Dict]): Detection results from detect() method

        Returns:
            np.ndarray: Image with drawn detections
        """
        raise NotImplementedError("子类必须实现可视化方法")


class YOLOv8(Detector):
    """
    YOLOv8 object detection model class for handling inference and visualization.

    This class provides functionality to load a YOLOv8 ONNX model, perform inference on images,
    and visualize the detection results. It also supports OpenVINO INT8 quantized models for
    faster inference on compatible hardware.

    Attributes:
        onnx_model (str): Path to the ONNX model file.
        confidence_thres (float): Confidence threshold for filtering detections.
        iou_thres (float): IoU threshold for non-maximum suppression.
        classes (List[str]): List of class names from the COCO dataset.
        color_palette (np.ndarray): Random color palette for visualizing different classes.
        input_width (int): Width dimension of the model input.
        input_height (int): Height dimension of the model input.
        backend (str): Inference backend to use ('onnx' or 'openvino').
    """

    def __init__(self, 
                 onnx_model: str = "models/default.onnx", 
                 confidence_thres: float = 0.5, 
                 iou_thres: float = 0.5,
                 backend: str = "onnx"):
        """
        Initialize an instance of the YOLOv8 class.

        Args:
            onnx_model (str): Path to the ONNX model.
            confidence_thres (float): Confidence threshold for filtering detections.
            iou_thres (float): IoU threshold for non-maximum suppression.
            backend (str): Inference backend to use ('onnx' or 'openvino').
        """
        super().__init__(confidence_thres, iou_thres)
        
        # Backend selection
        self.backend = backend.lower()
        if self.backend == "openvino" and not OPENVINO_AVAILABLE:
            log_message("OpenVINO requested but not available. Falling back to ONNX Runtime.", level="warning")
            self.backend = "onnx"

        # Load config to get class names and colors
        config = get_config()

        # Try to load class names from config, fallback to COCO dataset if not found
        try:
            # Check if we should use OpenVINO model from config
            if self.backend == "openvino" and 'openvino-int8' in config['model']:
                self.model_path = config['model']['openvino-int8']
                log_message(f"使用OpenVINO INT8模型路径: {self.model_path}")
            else:
                self.model_path = config['model']['path']
                log_message(f"使用的模型路径: {self.model_path}")

            # Load class names from config
            class_dict = config.get('model', {}).get('classes', {})
            if class_dict:
                # Create list of class names, ensuring proper ordering by index
                num_classes = len(class_dict)
                self.class_list = []
                for i in range(num_classes):
                    self.class_list.append(
                        class_dict.get(str(i), f"class_{i}"))

                log_message(f"从配置文件加载了 {len(self.class_list)} 个类别")
                log_message(f"类别列表: {self.class_list}")
                self.classes = class_dict
            else:
                raise KeyError("No classes found in config")

            self.confidence_thres = config['model']['confidence_thres']
            self.iou_thres = config['model']['iou_thres']
            log_message(f"使用的置信度阈值: {self.confidence_thres}")
            log_message(f"使用的IoU阈值: {self.iou_thres}")

            self.input_width = config['model']['input_size']["width"]
            self.input_height = config['model']['input_size']["height"]
            log_message(f"输入尺寸: {self.input_width}x{self.input_height}")

        except (KeyError, TypeError):
            # Fallback to COCO dataset
            log_message("未找到配置中的类别，使用COCO默认类别")
            self.classes = yaml_load(check_yaml("coco8.yaml"))["names"]
            self.class_list = self.classes
            log_message(f"使用的模型路径: {onnx_model}")
            self.model_path = onnx_model
            log_message(f"使用的置信度阈值: {confidence_thres}")
            log_message(f"使用的IoU阈值: {iou_thres}")
            self.input_width = 640  # Default, will be updated from model
            self.input_height = 640  # Default, will be updated from model
            log_message(f"输入尺寸: {self.input_width}x{self.input_height}")

        # Define colors for each class - with specific colors for known categories
        self.color_palette = self._generate_color_palette()

        # Initialize model session
        self._initialize_model()

    def _generate_color_palette(self) -> Dict[int, Tuple[int, int, int]]:
        """Generate color palette for visualization with special handling for known colors"""
        colors = {}
        # Generate random colors as fallback
        random_colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # Assign specific colors based on class name patterns
        for i in range(len(self.classes)):
            class_name = self.class_list[i].lower()

            if "red" in class_name:
                colors[i] = (0, 0, 255)  # BGR: Red
            elif "blue" in class_name:
                colors[i] = (255, 0, 0)  # BGR: Blue
            elif "green" in class_name:
                colors[i] = (0, 255, 0)  # BGR: Green
            else:
                # Use random color for other classes
                colors[i] = tuple(map(int, random_colors[i]))

        return colors

    def _initialize_model(self):
        """Initialize the model session with either ONNX Runtime or OpenVINO."""
        log_message(f"使用 {self.backend} 后端初始化模型")
        
        if self.backend == "openvino":
            try:
                # Initialize OpenVINO runtime
                self.core = ov.Core()
                
                # 设置额外的配置参数
                config = {
                    "PERFORMANCE_HINT": "LATENCY",
                    "INFERENCE_NUM_THREADS": 8,
                    "CACHE_DIR": "./model_cache"
                }
                
                # 读取模型
                if os.path.isdir(self.model_path):
                    model_xml = os.path.join(self.model_path, "best.xml")
                    if not os.path.exists(model_xml):
                        raise FileNotFoundError(f"OpenVINO XML file not found: {model_xml}")
                    model = self.core.read_model(model_xml)
                else:
                    model = self.core.read_model(self.model_path)
                
                # 获取输入细节
                self.model_input_name = model.input(0).any_name
                input_shape = model.input(0).shape
                if len(input_shape) == 4:
                    self.input_width = input_shape[3]
                    self.input_height = input_shape[2]
                
                # 使用优化配置编译模型
                self.compiled_model = self.core.compile_model(model, device_name="CPU", config=config)
                # 创建多个推理请求以支持异步处理
                self.infer_request = self.compiled_model.create_infer_request()
                
                log_message(f"OpenVINO模型初始化完成，输入尺寸: {self.input_width}x{self.input_height}")
        
            
            except Exception as e:
                log_message(f"OpenVINO模型初始化失败: {e}", level="error")
                log_message("回退到ONNX Runtime", level="warning")
                self.backend = "onnx"
                self._initialize_onnx_model()
        else:
            # Initialize with ONNX Runtime
            self._initialize_onnx_model()

    def _initialize_onnx_model(self):
        """Initialize the model using ONNX Runtime."""
        # 配置推理选项以优化性能
        session_options = ort.SessionOptions()

        # 启用图形优化
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        # 启用内存模式优化
        session_options.enable_mem_pattern = True

        # 启用并行推理 (根据CPU核心数量)
        session_options.intra_op_num_threads = os.cpu_count()  # 使用所有可用的CPU核心
        # Create an inference session using the ONNX model and specify execution providers
        self.session = ort.InferenceSession(
            self.model_path,
            sess_options=session_options,
            providers=["CPUExecutionProvider"]
        )

        # Get the model inputs and set dimensions
        model_inputs = self.session.get_inputs()
        input_shape = model_inputs[0].shape
        self.input_width = input_shape[2]
        self.input_height = input_shape[3]
        self.model_input_name = model_inputs[0].name

        log_message(
            f"ONNX模型初始化完成，输入尺寸: {self.input_width}x{self.input_height}")

    def letterbox(self, img: np.ndarray, new_shape: Tuple[int, int] = (640, 640)) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        Resize and reshape images while maintaining aspect ratio by adding padding.

        Args:
            img (np.ndarray): Input image to be resized.
            new_shape (Tuple[int, int]): Target shape (height, width) for the image.

        Returns:
            (np.ndarray): Resized and padded image.
            (Tuple[int, int]): Padding values (top, left) applied to the image.
        """
        shape = img.shape[:2]  # current shape [height, width]

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = (new_shape[1] - new_unpad[0]) / \
            2, (new_shape[0] - new_unpad[1]) / 2  # wh padding

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(
            img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        return img, (top, left)

    def draw_detections(self, img: np.ndarray, box: List[float], score: float, class_id: int) -> None:
        """
        Draw bounding boxes and labels on the input image based on the detected objects.

        Args:
            img (np.ndarray): The input image to draw detections on.
            box (List[float]): Detected bounding box coordinates [x, y, width, height].
            score (float): Confidence score of the detection.
            class_id (int): Class ID for the detected object.
        """
        # Extract the coordinates of the bounding box
        x1, y1, w, h = box

        # Retrieve the color for the class ID
        color = self.color_palette[class_id]

        # Draw the bounding box on the image
        cv2.rectangle(img, (int(x1), int(y1)),
                      (int(x1 + w), int(y1 + h)), color, 2)

        # Create the label text with class name and score
        label = f"{self.class_list[class_id]}: {score:.2f}"

        # Calculate the dimensions of the label text
        (label_width, label_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

        # Calculate the position of the label text
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10

        # Draw a filled rectangle as the background for the label text
        cv2.rectangle(
            img, (label_x, label_y - label_height), (label_x +
                                                     label_width, label_y + label_height), color, cv2.FILLED
        )

        # Draw the label text on the image
        cv2.putText(img, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def preprocess(self, img: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        Preprocess the input image before performing inference.

        This method converts color space, applies letterboxing to maintain aspect ratio,
        normalizes pixel values, and prepares the image data for model input.

        Args:
            img (np.ndarray): Input BGR image (OpenCV format)

        Returns:
            (np.ndarray): Preprocessed image data ready for inference with shape (1, 3, height, width).
            (Tuple[int, int]): Padding values (top, left) applied during letterboxing.
        """
        # 避免不必要的复制
        if img.shape[2] == 3:  # 确保是BGR格式
            # 使用更快的转换方法
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            img_rgb = img

        # 降低分辨率进行推理以提高速度
        target_size = (self.input_width, self.input_height)
        resized_img, pad = self.letterbox(img_rgb, target_size)

        # 使用更高效的数组操作
        image_data = resized_img.astype(np.float32) / 255.0

        # 使用transpose而不是reshape
        image_data = np.transpose(image_data, (2, 0, 1))
        image_data = np.expand_dims(image_data, axis=0)

        return image_data, pad

    def postprocess(self, original_img: np.ndarray, output: List[np.ndarray], pad: Tuple[int, int]) -> List[Dict]:
        """
        Process the model's output to extract detection information.

        Args:
            original_img (np.ndarray): The original input image.
            output (List[np.ndarray]): The output arrays from the model.
            pad (Tuple[int, int]): Padding values (top, left) used during letterboxing.

        Returns:
            List[Dict]: List of detection results with keys 'box', 'score', and 'class_id'.
        """
        # Transpose and squeeze the output to match the expected shape
        outputs = np.transpose(np.squeeze(output[0]))

        # Get the number of rows in the outputs array
        rows = outputs.shape[0]

        # Lists to store the bounding boxes, scores, and class IDs of the detections
        boxes = []
        scores = []
        class_ids = []

        img_height, img_width = original_img.shape[:2]

        # Calculate the scaling factors for the bounding box coordinates
        gain = min(self.input_height / img_height,
                   self.input_width / img_width)
        outputs[:, 0] -= pad[1]
        outputs[:, 1] -= pad[0]

        # Iterate over each row in the outputs array
        for i in range(rows):
            # Extract the class scores from the current row
            classes_scores = outputs[i][4:]

            # Find the maximum score among the class scores
            max_score = np.amax(classes_scores)

            # If the maximum score is above the confidence threshold
            if max_score >= self.confidence_thres:
                # Get the class ID with the highest score
                class_id = np.argmax(classes_scores)

                # Extract the bounding box coordinates from the current row
                x, y, w, h = outputs[i][0], outputs[i][1], outputs[i][2], outputs[i][3]

                # Calculate the scaled coordinates of the bounding box
                left = int((x - w / 2) / gain)
                top = int((y - h / 2) / gain)
                width = int(w / gain)
                height = int(h / gain)

                # Add the class ID, score, and box coordinates to the respective lists
                class_ids.append(class_id)
                scores.append(max_score)
                boxes.append([left, top, width, height])

        # Apply non-maximum suppression to filter out overlapping bounding boxes
        indices = cv2.dnn.NMSBoxes(
            boxes, scores, self.confidence_thres, self.iou_thres)

        # Create the list of detection results
        detections = []
        for i in indices:
            detections.append({
                'box': boxes[i],
                'score': scores[i],
                'class_id': class_ids[i]
            })

        return detections

    def detect(self, image: np.ndarray, confidence: Optional[float] = None) -> List[Dict]:
        # Save original confidence and update if needed
        original_confidence = self.confidence_thres
        if confidence is not None:
            self.confidence_thres = confidence
    
        try:
            # Preprocess the image
            img_data, pad = self.preprocess(image)
    
            # Run inference based on the selected backend
            if self.backend == "openvino":
                # OpenVINO 异步推理
                input_tensor = ov.Tensor(array=img_data)
                self.infer_request.set_input_tensor(input_tensor)
                self.infer_request.start_async()
                self.infer_request.wait()
                outputs = [self.infer_request.get_output_tensor(0).data]
            else:
                # ONNX Runtime inference
                outputs = self.session.run(None, {self.model_input_name: img_data})
    
            # Postprocess the outputs
            detections = self.postprocess(image, outputs, pad)
    
            return detections
        finally:
            # Restore original confidence threshold
            if confidence is not None:
                self.confidence_thres = original_confidence

    def visualize_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        Draw detection results on the image.

        Args:
            image (np.ndarray): Input image
            detections (List[Dict]): Detection results from detect() method

        Returns:
            np.ndarray: Image with drawn detections
        """
        # Create a copy of the image to avoid modifying the original
        result_image = image.copy()

        # Draw each detection on the image
        for detection in detections:
            self.draw_detections(
                result_image,
                detection['box'],
                detection['score'],
                detection['class_id']
            )

        return result_image


class QrCodeDetector(Detector):
    """
    QR码检测器类，用于识别和解码图像中的二维码。
    
    可以识别特定格式的任务编码，任务码由两组三位数组成，如"123+231"，
    其中"1"代表红色，"2"代表绿色，"3"代表蓝色。
    """
    
    def __init__(self):
        """初始化QR码检测器"""
        super().__init__()
        # 初始化OpenCV的QR码检测器
        self.qr_detector = cv2.QRCodeDetector()
        log_message("初始化QR码检测器")
        
    def detect(self, image: np.ndarray, confidence: Optional[float] = None) -> List[Dict]:
        """
        检测图像中的二维码并解码
        
        Args:
            image (np.ndarray): 输入图像
            confidence (float, optional): 置信度阈值 (本类中不使用)
            
        Returns:
            List[Dict]: 检测结果列表，每个元素包含:
                - 'box': [x, y, w, h] 格式的边界框
                - 'data': 解码后的数据
                - 'is_valid': 是否为有效的任务码
                - 'mission': 解析后的任务码 (如果有效)
        """
        # 检测和解码二维码
        ret_val, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(image)
        
        detections = []
        if ret_val:
            for i, data in enumerate(decoded_info):
                if data:  # 如果成功解码
                    # 计算边界框
                    qr_points = points[i]
                    x, y, w, h = self._calculate_bbox(qr_points)
                    
                    # 验证和解析任务码
                    is_valid, mission = self._validate_mission_code(data)
                    
                    detections.append({
                        'box': [x, y, w, h],
                        'data': data,
                        'is_valid': is_valid,
                        'mission': mission if is_valid else None
                    })
        
        return detections
    
    def _calculate_bbox(self, points: np.ndarray) -> List[int]:
        """
        从QR码角点计算边界框
        
        Args:
            points (np.ndarray): QR码角点坐标
            
        Returns:
            List[int]: [x, y, w, h] 格式的边界框
        """
        # 计算最小外接矩形
        x_min = int(np.min(points[:, 0]))
        y_min = int(np.min(points[:, 1]))
        x_max = int(np.max(points[:, 0]))
        y_max = int(np.max(points[:, 1]))
        
        # 返回左上角坐标和宽高
        return [x_min, y_min, x_max - x_min, y_max - y_min]
    
    def _validate_mission_code(self, code: str) -> Tuple[bool, Optional[Dict]]:
        """
        验证并解析任务码
        
        Args:
            code (str): 解码后的QR码数据
            
        Returns:
            Tuple[bool, Optional[Dict]]: 
                - 是否为有效格式
                - 解析后的任务信息 (如果有效)
        """
        # 检查任务码格式 (如 "123+231")
        import re
        pattern = r'^([123]{3})\+([123]{3})$'
        match = re.match(pattern, code)
        
        if not match:
            return False, None
            
        # 提取两组三位数
        first_group = match.group(1)
        second_group = match.group(2)
        
        # 解析颜色顺序
        color_map = {'1': 'red', '2': 'green', '3': 'blue'}
        
        first_sequence = [color_map[digit] for digit in first_group]
        second_sequence = [color_map[digit] for digit in second_group]
        
        mission = {
            'raw_code': code,
            'first_batch': first_sequence,
            'second_batch': second_sequence
        }
        
        return True, mission
        
    def visualize_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        在图像上可视化二维码检测结果
        
        Args:
            image (np.ndarray): 输入图像
            detections (List[Dict]): 检测结果列表
            
        Returns:
            np.ndarray: 标注后的图像
        """
        # 创建图像副本
        result_image = image.copy()
        
        for detection in detections:
            # 获取边界框和数据
            x, y, w, h = detection['box']
            data = detection['data']
            is_valid = detection['is_valid']
            
            # 根据有效性选择颜色 (绿色表示有效，红色表示无效)
            color = (0, 255, 0) if is_valid else (0, 0, 255)
            
            # 绘制边界框
            cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
            
            # 添加文本背景
            text_bg = np.zeros((30, w + 20, 3), dtype=np.uint8)
            text_bg[:, :] = (50, 50, 50)
            
            # 在图像上方显示解码数据
            cv2.putText(text_bg, f"QR: {data}", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            
            # 将文本背景添加到图像中
            bg_y = max(0, y - 30)
            if bg_y + 30 <= result_image.shape[0] and x + w + 20 <= result_image.shape[1]:
                # 注意：这里的alpha混合可能需要调整边缘情况
                result_image[bg_y:bg_y+30, x:x+w+20] = \
                    cv2.addWeighted(result_image[bg_y:bg_y+30, x:x+w+20], 0.5, text_bg, 0.5, 0)
            
            # 如果是有效的任务码，显示任务信息
            if is_valid and detection['mission']:
                mission = detection['mission']
                batch1 = '->'.join(mission['first_batch'])
                batch2 = '->'.join(mission['second_batch'])
                
                y_offset = y + h + 15
                if y_offset < result_image.shape[0] - 40:
                    cv2.putText(result_image, f"Batch 1: {batch1}", 
                               (x, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                    cv2.putText(result_image, f"Batch 2: {batch2}", 
                               (x, y_offset + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        
        return result_image
    
    def detect_from_camera(self, camera: Camera, display: bool = True) -> Optional[Dict]:
        """
        从摄像头检测二维码并返回有效的任务码
        
        Args:
            camera (Camera): 摄像头实例
            display (bool): 是否显示检测结果
            
        Returns:
            Optional[Dict]: 如果找到有效任务码，返回任务信息，否则返回None
        """
        log_message("开始从摄像头检测QR码，按'q'退出")
        
        # 帧计数和FPS计算
        frame_count = 0
        start_time = cv2.getTickCount()
        fps = 0
        
        while True:
            # 读取帧
            frame = camera.read_frame()
            if frame is None:
                log_message("无法从摄像头读取帧")
                break
            
            # 检测二维码
            detections = self.detect(frame)
            
            # 可视化检测结果
            result_frame = self.visualize_detections(frame, detections)
            
            # 计算并显示FPS
            frame_count += 1
            if frame_count >= 10:  # 每10帧更新一次FPS
                current_time = cv2.getTickCount()
                elapsed_time = (current_time - start_time) / cv2.getTickFrequency()
                fps = frame_count / elapsed_time
                frame_count = 0
                start_time = current_time
            
            # 添加FPS文本
            cv2.putText(result_frame, f"FPS: {fps:.1f}", (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            # 显示检测数量
            cv2.putText(result_frame, f"QR Codes: {len(detections)}", (20, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            # 显示图像
            if display:
                cv2.imshow("QR Code Detection", result_frame)
            
            # 检查是否有有效的任务码
            for detection in detections:
                if detection['is_valid']:
                    if display:
                        # 显示3秒后返回
                        cv2.waitKey(3000)
                        cv2.destroyAllWindows()
                    return detection['mission']
            
            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        if display:
            cv2.destroyAllWindows()
        
        return None


def detect_camera(camera_id=0, detecor: Optional[YOLOv8] = None, confidence: Optional[float] = None, display: bool = True) -> None:
    """
    Real-time object detection using camera feed.

    Args:
        camera_id (int): Camera device index
        confidence (float, optional): Override default confidence threshold
        display (bool): Whether to display the results in a window

    Returns:
        None
    """
    # Initialize video capture
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        log_message(f"无法打开摄像头 {camera_id}")
        return

    # Variables for FPS calculation
    frame_count = 0
    start_time = cv2.getTickCount()
    fps = 0

    log_message(f"开始摄像头检测，按'q'退出")

    while True:
        # Read frame
        ret, frame = cap.read()
        if not ret:
            log_message("无法读取摄像头帧")
            break

        # Detect objects
        detections = detecor.detect(frame, confidence)
        log_message(f"使用的模型输入尺寸为: {detecor.input_width}x{detecor.input_height}")
        # Draw detections
        result_frame = detecor.visualize_detections(frame, detections)

        # Calculate and display FPS
        frame_count += 1
        if frame_count >= 10:  # Update FPS every 10 frames
            current_time = cv2.getTickCount()
            elapsed_time = (current_time - start_time) / \
                cv2.getTickFrequency()
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = current_time

        # Add FPS text
        cv2.putText(result_frame, f"FPS: {fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display number of detections
        cv2.putText(result_frame, f"Detect: {len(detections)}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if display:
            cv2.imshow("YOLOv8 Camera Detection", result_frame)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    if display:
        cv2.destroyAllWindows()

    log_message("摄像头检测结束")


if __name__ == "__main__":
    # 控制是否显示UI界面的标志
    display_ui = True  # 设置为False可禁用所有UI显示

    camera_id = 0  # 默认相机ID
    camera = Camera(camera_id=camera_id, resolution=(640, 480))
    try:
        if camera.open():
            print("摄像头属性:", camera.get_properties())
    except Exception as e:
        print(f"摄像头打开失败: {e}")
        exit(1)

    # 使用不同的后端初始化检测器
    # 选择 'onnx' 或 'openvino' 作为后端
    backend = 'openvino'  # 可以改为 'onnx'
    
    detector = YOLOv8(backend=backend)
    log_message(f"使用 {backend} 后端进行推理")
    
    try:
        if display_ui:
            print(f"按 'q' 键退出，按 's' 键切换推理后端")
        
        current_backend = backend
        # 在主循环中添加帧跳过策略
        processing = False
        
        while True:
            frame = camera.read_frame()
            
            # 如果当前正在处理一帧，跳过此帧
            if not processing and frame is not None:
                processing = True
                
                start_time = time.time()
                detections = detector.detect(frame)
                inference_time = (time.time() - start_time) * 1000
                
                # 检测结果显示在控制台（无论是否启用UI）
                for detection in detections:
                    class_name = detector.class_list[detection['class_id']]
                    log_message(f"模型尺寸: {detector.input_width}x{detector.input_height}, "
                        f"检测到: {class_name}, 置信度: {detection['score']:.2f}, "
                              f"位置: {detection['box']}", level="info")
                
                if display_ui:
                    result_image = detector.visualize_detections(frame, detections)
                    
                    # 显示推理时间和当前后端
                    cv2.putText(result_image, f"Backend: {detector.backend}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(result_image, f"Inference: {inference_time:.1f}ms", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    cv2.imshow("YOLOv8 Detection", result_image)
                
                processing = False
                
            # 按键处理
            if display_ui:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    break
                elif key == ord('s'):
                    # 切换后端
                    new_backend = 'onnx' if detector.backend == 'openvino' else 'openvino'
                    log_message(f"切换后端从 {detector.backend} 到 {new_backend}")
                    detector = YOLOv8(backend=new_backend)
            
            # # 捕获一张图像并保存
            # image = camera.read_frame()
            # if image is not None:
                #     detections = detector.detect(frame)
            #     result_image = detector.visualize_detections(frame, detections)
            #     save_path = "captured_image.jpg"
            #     cv2.imwrite(save_path, result_image)
            #     print(f"图像已保存到: {save_path}")
            #     cv2.imshow("Captured Image", result_image)
            #     cv2.waitKey(0)

            # # 二维码检测模式

            # qr_detector = QrCodeDetector()
            # mission = qr_detector.detect_from_camera(camera)
            
            # if mission:
            #     print("检测到有效的任务码:")
            #     print(f"  原始代码: {mission['raw_code']}")
            #     print(f"  第一批: {' -> '.join(mission['first_batch'])}")
            #     print(f"  第二批: {' -> '.join(mission['second_batch'])}")
            # else:
            #     print("未检测到有效的任务码")


        # 关闭摄像头
        camera.close()
        if display_ui:
            cv2.destroyAllWindows()

    except KeyboardInterrupt as e:
        print("用户中断程序")
    finally:
        camera.close()
        log_message("摄像头关闭")
        if display_ui:
            cv2.destroyAllWindows()
        print("程序结束")

