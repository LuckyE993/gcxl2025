"""
应用程序主入口点。
"""

import cv2
import time
import sys
from pathlib import Path

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))

from src.core import detector
from src.core import camera
from src.utils import helpers

def main():
    """应用程序入口函数"""
    print("gcxl2025 应用程序启动")
    
    # 创建对象检测器
    object_detector = detector.create_detector()
    if object_detector.model is None:
        print("模型加载失败，应用程序无法继续")
        return 1
    
    # 创建并开启摄像头
    cam = camera.Camera(camera_id=0, resolution=(640, 480))
    if not cam.open():
        print("摄像头打开失败，应用程序无法继续")
        return 1
    
    # 创建FPS计数器
    fps_counter = camera.FPSCounter(update_interval=10)
    
    print("按 'q' 键退出程序")
    
    try:
        while True:
            # 读取一帧图像
            image = cam.read_frame()
            if image is None:
                print("未能捕获图像")
                break
            
            # 更新FPS
            fps_counter.update()
            
            # 执行对象检测
            detections = object_detector.detect(image, confidence=0.8)
            
            # 可视化检测结果
            result_image = object_detector.visualize_detections(image, detections)
            
            # 添加FPS信息到图像
            fps_counter.draw_fps(result_image)
            
            # 显示检测结果
            cv2.imshow("对象检测", result_image)
            
            # 检测按键，按q退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("用户请求退出")
                break
            
    except KeyboardInterrupt:
        print("用户中断程序")
    except Exception as e:
        print(f"程序异常: {str(e)}")
    finally:
        # 关闭资源
        cam.close()
        cv2.destroyAllWindows()
    
    print("gcxl2025 应用程序结束")
    return 0


if __name__ == "__main__":
    exit(main())