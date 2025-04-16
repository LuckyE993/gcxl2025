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
from src.core import control
from src.utils.helpers import log_message
from src.config.settings import get_config

def main():
    """应用程序入口函数"""
    log_message("程序开始运行")
    config = get_config()
    log_message(f"配置: {config}")

    # 初始化摄像头
    cam_front = camera.Camera(0,(640, 480))
    cam_down = camera.Camera(1,(640, 480))
    try:
        if cam_front.open():
            # 修改这一行，将字典转换为字符串
            log_message(f"cam_front properties: {cam_front.get_properties()}")
        else:
            log_message("无法打开正面摄像头", level="warning")
            
        if cam_down.open():
            # 修改这一行，将字典转换为字符串
            log_message(f"cam_down properties: {cam_down.get_properties()}")
        else:
            log_message("无法打开底部摄像头", level="warning")
            
    except Exception as e:
        log_message(f"Camera open failed: {e}", level="error")
        exit(1)
    
    
    
    return 0


if __name__ == "__main__":
    exit(main())