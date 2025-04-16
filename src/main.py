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

from src.core import detect
from src.core import camera
from src.core import control
from src.core.communication import SerialCommunication
from src.utils.helpers import log_message
from src.config.settings import get_config

def initialize_cameras(config):
    """初始化摄像头"""
    log_message("正在初始化摄像头...")
    cam_front = camera.Camera(config["camera"]["camera_id_front"], (640, 480))
    cam_down = camera.Camera(config["camera"]["camera_id_down"], (640, 480))
    
    try:
        if cam_front.open():
            log_message(f"cam_front properties: {cam_front.get_properties()}")
        else:
            log_message("无法打开正面摄像头", level="warning")
            
        if cam_down.open():
            log_message(f"cam_down properties: {cam_down.get_properties()}")
        else:
            log_message("无法打开底部摄像头", level="warning")
            
    except Exception as e:
        log_message(f"Camera open failed: {e}", level="error")
        return None, None
        
    return cam_front, cam_down

def initialize_communication(config):
    """初始化串口通信"""
    log_message("正在初始化串口通信...")
    com_port = config["communication"]["com"]
    baudrate = config["communication"]["baudrate"]
    timeout = config["communication"]["timeout"]

    serial_comm = SerialCommunication(
        port=com_port, baudrate=baudrate, timeout=timeout)
    
    log_message(f"成功初始化串口通信: {com_port}, 波特率: {baudrate}")
    if serial_comm.open():
        log_message("串口已打开")
        return serial_comm
    else:
        log_message("串口未打开", level="error")
        return None

def initialize_controllers(config, serial_comm):
    """初始化车辆和机械臂控制器"""
    log_message("正在初始化控制器...")
    # 初始化车辆控制对象
    vehicle = control.VehicleControl(config)
    vehicle.set_communication(serial_comm)

    time.sleep(1)

    # 初始化机械臂控制对象
    arm = control.ArmControl(config)
    arm.set_communication(serial_comm)
    arm.initialize()
    
    return vehicle, arm

def main():
    """应用程序入口函数"""
    log_message("程序开始运行")
    config = get_config()
    log_message(f"配置: {config}")

    # 初始化摄像头
    cam_front, cam_down = initialize_cameras(config)
    if cam_front is None or cam_down is None:
        exit(1)
    
    # 初始化串口通信
    serial_comm = initialize_communication(config)
    if serial_comm is None:
        sys.exit(1)
        
    # 初始化车辆和机械臂控制器
    vehicle, arm = initialize_controllers(config, serial_comm)

    # 初始化游戏控制器
    controller = control.PyGameController(vehicle, arm)
    controller.set_vehicle_control(vehicle)
    controller.set_arm_control(arm)
    
    detector = detect.YOLOv8()
    qr_detector = detect.QrCodeDetector()
    
    
    return 0


if __name__ == "__main__":
    exit(main())