from src.config.settings import get_config
from src.utils.helpers import log_message
from src.core.communication import SerialCommunication
from src.core import control
from src.core.control import adjust_position_to_target
from src.core import camera
from src.core import detect
import cv2
import time
import sys
from pathlib import Path
import threading
import queue
from threading import Thread, Event
import keyboard  # 添加keyboard库用于键盘监听
# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))

# 全局变量用于控制定位测试
positioning_active = False
positioning_start_time = 0
positioning_times = []


def initialize_cameras(config):
    """初始化摄像头"""
    log_message("正在初始化摄像头...")

    # 使用新的配置函数创建摄像头
    cam_front, cam_down = camera.get_camera_from_config()

    try:
        # 检查前置摄像头
        if cam_front is not None:
            log_message(f"cam_front properties: {cam_front.get_properties()}")
        else:
            log_message("无法打开正面摄像头", level="warning")
            # 备用方案：手动创建前置摄像头
            cam_front = camera.Camera(
                config["camera"]["camera_id_front"], (640, 480), apply_correction=False)
            if not cam_front.open():
                cam_front = None

        # 检查下置摄像头
        if cam_down is not None:
            log_message(f"cam_down properties: {cam_down.get_properties()}")
            log_message("下置摄像头已启用广角畸变校正")
        else:
            log_message("无法打开底部摄像头", level="warning")
            # 备用方案：手动创建下置摄像头
            cam_down = camera.Camera(
                config["camera"]["camera_id_down"], (640, 480), apply_correction=True)
            if not cam_down.open():
                cam_down = None

    except Exception as e:
        log_message(f"摄像头初始化异常: {e}", level="error")
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
 # 定义数据接收回调函数


def data_received_callback(data):
    hex_data = SerialCommunication.bytes_to_hex(data)
    log_message(
        f"收到数据: {hex_data} (ASCII: {data.decode('ascii', errors='replace')})")
    global positionFinish
    if len(data) >= 10 and data[0] == 0xFF and data[1] == 0x00 and data[2] == 0x01 and \
            data[3] == 0x00 and data[4] == 0x00 and data[5] == 0x00 and data[6] == 0x00 and \
            data[7] == 0x00 and data[8] == 0x00 and data[9] == 0xFE:
        positionFinish = True
        log_message(
            "Received position finish signal, positionFinish set to True")


def move_and_wait_for_completion(vehicle, x, y, angle=0, timeout=20):
    """
    发送位置移动命令并等待完成

    Args:
        vehicle: 车辆控制对象
        x (float): X轴移动距离
        y (float): Y轴移动距离
        angle (float, optional): 旋转角度，默认为0
        timeout (int, optional): 超时时间(秒)，默认为20秒

    Returns:
        bool: 如果位置调整完成返回True，超时返回False
    """
    global positionFinish
    positionFinish = False

    # 发送位置调整命令
    log_message(f"发送位置移动命令: x={x:.2f}, y={y:.2f}, angle={angle:.2f}")
    vehicle.position_control(x, y, angle)

    # 等待位置调整完成
    log_message("等待位置调整完成...")
    start_time = time.time()

    while not positionFinish:
        time.sleep(0.05)
        if time.time() - start_time > timeout:
            log_message("位置调整超时", level="warning")
            return False

    log_message("位置调整完成")
    return True


def track_target(config, vehicle, camera_down, detector):

    pixel_distance_ratio = config.get("vehicle", {}).get(
        "PIXEL_DISTANCE_RATIO_HEIGHT_0_OBJECT_1", 0.2848)
    log_message(
        f"像素距离比率: {pixel_distance_ratio:.4f} (高度0对象1)")

    # 创建检测结果队列、停止事件和调整完成事件
    detection_queue = queue.Queue(maxsize=1)  # 只保留最新的检测结果
    stop_event = threading.Event()
    # adjustment_complete = threading.Event()

    # 定义位置调整线程函数
    def adjust_position_thread():
        log_message("位置调整线程已启动")

        while not stop_event.is_set():
            try:
                # 尝试从队列获取最新的检测结果，等待最多2秒
                try:
                    detections = detection_queue.get(timeout=2.0)

                    # 调整位置
                    position_adjusted = adjust_position_to_target(
                        target_class_id=None,  # 指定目标 color_info['class_id']
                        detections=detections,
                        pixel_distance_ratio=pixel_distance_ratio,
                        vehicle=vehicle,
                        frame_size=(640, 480),
                        config=config
                    )

                    if position_adjusted:
                        # log_message(f"已成功调整位置到 {color_info['name']} 圆环")
                        log_message("已成功调整位置到目标位置")
                        # adjustment_complete.set()
                        # break

                except queue.Empty:
                    # 队列为空，继续等待
                    continue

            except Exception as e:
                log_message(f"位置调整线程异常: {e}", level="error")
                time.sleep(0.5)

        log_message("位置调整线程已结束")

    # 创建并启动位置调整线程
    adjustment_thread = threading.Thread(target=adjust_position_thread)
    adjustment_thread.daemon = True
    adjustment_thread.start()

    # 主线程进行检测
    max_detection_time = 30  # 最大检测时间(秒)
    start_time = time.time()

    log_message("开始检测圆环...")
    # while not adjustment_complete.is_set() and (time.time() - start_time) < max_detection_time:
    while True :
        try:
            # 获取一帧图像
            frame = camera_down.read_frame()
            if frame is None:
                log_message("无法读取图像帧", level="warning")
                time.sleep(0.1)
                continue

            # 执行物体检测
            detections = detector.detect(frame)

            # 更新队列中的检测结果(丢弃旧的)
            try:
                # 如果队列已满，先清空
                if detection_queue.full():
                    detection_queue.get_nowait()
                detection_queue.put_nowait(detections)
            except queue.Full:
                pass  # 极少情况下会发生，可以忽略

            # 显示检测结果（可选）
            detect_img = detector.visualize_detections(
                frame.copy(), detections)
            cv2.imshow("Ring Detection", detect_img)
            cv2.waitKey(1)

            time.sleep(0.03)  # 控制检测频率

        except Exception as e:
            log_message(f"检测异常: {e}", level="error")
            time.sleep(0.5)

    # 关闭显示窗口
    cv2.destroyAllWindows()

    # 停止调整线程
    stop_event.set()
    adjustment_thread.join(timeout=1.0)  # 等待线程结束，最多等待1秒

    if adjustment_complete.is_set():
        log_message("位置已成功调整到目标位置", level="info")
    else:
        log_message("位置调整超时，继续执行下一步", level="warning")

    # 等待车辆稳定
    time.sleep(0.2)


def track_target_with_timing(config, vehicle, camera_down, detector):
    """
    带时间记录的目标跟踪函数
    """
    global positioning_active, positioning_start_time, positioning_times
    
    pixel_distance_ratio = config.get("vehicle", {}).get(
        "PIXEL_DISTANCE_RATIO_HEIGHT_0_OBJECT_1", 0.2848)
    log_message(
        f"像素距离比率: {pixel_distance_ratio:.4f} (高度0对象1)")

    # 创建检测结果队列、停止事件和调整完成事件
    detection_queue = queue.Queue(maxsize=1)  # 只保留最新的检测结果
    stop_event = threading.Event()
    adjustment_complete = threading.Event()

    # 定义位置调整线程函数
    def adjust_position_thread():
        global positioning_active, positioning_start_time
        log_message("位置调整线程已启动")

        while not stop_event.is_set():
            try:
                # 只有在定位激活状态下才进行调整
                if not positioning_active:
                    time.sleep(0.1)
                    continue
                    
                # 尝试从队列获取最新的检测结果，等待最多0.5秒
                try:
                    detections = detection_queue.get(timeout=0.5)

                    # 调整位置
                    position_adjusted = adjust_position_to_target(
                        target_class_id=None,
                        detections=detections,
                        pixel_distance_ratio=pixel_distance_ratio,
                        vehicle=vehicle,
                        frame_size=(640, 480),
                        config=config
                    )

                    if position_adjusted:
                        # 记录定位完成时间
                        end_time = time.time()
                        positioning_time = end_time - positioning_start_time
                        positioning_times.append(positioning_time)
                        
                        log_message(f"定位完成！用时: {positioning_time:.3f}秒")
                        print(f"*** 定位用时: {positioning_time:.3f}秒 ***")
                        
                        # 重置定位状态
                        positioning_active = False
                        adjustment_complete.set()

                except queue.Empty:
                    # 队列为空，继续等待
                    continue

            except Exception as e:
                log_message(f"位置调整线程异常: {e}", level="error")
                time.sleep(0.5)

        log_message("位置调整线程已结束")

    # 创建并启动位置调整线程
    adjustment_thread = threading.Thread(target=adjust_position_thread)
    adjustment_thread.daemon = True
    adjustment_thread.start()

    log_message("定位测试系统已启动")
    log_message("按 'SPACE' 键开始新的定位测试")
    log_message("按 'ESC' 键退出程序")
    log_message("按 'R' 键显示统计结果")

    try:
        while True:
            # 检查键盘输入
            if keyboard.is_pressed('space') and not positioning_active:
                # 开始新的定位测试
                positioning_active = True
                positioning_start_time = time.time()
                adjustment_complete.clear()
                log_message("开始定位测试...")
                print(f"*** 开始定位测试 #{len(positioning_times) + 1} ***")
                
            elif keyboard.is_pressed('r'):
                # 显示统计结果
                print_positioning_statistics()
                time.sleep(0.5)  # 防止重复触发
                
            elif keyboard.is_pressed('esc'):
                # 退出程序
                log_message("用户请求退出程序")
                break

            # 获取一帧图像并进行检测
            try:
                frame = camera_down.read_frame()
                if frame is None:
                    log_message("无法读取图像帧", level="warning")
                    time.sleep(0.1)
                    continue

                # 执行物体检测
                detections = detector.detect(frame)

                # 更新队列中的检测结果(丢弃旧的)
                try:
                    if detection_queue.full():
                        detection_queue.get_nowait()
                    detection_queue.put_nowait(detections)
                except queue.Full:
                    pass

                # 显示检测结果
                detect_img = detector.visualize_detections(frame.copy(), detections)
                
                # 在图像上显示状态信息
                status_text = "POSITIONING..." if positioning_active else "READY (Press SPACE to start)"
                cv2.putText(detect_img, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if positioning_active else (255, 255, 255), 2)
                
                if positioning_times:
                    last_time_text = f"Last time: {positioning_times[-1]:.3f}s"
                    cv2.putText(detect_img, last_time_text, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                cv2.imshow("Positioning Test", detect_img)
                cv2.waitKey(1)

                time.sleep(0.03)  # 控制检测频率

            except Exception as e:
                log_message(f"检测异常: {e}", level="error")
                time.sleep(0.5)

    except KeyboardInterrupt:
        log_message("程序被中断")

    finally:
        # 清理资源
        cv2.destroyAllWindows()
        stop_event.set()
        adjustment_thread.join(timeout=1.0)
        
        # 显示最终统计结果
        print_positioning_statistics()

def print_positioning_statistics():
    """打印定位时间统计结果"""
    global positioning_times
    
    if not positioning_times:
        print("*** 暂无定位测试数据 ***")
        return
    
    print("\n" + "="*50)
    print("*** 定位时间统计结果 ***")
    print(f"测试次数: {len(positioning_times)}")
    print(f"最短时间: {min(positioning_times):.3f}秒")
    print(f"最长时间: {max(positioning_times):.3f}秒")
    print(f"平均时间: {sum(positioning_times)/len(positioning_times):.3f}秒")
    print("\n详细记录:")
    for i, t in enumerate(positioning_times, 1):
        print(f"  测试 #{i}: {t:.3f}秒")
    print("="*50 + "\n")


def main():
    # Load configuration
    config = get_config()
    log_message("Configuration loaded successfully.")
    print("Configuration:", config)
    
    # 初始化摄像头
    cam_front, cam_down = initialize_cameras(config)
    if cam_front is None or cam_down is None:
        exit(1)

    # 初始化串口通信
    serial_comm = initialize_communication(config)
    if serial_comm is None:
        sys.exit(1)
    # 启动连续读取
    serial_comm.start_continuous_read(data_received_callback)

    # 初始化车辆和机械臂控制器
    vehicle, arm = initialize_controllers(config, serial_comm)

    # 初始化游戏控制器
    controller = control.PyGameController(vehicle, arm)
    controller.set_vehicle_control(vehicle)
    controller.set_arm_control(arm)

    detector = detect.YOLOv8(backend="openvino")
    qr_detector = detect.QrCodeDetector()
    
    # 启动定位时间测试
    track_target_with_timing(config, vehicle, cam_down, detector)
    
    return 0

if __name__ == "__main__":
    exit(main())
