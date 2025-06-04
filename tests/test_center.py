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
import numpy as np
import json
from datetime import datetime

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))

# 全局变量用于控制对准精度测试
alignment_active = False
alignment_start_time = 0
test_results = []
current_test_config = {}

# 像素到毫米的转换系数
PIXEL_TO_MM_RATIO = 0.5215  # mm/pixel

# 容差设置（像素）
TOLERANCE_PIXELS = 5


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


def calculate_pixel_distance(point1, point2):
    """计算两点之间的像素距离"""
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def is_within_tolerance(center_point, target_point, tolerance_pixels=TOLERANCE_PIXELS):
    """检查是否在容差范围内"""
    distance = calculate_pixel_distance(center_point, target_point)
    return distance <= tolerance_pixels, distance


def start_alignment_test(config, test_number):
    """开始对准测试"""
    global alignment_active, alignment_start_time, current_test_config
    
    current_test_config = config
    alignment_active = True
    alignment_start_time = time.time()
    
    log_message(f"开始测试: {config['name']}, 第{test_number}次")
    print(f"*** 开始测试: {config['name']}, 第{test_number}次 ***")
    print(f"初始偏移: X={config['offset_x']}mm, Y={config['offset_y']}mm")


def visual_servoing_alignment_test(config, vehicle, camera_down, detector):
    """
    视觉伺服对准精度测试系统
    """
    global alignment_active, alignment_start_time, test_results, current_test_config
    
    pixel_distance_ratio = config.get("vehicle", {}).get(
        "PIXEL_DISTANCE_RATIO_HEIGHT_0_OBJECT_1", 0.2848)
    
    # 图像中心作为目标点
    frame_size = (640, 480)
    target_point = (frame_size[0] // 2, frame_size[1] // 2)
    
    # 测试配置
    test_configurations = [
        {"name": "X_dir +20mm", "offset_x": 20, "offset_y": 0},
        {"name": "X_dir -20mm", "offset_x": -20, "offset_y": 0},
        {"name": "Y_dir +20mm", "offset_x": 0, "offset_y": 20},
        {"name": "Y_dir -20mm", "offset_x": 0, "offset_y": -20},
        # {"name": "对角线+10mm", "offset_x": 7.07, "offset_y": 7.07},
        # {"name": "对角线-10mm", "offset_x": -7.07, "offset_y": -7.07},
    ]
    
    current_config_index = 0
    current_test_number = 1
    max_tests_per_config = 10
    
    # 创建检测结果队列和控制事件
    detection_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    alignment_complete = threading.Event()
    
    # 稳定计数器（需要连续几帧都在容差内才算完成）
    stable_frames_required = 10
    stable_frame_count = 0
    
    def alignment_thread():
        """对准控制线程"""
        global alignment_active, alignment_start_time
        nonlocal stable_frame_count
        
        log_message("对准控制线程已启动")
        
        while not stop_event.is_set():
            if not alignment_active:
                time.sleep(0.1)
                continue
                
            try:
                detections = detection_queue.get(timeout=0.5)
                
                if not detections:
                    continue
                
                # 找到最佳检测目标（置信度最高的）
                best_detection = max(detections, key=lambda x: x['score'])
                
                # 计算物料中心
                box = best_detection['box']
                material_center = (int(box[0] + box[2] / 2), int(box[1] + box[3] / 2))
                
                # 检查是否在容差内
                within_tolerance, pixel_distance = is_within_tolerance(
                    material_center, target_point, TOLERANCE_PIXELS)
                
                if within_tolerance:
                    stable_frame_count += 1
                    if stable_frame_count >= stable_frames_required:
                        # 记录测试结果
                        end_time = time.time()
                        alignment_time = end_time - alignment_start_time
                        physical_distance = pixel_distance * PIXEL_TO_MM_RATIO
                        
                        result = {
                            'config_name': current_test_config['name'],
                            'test_number': current_test_number,
                            'initial_offset_x': current_test_config['offset_x'],
                            'initial_offset_y': current_test_config['offset_y'],
                            'final_pixel_deviation': pixel_distance,
                            'final_physical_deviation': physical_distance,
                            'alignment_time': alignment_time,
                            'within_tolerance': within_tolerance,
                            'timestamp': datetime.now().isoformat()
                        }
                        
                        test_results.append(result)
                        
                        log_message(f"对准完成！用时: {alignment_time:.3f}秒, "
                                  f"像素偏差: {pixel_distance:.2f}px, "
                                  f"物理偏差: {physical_distance:.3f}mm")
                        
                        alignment_active = False
                        alignment_complete.set()
                        stable_frame_count = 0
                else:
                    stable_frame_count = 0
                    # 执行位置调整
                    adjust_position_to_target(
                        target_class_id=None,
                        detections=detections,
                        pixel_distance_ratio=pixel_distance_ratio,
                        vehicle=vehicle,
                        frame_size=frame_size,
                        config=config
                    )
                    
            except queue.Empty:
                continue
            except Exception as e:
                log_message(f"对准线程异常: {e}", level="error")
                time.sleep(0.5)
        
        log_message("对准控制线程已结束")
    
    # 启动对准线程
    alignment_thread_obj = threading.Thread(target=alignment_thread)
    alignment_thread_obj.daemon = True
    alignment_thread_obj.start()
    
    log_message("视觉伺服对准精度测试系统已启动")
    print_test_instructions()
    
    try:
        while True:
            # 键盘控制
            if keyboard.is_pressed('space') and not alignment_active:
                if current_config_index < len(test_configurations):
                    start_alignment_test(test_configurations[current_config_index], 
                                        current_test_number)
                else:
                    log_message("所有测试配置已完成")
                    
            elif keyboard.is_pressed('n') and not alignment_active:
                # 下一个测试配置
                if current_test_number >= max_tests_per_config:
                    current_config_index += 1
                    current_test_number = 1
                else:
                    current_test_number += 1
                    
                time.sleep(0.5)  # 防止重复触发
                
            elif keyboard.is_pressed('r'):
                # 显示结果统计
                print_test_statistics()
                time.sleep(0.5)
                
            elif keyboard.is_pressed('s'):
                # 保存结果到文件
                save_test_results()
                time.sleep(0.5)
                
            elif keyboard.is_pressed('esc'):
                break
            
            # 图像处理和显示
            try:
                frame = camera_down.read_frame()
                if frame is None:
                    continue
                
                detections = detector.detect(frame)
                
                # 更新检测队列
                try:
                    if detection_queue.full():
                        detection_queue.get_nowait()
                    detection_queue.put_nowait(detections)
                except queue.Full:
                    pass
                
                # 可视化
                result_image = visualize_alignment_test(frame, detections, target_point, 
                                                      test_configurations, current_config_index, 
                                                      current_test_number, max_tests_per_config)
                
                cv2.imshow("Visual Servoing Alignment Test", result_image)
                cv2.waitKey(1)
                
                time.sleep(0.03)
                
            except Exception as e:
                log_message(f"图像处理异常: {e}", level="error")
                time.sleep(0.5)
                
    except KeyboardInterrupt:
        log_message("程序被中断")
    finally:
        cv2.destroyAllWindows()
        stop_event.set()
        alignment_thread_obj.join(timeout=1.0)
        print_final_statistics()

def visualize_alignment_test(frame, detections, target_point, test_configs, 
                           current_config_idx, current_test_num, max_tests):
    """可视化对准测试界面"""
    result_image = frame.copy()
    h, w = result_image.shape[:2]
    
    # 绘制目标点（图像中心）
    cv2.circle(result_image, target_point, 15, (0, 255, 255), 3)  # 黄色圆圈
    cv2.circle(result_image, target_point, TOLERANCE_PIXELS, (0, 255, 255), 2)  # 容差范围
    cv2.putText(result_image, "TARGET", (target_point[0] + 20, target_point[1]), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # 绘制检测结果
    for detection in detections:
        box = detection['box']
        x, y, w, h = box
        material_center = (int(x + w / 2), int(y + h / 2))
        
        # 绘制检测框
        cv2.rectangle(result_image, (int(x), int(y)), 
                     (int(x + w), int(y + h)), (0, 255, 0), 2)
        
        # 绘制物料中心
        cv2.circle(result_image, material_center, 8, (255, 0, 0), -1)  # 蓝色填充
        
        # 连接线和距离
        cv2.line(result_image, target_point, material_center, (255, 255, 255), 2)
        pixel_distance = calculate_pixel_distance(material_center, target_point)
        physical_distance = pixel_distance * PIXEL_TO_MM_RATIO
        
        # 检查是否在容差内
        within_tolerance = pixel_distance <= TOLERANCE_PIXELS
        color = (0, 255, 0) if within_tolerance else (0, 0, 255)
        
        # 显示距离信息
        dist_text = f"{pixel_distance:.1f}px ({physical_distance:.2f}mm)"
        cv2.putText(result_image, dist_text, (material_center[0] + 15, material_center[1] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 显示置信度
        conf_text = f"Conf: {detection['score']:.2f}"
        cv2.putText(result_image, conf_text, (int(x), int(y) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 绘制状态信息
    status_y = 30
    line_height = 25
    
    # 测试状态
    status_text = "ALIGNING..." if alignment_active else "READY"
    status_color = (0, 255, 255) if alignment_active else (255, 255, 255)
    cv2.putText(result_image, f"Status: {status_text}", (10, status_y), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    
    # 当前配置
    if current_config_idx < len(test_configs):
        config = test_configs[current_config_idx]
        cv2.putText(result_image, f"Config: {config['name']}", (10, status_y + line_height),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result_image, f"Test: {current_test_num}/{max_tests}", 
                   (10, status_y + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # 控制说明
    instructions = [
        "SPACE: Start test",
        "N: Next test",
        "R: Show results", 
        "S: Save results",
        "ESC: Exit"
    ]
    
    for i, instruction in enumerate(instructions):
        cv2.putText(result_image, instruction, (10, h - 120 + i*20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    # 显示测试进度
    if test_results:
        last_result = test_results[-1]
        result_text = f"Last: {last_result['final_pixel_deviation']:.1f}px, {last_result['alignment_time']:.2f}s"
        cv2.putText(result_image, result_text, (10, status_y + 3*line_height),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return result_image

def print_test_instructions():
    """打印测试说明"""
    print("\n" + "="*60)
    print("*** 视觉伺服对准精度测试系统 ***")
    print("="*60)
    print("操作说明:")
    print("  SPACE键 - 开始当前配置的测试")
    print("  N键     - 进入下一个测试")
    print("  R键     - 显示统计结果")
    print("  S键     - 保存结果到文件")
    print("  ESC键   - 退出程序")
    print("\n测试参数:")
    print(f"  目标容差: {TOLERANCE_PIXELS} 像素")
    print(f"  转换系数: {PIXEL_TO_MM_RATIO} mm/像素")
    print(f"  每组测试次数: 10次")
    print("="*60 + "\n")

def print_test_statistics():
    """打印测试统计结果"""
    if not test_results:
        print("*** 暂无测试数据 ***")
        return
    
    print("\n" + "="*80)
    print("*** 视觉伺服对准精度测试统计结果 ***")
    print("="*80)
    
    # 按配置分组统计
    config_groups = {}
    for result in test_results:
        config_name = result['config_name']
        if config_name not in config_groups:
            config_groups[config_name] = []
        config_groups[config_name].append(result)
    
    for config_name, results in config_groups.items():
        print(f"\n配置: {config_name}")
        print("-" * 60)
        
        pixel_deviations = [r['final_pixel_deviation'] for r in results]
        physical_deviations = [r['final_physical_deviation'] for r in results]
        alignment_times = [r['alignment_time'] for r in results]
        success_count = sum(1 for r in results if r['within_tolerance'])
        
        print(f"测试次数: {len(results)}")
        print(f"成功率: {success_count}/{len(results)} ({success_count/len(results)*100:.1f}%)")
        print(f"像素偏差 - 平均: {np.mean(pixel_deviations):.2f}px, "
              f"最大: {np.max(pixel_deviations):.2f}px, "
              f"最小: {np.min(pixel_deviations):.2f}px, "
              f"标准差: {np.std(pixel_deviations):.2f}px")
        print(f"物理偏差 - 平均: {np.mean(physical_deviations):.3f}mm, "
              f"最大: {np.max(physical_deviations):.3f}mm, "
              f"最小: {np.min(physical_deviations):.3f}mm, "
              f"标准差: {np.std(physical_deviations):.3f}mm")
        print(f"对准时间 - 平均: {np.mean(alignment_times):.2f}s, "
              f"最大: {np.max(alignment_times):.2f}s, "
              f"最小: {np.min(alignment_times):.2f}s, "
              f"标准差: {np.std(alignment_times):.2f}s")
        
        # 详细记录
        print("\n详细记录:")
        for i, result in enumerate(results, 1):
            status = "✓" if result['within_tolerance'] else "✗"
            print(f"  测试#{i}: {result['final_pixel_deviation']:.2f}px "
                  f"({result['final_physical_deviation']:.3f}mm), "
                  f"{result['alignment_time']:.2f}s {status}")
    
    print("="*80 + "\n")

def save_test_results():
    """保存测试结果到文件"""
    if not test_results:
        print("*** 无测试数据可保存 ***")
        return
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"alignment_test_results_{timestamp}.json"
    
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump({
                'test_parameters': {
                    'tolerance_pixels': TOLERANCE_PIXELS,
                    'pixel_to_mm_ratio': PIXEL_TO_MM_RATIO,
                    'target_point': [320, 240],  # 图像中心
                },
                'results': test_results
            }, f, indent=2, ensure_ascii=False)
        
        print(f"*** 测试结果已保存到: {filename} ***")
        
        # 同时保存CSV格式方便分析
        csv_filename = f"alignment_test_results_{timestamp}.csv"
        import csv
        
        with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['config_name', 'test_number', 'initial_offset_x', 'initial_offset_y',
                         'final_pixel_deviation', 'final_physical_deviation', 'alignment_time',
                         'within_tolerance', 'timestamp']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(test_results)
        
        print(f"*** CSV格式已保存到: {csv_filename} ***")
        
    except Exception as e:
        log_message(f"保存文件失败: {e}", level="error")

def print_final_statistics():
    """打印最终统计结果"""
    print_test_statistics()
    
    if test_results:
        # 生成表格格式的总结
        print("\n" + "="*100)
        print("*** 表4-10 视觉伺服对准精度测试 (目标：物料块中心) ***")
        print("="*100)
        
        # 表头
        print(f"{'初始偏移类型/距离':<20} {'测试次数':<8} {'最终像素偏差(px)':<15} "
              f"{'最终物理偏差(mm)':<15} {'对准时间(s)':<12} {'成功率(%)':<10}")
        print("-" * 100)
        
        # 按配置分组
        config_groups = {}
        for result in test_results:
            config_name = result['config_name']
            if config_name not in config_groups:
                config_groups[config_name] = []
            config_groups[config_name].append(result)
        
        for config_name, results in config_groups.items():
            pixel_deviations = [r['final_pixel_deviation'] for r in results]
            physical_deviations = [r['final_physical_deviation'] for r in results]
            alignment_times = [r['alignment_time'] for r in results]
            success_count = sum(1 for r in results if r['within_tolerance'])
            success_rate = success_count / len(results) * 100
            
            print(f"{config_name:<20} {len(results):<8} "
                  f"{np.mean(pixel_deviations):<15.2f} "
                  f"{np.mean(physical_deviations):<15.3f} "
                  f"{np.mean(alignment_times):<12.2f} "
                  f"{success_rate:<10.1f}")
        
        print("="*100)

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
    
    # 启动视觉伺服对准精度测试
    visual_servoing_alignment_test(config, vehicle, cam_down, detector)
    
    return 0

if __name__ == "__main__":
    exit(main())
