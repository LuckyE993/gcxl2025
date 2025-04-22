"""
应用程序主入口点。
"""

from src.config.settings import get_config
from src.utils.helpers import log_message
from src.core.communication import SerialCommunication
from src.core import control
from src.core import camera
from src.core import detect
import cv2
import time
import sys
from pathlib import Path

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))


positionFinish = False


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


def adjust_position_to_target(target_class_id, detections, pixel_distance_ratio, vehicle=None, frame_size=(640, 480)):
    """
    根据检测结果调整位置，使目标对象位于画面中心位置（差距在±5像素以内）

    Args:
        target_class_id (int): 目标物体的类别ID
        detections (List[Dict]): 检测结果列表
        pixel_distance_ratio (float): 像素距离到实际距离的比例系数
        vehicle (VehicleControl, optional): 用于控制车辆移动的对象
        frame_size (Tuple[int, int]): 画面尺寸 (宽, 高)

    Returns:
        bool: 如果目标已经位于中心（±5像素内）返回True，否则返回False
    """
    # 画面中心点
    center_x = frame_size[0] // 2
    center_y = frame_size[1] // 2

    # 像素误差容忍范围
    tolerance = 5

    # 寻找目标类别的物体
    target_found = False
    target_center_x = 0
    target_center_y = 0

    for detection in detections:
        if detection['class_id'] == target_class_id:
            # 获取边界框
            box = detection['box']
            x, y, w, h = box

            # 计算物体中心点
            target_center_x = x + w // 2
            target_center_y = y + h // 2
            target_found = True

            # 记录最高置信度的目标（如果有多个相同类别）
            break

    if not target_found:
        log_message(f"未找到目标类别 {target_class_id} 的物体")
        return False

    # 计算目标中心与画面中心的偏移量
    delta_x = target_center_x - center_x
    delta_y = target_center_y - center_y

    log_message(
        f"目标位置: ({target_center_x}, {target_center_y}), 中心位置: ({center_x}, {center_y})")
    log_message(f"偏移量: x={delta_x}px, y={delta_y}px")

    # 检查是否已经在容忍范围内
    if abs(delta_x) <= tolerance and abs(delta_y) <= tolerance:
        log_message("目标已经位于中心位置（±5像素范围内）")
        return True

    # 如果传入了车辆控制对象，计算实际距离并进行微调
    if vehicle is not None:
        # 将像素偏移转换为物理单位
        real_x_offset = delta_x * pixel_distance_ratio
        real_y_offset = delta_y * pixel_distance_ratio

        # 注意：根据实际坐标系统调整正负号
        # 假设: 画面 x+ 向右, y+ 向下, 车辆 x+ 向前, y+ 向左
        move_x = -real_y_offset  # 垂直方向可能需要换成前进/后退
        move_y = -real_x_offset  # 水平方向可能需要换成左/右移动

        log_message(f"调整位置: x={move_x:.2f}, y={move_y:.2f}")

        # 重置位置完成标志
        global positionFinish
        positionFinish = False

        # 发送位置调整命令
        vehicle.position_control(move_x, move_y, 0)  # 第三个参数是旋转角度，保持为0

        # 等待位置调整完成
        log_message("等待位置调整完成...")
        timeout = 20  # 设置超时时间（秒）
        start_time = time.time()

        while not positionFinish:
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                log_message("位置调整超时", level="warning")
                break

        if positionFinish:
            log_message("位置调整完成")
            # 位置调整完成后，重新检测目标位置
            return False  # 返回False以便在下一轮循环中重新检测位置

    return False


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
        time.sleep(0.1)
        if time.time() - start_time > timeout:
            log_message("位置调整超时", level="warning")
            return False

    log_message("位置调整完成")
    return True


class State:
    def execute(self, context):
        pass

# 具体状态类

class TestState(State):
    def execute(self, context):
        log_message("执行 TestState: 测试状态")
        time.sleep(1)
        if move_and_wait_for_completion(vehicle=context.vehicle, x=0, y=600, angle=0, timeout=30):
            log_message("测试状态1完成")
        time.sleep(0.5)
        if move_and_wait_for_completion(vehicle=context.vehicle, x=0, y=-600, angle=0, timeout=30):
            log_message("测试状态1完成")
            
        context.current_state = GrabAtRawMaterialState_Step4()

class ActionGroup1State(State):
    def execute(self, context):
        log_message("执行 动作组1: 出库移动")
        time.sleep(0.5)
        # 使用函数进行移动并等待完成
        if move_and_wait_for_completion(vehicle=context.vehicle, x=-230, y=175, angle=0, timeout=15):
            log_message("出库完成，开始扫码")

        else:
            log_message("出库执行失败", level="error")
        
        context.current_state = GrabAtRawMaterialState_Step4()


class ScanState(State):
    def execute(self, context):
        move_and_wait_for_completion(vehicle=context.vehicle, x=500, y=0, angle=0, timeout=15)
        
        mission = context.qr_detector.detect_from_camera(
            context.camera_front, display=False)

        # 获取原始码，并删除其中的加号
        raw_code = mission['raw_code']
        if raw_code is not None:
            # 删除所有加号
            context.scanned_data = raw_code.replace('+', '')
        else:
            context.scanned_data = None

        print(f"扫码结果: {context.scanned_data}")
        if context.scanned_data is None:
            log_message("扫码失败", level="error")
            return
        log_message(f"执行 识别1: 扫码完成，结果存入数组: {context.scanned_data}")
        context.current_state = MoveToRawMaterialAreaState()


class MoveToRawMaterialAreaState(State):
    def execute(self, context):
        log_message("执行 动作组2: 移动到原料区")

        # 设置机械臂高度到45
        log_message("设置机械臂高度到45")
        context.arm.arm_height_control(100)
        # 设置机械臂夹爪到0
        log_message("设置机械臂夹爪到打开")
        context.arm.arm_grab_control(0)
        time.sleep(1)  # 给机械臂一些时间执行命令

        log_message("动作组2完成，准备抓取原料")
        context.current_state = GrabAtRawMaterialState_Step4()


class GrabAtRawMaterialState_Step4(State):
    def execute(self, context):
        log_message("执行 动作组3 和 识别2: 原料区抓取")
        context.scanned_data = "123321"
        # 解析扫描数据，确定抓取顺序
        if not context.scanned_data or len(context.scanned_data) < 3:
            log_message("扫描数据不完整，无法确定抓取顺序", level="error")
            return

        # 获取抓取顺序（1=红色，2=绿色，3=蓝色）
        grab_order = context.scanned_data[:3]
        log_message(f"抓取顺序: {grab_order}")

        # 颜色ID映射
        color_map = {
            '1': {'name': '红色', 'class_id': 5},
            '2': {'name': '绿色', 'class_id': 4},
            '3': {'name': '蓝色', 'class_id': 3}
        }

        # 记录已抓取的物料
        grabbed_items = []

        # 按顺序抓取
        for color_code in grab_order:
            if color_code not in color_map:
                log_message(f"未知颜色代码: {color_code}", level="warning")
                continue

            color_info = color_map[color_code]
            log_message(f"准备抓取 {color_info['name']} 物料")
            context.arm.arm_height_control(180)
            # 物体稳定性检测
            stable_detections = self.wait_for_stable_object(
                context, color_info['class_id'],stability_threshold=2)

            if stable_detections:
                log_message(f"检测到稳定的 {color_info['name']} 物料，开始抓取")
                # 使用占位函数执行抓取
                grab(context=context, color=color_info['name'], platelist=grabbed_items)
                grabbed_items.append(color_info['name'])
            else:
                log_message(
                    f"未能检测到稳定的 {color_info['name']} 物料", level="warning")

        log_message("物料抓取完成，准备移动到粗加工区")
        context.current_state = MoveToRoughProcessingState_Step5()

    def wait_for_stable_object(self, context, target_class_id, stability_threshold=3):
        """
        等待目标物体稳定后返回检测结果

        Args:
            context: 执行上下文
            target_class_id: 目标物体的类别ID
            stability_threshold: 连续稳定帧数阈值

        Returns:
            稳定的检测结果或None
        """
        log_message(f"等待类别ID为 {target_class_id} 的物体稳定...")

        # 记录连续稳定的帧数
        stable_frames = 0

        # 上一帧的检测结果
        last_detection = None

        # 最大等待时间（秒）
        max_wait_time = 30
        start_time = time.time()

        while stable_frames < stability_threshold:
            # 检查是否超时
            if time.time() - start_time > max_wait_time:
                log_message("等待物体稳定超时", level="warning")
                return None

            # 获取一帧图像
            frame = context.camera_down.read_frame()
            if frame is None:
                log_message("无法读取图像帧", level="error")
                time.sleep(0.1)
                continue

            # 执行物体检测
            detections = context.detector.detect(frame)

            # 查找目标类别
            target_detection = None
            for detection in detections:
                if detection['class_id'] == target_class_id:
                    target_detection = detection
                    break

            if target_detection is None:
                log_message(f"未检测到类别ID为 {target_class_id} 的物体")
                stable_frames = 0
                time.sleep(0.1)
                continue

            # 如果是第一次检测到目标
            if last_detection is None:
                last_detection = target_detection
                stable_frames = 1
                continue

            # 计算与上一帧的位置差异
            current_box = target_detection['box']
            last_box = last_detection['box']

            # 计算中心点
            current_center_x = current_box[0] + current_box[2] // 2
            current_center_y = current_box[1] + current_box[3] // 2
            last_center_x = last_box[0] + last_box[2] // 2
            last_center_y = last_box[1] + last_box[3] // 2

            # 计算位置差异
            distance = ((current_center_x - last_center_x) ** 2 +
                        (current_center_y - last_center_y) ** 2) ** 0.5

            # 判断是否稳定（位置变化小于10像素）
            if distance < 10:
                stable_frames += 1
                log_message(f"物体稳定帧数: {stable_frames}/{stability_threshold}")
            else:
                stable_frames = 0
                log_message(f"物体不稳定，位置变化: {distance:.2f}px")

            last_detection = target_detection
            time.sleep(0.1)

        log_message("物体位置稳定，准备抓取")
        return last_detection

# 临时抓取函数，实际实现中需要替换为真正的抓取逻辑


def grab(context,color, platelist):
    """
    原料区抓取函数

    Args:
        color (str): 要抓取的颜色
        platelist (list): 已抓取物料列表
    """
    log_message(f"抓取 {color} 物料（占位函数）")
    # 放下机械臂
    context.arm.arm_height_control(370)
    time.sleep(1.5)
    # 夹爪闭合
    context.arm.arm_grab_control(1)
    time.sleep(0.8)
    # 机械臂抬起
    context.arm.arm_height_control(75)
    time.sleep(1.5)
    #机械臂旋转
    context.arm.arm_rotate_control(1)
    time.sleep(1)
    # 夹爪打开
    context.arm.arm_grab_control(0)
    time.sleep(0.8)
    # 机械臂抬起
    context.arm.arm_height_control(20)
    time.sleep(1)
    # 机械臂旋转
    context.arm.arm_rotate_control(0)
    time.sleep(0.8)
    # 转盘旋转
    context.plate_angle+=120
    context.arm.arm_plate_control(context.plate_angle)
    
    log_message(f"{color} 物料抓取完成")


class MoveToRoughProcessingState_Step5(State):
    def execute(self, context):
        log_message("执行 动作组4: 移动到粗加工区")
        context.current_state = AdjustPositionWithRingState_Step6()


class AdjustPositionWithRingState_Step6(State):
    def execute(self, context):
        log_message("执行 动作组5 和 识别2: 根据圆环矫正位置")
        # 解析扫描数据，确定抓取顺序
        if not context.scanned_data or len(context.scanned_data) < 3:
            log_message("扫描数据不完整，无法确定抓取顺序", level="error")
            return
        
        
        context.current_state = PlaceAtRoughProcessingState_Step7()


class PlaceAtRoughProcessingState_Step7(State):
    def execute(self, context):
        log_message("执行 动作组6: 粗加工区放置物料")
        context.current_state = GrabAtRoughProcessingState_Step8()


class GrabAtRoughProcessingState_Step8(State):
    def execute(self, context):
        log_message("执行 动作组7: 粗加工区抓取物料")
        context.current_state = MoveToPrecisionProcessingState_Step9()


class MoveToPrecisionProcessingState_Step9(State):
    def execute(self, context):
        log_message("执行 动作组8: 移动到精加工区")
        context.current_state = AdjustPositionWithRingState_Step10()


class AdjustPositionWithRingState_Step10(State):
    def execute(self, context):
        log_message("执行 动作组5 和 识别2: 根据圆环矫正位置")
        context.current_state = PlaceAtPrecisionProcessingState_Step11()


class PlaceAtPrecisionProcessingState_Step11(State):
    def execute(self, context):
        log_message("执行 动作组6: 精加工区放置物料")
        context.current_state = ReturnToRawMaterialAreaState_Step12()


class ReturnToRawMaterialAreaState_Step12(State):
    def execute(self, context):
        log_message("执行 动作组9: 返回原料区")
        context.current_state = GrabAtRawMaterialState_Step13()


class GrabAtRawMaterialState_Step13(State):
    def execute(self, context):
        log_message("执行 动作组3 和 识别2: 原料区抓取")
        context.current_state = MoveToRoughProcessingState_Step14()


class MoveToRoughProcessingState_Step14(State):
    def execute(self, context):
        log_message("执行 动作组4: 移动到粗加工区")
        context.current_state = AdjustPositionWithRingState_Step15()


class AdjustPositionWithRingState_Step15(State):
    def execute(self, context):
        log_message("执行 动作组5 和 识别2: 根据圆环矫正位置")
        context.current_state = PlaceAtRoughProcessingState_Step16()


class PlaceAtRoughProcessingState_Step16(State):
    def execute(self, context):
        log_message("执行 动作组6: 粗加工区放置物料")
        context.current_state = GrabAtRoughProcessingState_Step17()


class GrabAtRoughProcessingState_Step17(State):
    def execute(self, context):
        log_message("执行 动作组7: 粗加工区抓取物料")
        context.current_state = MoveToPrecisionProcessingState_Step18()


class MoveToPrecisionProcessingState_Step18(State):
    def execute(self, context):
        log_message("执行 动作组8: 移动到精加工区")
        context.current_state = AdjustPositionWithRingState_Step19()


class AdjustPositionWithRingState_Step19(State):
    def execute(self, context):
        log_message("执行 动作组5 和 识别2: 根据圆环矫正位置")
        context.current_state = StackPlaceAtPrecisionState_Step20()


class StackPlaceAtPrecisionState_Step20(State):
    def execute(self, context):
        log_message("执行 动作组10: 精加工区堆叠放置物料")
        context.current_state = ReturnToGarageState_Step21()


class ReturnToGarageState_Step21(State):
    def execute(self, context):
        log_message("执行 返回车库")
        context.current_state = None  # 结束状态机


class RobotContext:
    def __init__(self, vehicle=None, arm=None, communication=None, detector=None, qr_detector=None, camera_front=None, camera_down=None):
        self.camera_front = camera_front
        self.camera_down = camera_down
        self.vehicle = vehicle
        self.arm = arm
        self.communication = communication
        self.detector = detector
        self.qr_detector = qr_detector
        self.current_state = GrabAtRawMaterialState_Step4()
        self.plate_angle = 14
        self.scanned_data = []

    def run(self):
        self.arm.arm_plate_control(self.plate_angle)
        
        while self.current_state is not None:
            self.current_state.execute(self)


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

    context = RobotContext(vehicle, arm, serial_comm, detector,
                           qr_detector, camera_front=cam_front, camera_down=cam_down)
    context.run()

    return 0


if __name__ == "__main__":
    exit(main())
