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
# 比例控制参数 (0.5~0.8)
PROPORTIONAL_GAIN = 0.7
# 最大调整尝试次数
MAX_ATTEMPTS = 5  
PIXEL_DISTANCE_RATIO_HEIGHT_0 = 0.5  # 像素到物理距离的比例系数，需要实际校准

def initialize_cameras(config):
    """初始化摄像头"""
    log_message("正在初始化摄像头...")
    cam_front = camera.Camera(config["camera"]["camera_id_front"], (640, 480))
    cam_down = camera.Camera(config["camera"]["camera_id_down"], (640, 480))

    try:
        # Initialize front camera
        if cam_front.open():
            log_message(f"cam_front properties: {cam_front.get_properties()}")
        else:
            log_message("无法打开正面摄像头", level="warning")

        if cam_down.open():
            log_message(f"cam_down properties: {cam_down.get_properties()}")
        else:
            log_message("无法打开底部摄像头", level="warning")

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
        move_and_wait_for_completion(
            vehicle=context.vehicle, x=500, y=0, angle=0, timeout=15)

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
                context, color_info['class_id'], stability_threshold=2)

            if stable_detections:
                log_message(f"检测到稳定的 {color_info['name']} 物料，开始抓取")
                # 使用占位函数执行抓取
                control.grabAtRawArea(
                    context=context, color=color_info['name'], platelist=grabbed_items)
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
        # 尝试进行位置调整
        attempts = 0
        adjusted = False

        while attempts < MAX_ATTEMPTS and not adjusted:
            log_message(f"位置调整尝试 {attempts + 1}/{MAX_ATTEMPTS}")
            
            # 获取当前图像
            frame = context.camera_down.read_frame()
            if frame is None:
                log_message("无法读取图像", level="error")
                time.sleep(0.5)
                attempts += 1
                continue
                
            # 执行目标检测
            detections = context.detector.detect(frame)
            
            # 显示检测结果（调试用）
            detect_img = context.detector.draw_detections(frame.copy(), detections)
            cv2.imshow("Position Adjustment", detect_img)
            cv2.waitKey(1)
            
            # 根据检测结果调整位置
            adjusted = adjust_position_to_target(
                detections, 
                PIXEL_DISTANCE_RATIO_HEIGHT_0,
                vehicle=context.vehicle
            )
            
            if adjusted:
                log_message("位置调整成功")
                time.sleep(1.0)  # 稳定等待
                break
                
            attempts += 1
            time.sleep(0.5)  # 短暂等待后重试

        if not adjusted:
            log_message("无法完成位置调整，继续执行后续步骤", level="warning")

        cv2.destroyAllWindows()
        context.current_state = PlaceAtRoughProcessingState_Step7()


def adjust_position_to_target(target_class_id=None, detections=None, pixel_distance_ratio=1.0, vehicle=None, 
                              frame_size=(640, 480), target_position=None):
    """
    改进版：增加比例控制、迭代调整和容错机制
    支持指定目标点位或默认使用画面中心点
    支持不指定目标类别时自动选择离目标点最近的检测物体
    
    Args:
        target_class_id: 目标物体的类别ID，不指定则选择最靠近目标点的物体
        detections: 检测到的物体列表
        pixel_distance_ratio: 像素距离到实际距离的转换比例
        vehicle: 车辆控制对象
        frame_size: 画面尺寸，默认(640, 480)
        target_position: 可选的目标位置(x, y)，不指定则使用画面中心
        
    Returns:
        bool: 调整完成返回True，否则返回False
    """
    # 检查是否有检测结果
    if not detections:
        log_message("没有检测到任何物体")
        return False
    
    # 设置目标位置（指定位置或画面中心）
    if target_position is None:
        # 画面中心点
        target_x = frame_size[0] // 2
        target_y = frame_size[1] // 2
    else:
        target_x, target_y = target_position
    
    tolerance = 5  # 像素误差容忍范围

    # 如果没有指定目标类别ID，选择离目标点最近的物体
    if target_class_id is None:
        min_distance = float('inf')
        target = None
        
        for detection in detections:
            x, y, w, h = detection['box']
            center_x = x + w//2
            center_y = y + h//2
            
            # 计算到目标点的距离
            distance = ((center_x - target_x) ** 2 + (center_y - target_y) ** 2) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                target = detection
                
        if not target:
            log_message("未找到合适的目标物体")
            return False
            
        log_message(f"自动选择了离目标点最近的物体 (类别ID: {target['class_id']})")
    else:
        # 按指定类别ID寻找目标物体
        target = next((d for d in detections if d['class_id'] == target_class_id), None)
        if not target:
            log_message(f"目标 {target_class_id} 未检测到")
            return False

    # 计算目标中心
    x, y, w, h = target['box']
    target_center = (x + w//2, y + h//2)
    delta_x = target_center[0] - target_x
    delta_y = target_center[1] - target_y

    # 检查是否满足条件
    if abs(delta_x) <= tolerance and abs(delta_y) <= tolerance:
        log_message("目标已到达指定位置")
        return True

    # 渐进式调整逻辑
    if vehicle:
        # 计算带比例系数的物理偏移量
        real_x = delta_x * pixel_distance_ratio * PROPORTIONAL_GAIN
        real_y = delta_y * pixel_distance_ratio * PROPORTIONAL_GAIN

        # 运动方向映射（根据实际坐标系调整）
        move_x = -real_x  # 前后移动量（假设y轴对应深度方向）
        move_y = -real_y  # 左右移动量

        # 执行渐进式移动
        log_message(f"渐进调整: X={move_x:.2f}, Y={move_y:.2f}")
        # vehicle.position_control(move_y, move_x, 0)  # 注意参数顺序可能需要调整
        move_and_wait_for_completion(vehicle=vehicle, x=move_y, y=move_x, angle=0, timeout=5)
        # 添加稳定等待时间（根据实际运动速度调整）
        time.sleep(0.2)  # 示例值，需实际测试调整

    return False  # 需要继续调整


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

    detector = detect.YOLOv8(backend="onnx")
    qr_detector = detect.QrCodeDetector()
    
    vehicle.position_control(x=0,y=60,yaw=0)
    time.sleep(1)
    vehicle.position_control(x=0,y=-60,yaw=0)
    time.sleep(1)
    # 增加连续位置调整的测试功能
    log_message("开始连续位置调整测试...")
    
    try:
        # 确保摄像头已打开
        if not cam_down :
            log_message("底部摄像头未打开，无法执行位置调整", level="error")
        else:
            # 设置调整参数
            target_class_id = None  # 不指定类别ID，选择最近的目标
            adjustment_running = True
            
            log_message("开始连续位置调整，按ESC键停止...")

            camera.start_camera_thread(cam_down)
            while adjustment_running:
                # Read current frame
                frame = camera.get_latest_frame()
                if frame is None:
                    log_message("无法读取图像帧", level="warning")
                    time.sleep(0.1)  # Add small delay to avoid tight loop
                    continue
                
                try:
                    # Perform object detection
                    detections = detector.detect(frame)
                    
                    # Show detection results - use copy() to avoid modifying original frame
                    detect_img = detector.visualize_detections(frame.copy(), detections)
                    cv2.imshow("Continuous Position Adjustment", detect_img)
                    
                    # Perform position adjustment
                    adjust_position_to_target(
                        target_class_id=target_class_id,
                        detections=detections,
                        pixel_distance_ratio=PIXEL_DISTANCE_RATIO_HEIGHT_0,
                        vehicle=vehicle
                    )
                    
                    # Check for ESC key to exit - increase wait time for smoother UI
                    key = cv2.waitKey(20)
                    if key == 27:  # ESC key
                        log_message("用户中断位置调整")
                        camera.stop_camera_thread(cam_down)
                        adjustment_running = False
                except cv2.error as e:
                    log_message(f"显示异常: {e}", level="error")
                    adjustment_running = False
            cv2.destroyAllWindows()
            
    except Exception as e:
        log_message(f"连续位置调整异常: {e}", level="error")
        cv2.destroyAllWindows()
    
    # context = RobotContext(vehicle, arm, serial_comm, detector,
    #                        qr_detector, camera_front=cam_front, camera_down=cam_down)
    # context.run()

    return 0


if __name__ == "__main__":
    exit(main())
