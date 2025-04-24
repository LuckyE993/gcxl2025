from time import sleep
from src.core.communication import *
from src.utils import helpers
from src.config import settings
import pygame
import os
import sys


class VehicleControl:
    def __init__(self, config=None):
        """Initialize vehicle control parameters"""
        if config is None:
            config = settings.get_config()

        # Load vehicle parameters from config
        self.max_speed = config.get("vehicle", {}).get(
            "max_speed", 1000)  # Default max speed: 1000 mm/s
        self.max_yaw_rate = config.get("vehicle", {}).get(
            "max_yaw_rate", 180)  # Default max yaw rate: 180 deg/s
        self.comm = None
        log_message(
            f"Vehicle control initialized with max speed: {self.max_speed} mm/s, max yaw rate: {self.max_yaw_rate} deg/s")

    def set_communication(self, comm):
        """Set the communication interface"""
        self.comm = comm

    def position_control(self, x, y, yaw):
        """
        Position control loop
        Args:
            x (int): Target x position in mm (16-bit signed integer)
            y (int): Target y position in mm (16-bit signed integer)
            yaw (int): Target yaw angle in degrees (16-bit signed integer)
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        log_message(f"Position control: x={x}mm, y={y}mm, yaw={yaw}°")

        # Create payload for position control
        # Format: Command type (0x01 0x01) + x (2 bytes) + y (2 bytes) + yaw (2 bytes)
        try:
            # Convert command type to 2 bytes
            cmd_bytes = bytes([0x01, 0x01])

            # Convert x, y, yaw to 16-bit signed integers (2 bytes each)
            # Using two's complement representation
            x_int = int(x)
            y_int = int(y)
            yaw_int = int(yaw)

            # Ensure values are within 16-bit signed integer range (-32768 to 32767)
            x_int = max(min(x_int, 32767), -32768)
            y_int = max(min(y_int, 32767), -32768)
            yaw_int = max(min(yaw_int, 32767), -32768)

            # Convert to 2-byte representation (big endian)
            x_bytes = x_int.to_bytes(2, byteorder='big', signed=True)
            y_bytes = y_int.to_bytes(2, byteorder='big', signed=True)
            yaw_bytes = yaw_int.to_bytes(2, byteorder='big', signed=True)

            # Build the complete payload
            payload = cmd_bytes + x_bytes + y_bytes + yaw_bytes

            # Verify payload length is 8 bytes
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            return send_protocol_message(self.comm, payload)
        except Exception as e:
            log_message(f"Position control failed: {str(e)}", level="error")
            return False

    def velocity_control(self, vx, vy, vyaw):
        """
        Velocity control loop
        Args:
            vx (int): Target x velocity in mm/s (16-bit signed integer)
            vy (int): Target y velocity in mm/s (16-bit signed integer)
            vyaw (int): Target yaw rate in degrees/s (16-bit signed integer)
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        # Apply speed limits
        vx_int = int(max(min(vx, self.max_speed), -self.max_speed))
        vy_int = int(max(min(vy, self.max_speed), -self.max_speed))
        vyaw_int = int(max(min(vyaw, self.max_yaw_rate), -self.max_yaw_rate))

        # Ensure values are within 16-bit signed integer range (-32768 to 32767)
        vx_int = max(min(vx_int, 32767), -32768)
        vy_int = max(min(vy_int, 32767), -32768)
        vyaw_int = max(min(vyaw_int, 32767), -32768)

        log_message(
            f"Velocity control: vx={vx_int}mm/s, vy={vy_int}mm/s, vyaw={vyaw_int}°/s")

        # Create payload for velocity control
        # Format: Command type (0x02 0x01) + vx (2 bytes) + vy (2 bytes) + vyaw (2 bytes)
        try:
            # Convert command type to 2 bytes
            cmd_bytes = bytes([0x01, 0x00])

            # Convert to 2-byte representation (big endian)
            vx_bytes = vx_int.to_bytes(2, byteorder='big', signed=True)
            vy_bytes = vy_int.to_bytes(2, byteorder='big', signed=True)
            vyaw_bytes = vyaw_int.to_bytes(2, byteorder='big', signed=True)

            # Build the complete payload
            payload = cmd_bytes + vx_bytes + vy_bytes + vyaw_bytes

            # Verify payload length is 8 bytes
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            return send_protocol_message(self.comm, payload)
        except Exception as e:
            log_message(f"Velocity control failed: {str(e)}", level="error")
            return False


class ArmControl:
    def __init__(self, config=None):
        """Initialize arm control parameters"""
        if config is None:
            config = settings.get_config()

        # Load arm parameters from config
        self.max_height = config.get("arm", {}).get(
            "max_height", 400)  # Default max height: 400 mm
        self.rotate_max_height = config.get("arm", {}).get(
            "rotate_max_height", 300)  # Default max height for rotation: 300 mm
        self.backward_height_limit = config.get("arm", {}).get(
            "backward_height_limit", 200)  # 朝后时的最大高度限制: 200 mm
        self.comm = None
        self.arm_height = 0  # Current arm height
        self.arm_grab = 0
        self.arm_plate_angle = 0
        self.arm_rotate = 0
        log_message(
            f"Arm control initialized with max height: {self.max_height} mm")

    def set_communication(self, comm):
        """Set the communication interface"""
        self.comm = comm

    def initialize(self):
        """
        初始化机械臂到基准位置
        - 高度设置为0
        - 方向朝前
        - 夹爪打开
        - 旋转盘角度为0

        Returns:
            bool: 所有操作成功返回True，任一操作失败返回False
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        log_message("正在初始化机械臂到基准位置...")
        success = True

        # 先将机械臂高度设置为0（安全起见，先降低高度）
        if not self.arm_height_control(0):
            log_message("机械臂高度初始化失败", level="error")
            success = False
        sleep(1)
        # 设置方向朝前
        if not self.arm_rotate_control(0):
            log_message("机械臂方向初始化失败", level="error")
            success = False
        sleep(0.5)
        # 设置夹爪打开
        if not self.arm_grab_control(0):
            log_message("机械臂夹爪初始化失败", level="error")
            success = False
        sleep(0.5)
        # 设置旋转盘角度为0
        if not self.arm_plate_control(0):
            log_message("机械臂旋转盘初始化失败", level="error")

            success = False
        sleep(0.5)
        if success:
            log_message("机械臂初始化完成，已设置到基准位置")
        else:
            log_message("机械臂部分初始化失败，请检查连接和设备状态", level="warning")

        return success

    def arm_height_control(self, height):
        """
        控制机械臂高度

        Args:
            height (int): 目标高度，单位mm，范围[0, max_height]

        Returns:
            bool: 成功返回True，失败返回False
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        # 根据方向应用高度限制
        if self.arm_rotate == 1:  # 朝后
            max_allowed_height = min(self.max_height, self.rotate_max_height)
            if height > self.rotate_max_height:
                log_message(f"朝后方向高度受限: 请求高度 {height}mm 超过后向限制 {self.rotate_max_height}mm", level="warning")
        else:  # 朝前
            max_allowed_height = self.max_height

        # 限制高度范围
        height_val = max(0, min(int(height), max_allowed_height))
        
        log_message(f"Arm height control: height={height_val}mm")

        try:
            # 命令类型: 0x02 0x01 (机械臂高度控制)
            cmd_bytes = bytes([0x02, 0x01])

            # 将高度值转换为2字节（高位在前）
            height_bytes = height_val.to_bytes(
                2, byteorder='big', signed=False)

            # 剩余4个字节为0
            zero_bytes = bytes([0x00, 0x00, 0x00, 0x00])

            # 构建完整payload: 命令类型(2字节) + 高度值(2字节) + 0填充(4字节)
            payload = cmd_bytes + height_bytes + zero_bytes

            # 验证payload长度为8字节
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            # 保存当前高度值
            self.arm_height = height_val

            # 发送控制命令
            return send_protocol_message(self.comm, payload)

        except Exception as e:
            log_message(f"Arm height control failed: {str(e)}", level="error")
            return False

    def arm_grab_control(self, state):
        """
        控制机械臂夹爪开合（舵机2）

        Args:
            state (int): 夹爪状态
                0: 打开
                1: 闭合
                2: 开到最大

        Returns:
            bool: 成功返回True，失败返回False
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        # 验证状态值
        if state not in [0, 1, 2]:
            log_message(
                f"无效的夹爪状态: {state}，必须是0(打开)、1(闭合)或2(最大开启)", level="error")
            return False

        log_message(
            f"机械臂夹爪控制: state={state} ({'打开' if state == 0 else '闭合' if state == 1 else '最大开启'})")

        # 保存当前夹爪状态
        self.arm_grab = state

        try:
            # 命令类型: 0x02 0x00 (机械臂步进舵机控制)
            cmd_bytes = bytes([0x02, 0x00])

            # 步进电机1位置（假设为0，或使用已存储的值）
            stepper1_bytes = bytes([0x00, 0x00])

            # 步进电机2（旋转盘）位置
            stepper2_bytes = self.arm_plate_angle.to_bytes(
                2, byteorder='big', signed=False)

            # 舵机1（机械臂旋转）和舵机2（夹爪开合）
            servo1_byte = bytes([self.arm_rotate])
            servo2_byte = bytes([state])

            # 构建完整payload
            payload = cmd_bytes + stepper1_bytes + \
                stepper2_bytes + servo1_byte + servo2_byte

            # 验证payload长度为8字节
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            # 发送控制命令
            return send_protocol_message(self.comm, payload)

        except Exception as e:
            log_message(f"机械臂夹爪控制失败: {str(e)}", level="error")
            return False

    def arm_rotate_control(self, direction):
        """
        控制机械臂旋转方向（舵机1）

        Args:
            direction (int): 旋转方向
                0: 朝前
                1: 朝后

        Returns:
            bool: 成功返回True，失败返回False
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        # 验证方向值
        if direction not in [0, 1]:
            log_message(f"无效的旋转方向: {direction},必须是0(朝前)或1(朝后)", level="error")
            return False
        # Verify height value
        if self.arm_height > self.rotate_max_height:
            log_message(
                f"机械臂高度过高: {self.arm_height}mm,最大高度为: {self.rotate_max_height},无法旋转", level="error")
            return False

        log_message(
            f"机械臂旋转控制: direction={direction} ({'朝前' if direction == 0 else '朝后'})")

        # 保存当前旋转方向
        self.arm_rotate = direction

        try:
            # 命令类型: 0x02 0x00 (机械臂步进舵机控制)
            cmd_bytes = bytes([0x02, 0x00])

            # 步进电机1位置（假设为0，或使用已存储的值）
            stepper1_bytes = bytes([0x00, 0x00])

            # 步进电机2（旋转盘）位置
            stepper2_bytes = self.arm_plate_angle.to_bytes(
                2, byteorder='big', signed=False)

            # 舵机1（机械臂旋转）和舵机2（夹爪开合）
            servo1_byte = bytes([direction])
            servo2_byte = bytes([self.arm_grab])

            # 构建完整payload
            payload = cmd_bytes + stepper1_bytes + \
                stepper2_bytes + servo1_byte + servo2_byte

            # 验证payload长度为8字节
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            # 发送控制命令
            return send_protocol_message(self.comm, payload)

        except Exception as e:
            log_message(f"机械臂旋转控制失败: {str(e)}", level="error")
            return False

    def arm_plate_control(self, angle):
        """
        控制机械臂旋转盘角度（步进电机2）

        Args:
            angle (int): 旋转盘角度，范围[0, 359]度

        Returns:
            bool: 成功返回True，失败返回False
        """
        if not self.comm:
            log_message("Communication interface not set", level="error")
            return False

        # 限制角度范围
        angle_val = max(0, min(int(angle), 359))

        log_message(f"机械臂旋转盘控制: angle={angle_val}°")

        # 保存当前旋转盘角度
        self.arm_plate_angle = angle_val

        try:
            # 命令类型: 0x02 0x00 (机械臂步进舵机控制)
            cmd_bytes = bytes([0x02, 0x00])

            # 步进电机1位置（假设为0，或使用已存储的值）
            stepper1_bytes = bytes([0x00, 0x00])

            # 步进电机2（旋转盘）位置
            stepper2_bytes = angle_val.to_bytes(
                2, byteorder='big', signed=False)

            # 舵机1（机械臂旋转）和舵机2（夹爪开合）
            servo1_byte = bytes([self.arm_rotate])
            servo2_byte = bytes([self.arm_grab])

            # 构建完整payload
            payload = cmd_bytes + stepper1_bytes + \
                stepper2_bytes + servo1_byte + servo2_byte

            # 验证payload长度为8字节
            if len(payload) != 8:
                raise ValueError(
                    f"Invalid payload length: {len(payload)}, should be 8 bytes")

            # 发送控制命令
            return send_protocol_message(self.comm, payload)

        except Exception as e:
            log_message(f"机械臂旋转盘控制失败: {str(e)}", level="error")
            return False


def grabAtRawArea(context,color):
    """
    原料区抓取函数

    Args:
        color (str): 要抓取的颜色
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

class PyGameController:
    def __init__(self, vehicle_control=None, arm_control=None):
        """Initialize PyGame controller

        Args:
            vehicle_control (VehicleControl, optional): 车辆控制对象
            arm_control (ArmControl, optional): 机械臂控制对象
        """
        pygame.init()
        pygame.joystick.init()

        self.vehicle = vehicle_control
        self.arm = arm_control
        self.joysticks = []
        self.active_joystick = None
        self.running = False
        self.thread = None

        # 控制模式
        self.arm_control_mode = False  # 默认为底盘控制模式
        self.arm_height_increment = 5  # 每次按下Trigger机械臂高度变化值(mm)
        self.arm_grab_state = 0  # 夹爪状态：0=打开，1=闭合

        # 控制参数
        self.deadzone = 0.1  # 摇杆死区
        self.max_speed = 1000  # 最大速度 mm/s
        self.max_rotation = 180  # 最大旋转速度 deg/s

        # 根据平台设置按钮和轴映射
        self._setup_platform_mappings()

        # 初始化可用的手柄
        self._init_joysticks()
        
    def _setup_platform_mappings(self):
        """根据配置文件设置控制器映射关系"""
        from src.config import settings
        
        # 获取配置
        config = settings.get_config()
        controller_config = config.get("controller", {})
        
        # 获取平台名称，默认为系统平台名称
        platform = controller_config.get("platform", None)
        
        # 如果配置中没有指定平台，则根据系统自动判断
        if not platform:
            import sys
            if sys.platform.startswith('win'):
                platform = "win"
            else:
                platform = "ubuntu"  # 默认使用Ubuntu映射
        
        # 获取对应平台的映射配置
        platform_config = controller_config.get(platform, {})
        
        log_message(f"使用'{platform}'平台的控制器映射配置")
        
        # 设置轴映射
        self.axis_map = {
            "lx": platform_config.get("left_stick_x", 1),
            "ly": platform_config.get("left_stick_y", 0),
            "rx": platform_config.get("right_stick_x", 3 if platform != "win" else 2),
            "lt": platform_config.get("left_trigger", 2 if platform != "win" else 4),
            "rt": platform_config.get("right_trigger", 5)
        }
        
        # 设置按钮映射
        self.button_map = {
            "forward": platform_config.get("left_bumper", 4 if platform != "win" else 9),
            "backward": platform_config.get("right_bumper", 5 if platform != "win" else 10),
            "grab": platform_config.get("a_button", 0),
            "plate": platform_config.get("x_button", 2 if platform == "ubuntu" else 3),
            "mode": platform_config.get("y_button", 3 if platform == "ubuntu" else 2),
            "mode_back": platform_config.get("b_button", 1)
        }
        
        log_message(f"控制器轴映射: {self.axis_map}")
        log_message(f"控制器按钮映射: {self.button_map}")

    def _init_joysticks(self):
        """初始化所有可用的手柄"""
        self.joysticks = [pygame.joystick.Joystick(
            i) for i in range(pygame.joystick.get_count())]

        if self.joysticks:
            log_message(f"检测到 {len(self.joysticks)} 个游戏控制器")
            for i, joystick in enumerate(self.joysticks):
                joystick.init()
                log_message(f"控制器 {i}: {joystick.get_name()}")
            self.active_joystick = self.joysticks[0]
            log_message(f"已激活控制器: {self.active_joystick.get_name()}")
        else:
            log_message("未检测到游戏控制器", level="warning")

    def set_vehicle_control(self, vehicle_control):
        """设置车辆控制对象

        Args:
            vehicle_control (VehicleControl): 车辆控制对象
        """
        self.vehicle = vehicle_control
        if self.vehicle:
            # 获取车辆参数
            self.max_speed = self.vehicle.max_speed
            self.max_rotation = self.vehicle.max_yaw_rate

    def set_arm_control(self, arm_control):
        """设置机械臂控制对象

        Args:
            arm_control (ArmControl): 机械臂控制对象
        """
        self.arm = arm_control

    def start(self):
        """启动控制器监听线程"""
        if not self.joysticks:
            log_message("未检测到游戏控制器，无法启动", level="error")
            return False

        if not self.vehicle:
            log_message("未设置车辆控制对象，无法启动", level="error")
            return False

        if self.running:
            return True

        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        log_message("游戏控制器监听已启动")
        return True

    def stop(self):
        """停止控制器监听线程"""
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            log_message("游戏控制器监听已停止")

            # 停止车辆运动
            if self.vehicle:
                self.vehicle.velocity_control(0, 0, 0)

    def _control_loop(self):
        """控制循环"""
        clock = pygame.time.Clock()

        # 按钮状态存储（防止重复触发）
        button_states = {}

        while self.running:
            # 处理pygame事件
            pygame.event.pump()

            if not self.active_joystick:
                time.sleep(0.1)
                continue

            try:
                # 检测模式切换按钮
                if not self.arm:
                    return
    
                mode_button_pressed = self.active_joystick.get_button(self.button_map["mode"])
                if mode_button_pressed and not button_states.get("mode_button", False) and not self.arm_control_mode:
                    if self.arm:
                        self.arm_control_mode = True
                        log_message("已切换到机械臂控制模式")
                    else:
                        log_message("未设置机械臂控制对象，无法切换到机械臂控制模式", level="warning")
                button_states["mode_button"] = mode_button_pressed

                # mode_back_button - 退出机械臂控制模式
                mode_back_button_pressed = self.active_joystick.get_button(self.button_map["mode_back"])
                if mode_back_button_pressed and not button_states.get("mode_back_button", False) and self.arm_control_mode:
                    self.arm_control_mode = False
                    log_message("已切换回底盘控制模式")
                button_states["mode_back_button"] = mode_back_button_pressed

                if self.arm_control_mode:
                    # 机械臂控制模式
                    self._handle_arm_control(button_states)
                else:
                    # 底盘控制模式
                    self._handle_vehicle_control()

            except Exception as e:
                log_message(f"控制器处理异常: {str(e)}", level="error")

            # 控制循环频率
            clock.tick(15)  # 15Hz

    def _handle_vehicle_control(self):
        """处理底盘控制逻辑"""
        if not self.vehicle:
            return
        # 底盘设置xy与手柄相反    
        steer_dir_y = self.active_joystick.get_axis(self.axis_map["lx"])*-1
        steer_dir_x = self.active_joystick.get_axis(self.axis_map["ly"])*-1
        steer_rotation_x = self.active_joystick.get_axis(self.axis_map["rx"])*-1
        left_trigger = self.active_joystick.get_axis(self.axis_map["lt"])
        right_trigger = self.active_joystick.get_axis(self.axis_map["rt"])
    
        # 应用死区
        steer_dir_x = 0 if abs(steer_dir_x) < self.deadzone else steer_dir_x
        steer_dir_y = 0 if abs(steer_dir_y) < self.deadzone else steer_dir_y
        steer_rotation_x = 0 if abs(steer_rotation_x) < self.deadzone else steer_rotation_x
    
        # 计算运动控制值
        # 将扳机值从[-1,1]映射到[0,1]
        left_power = ((left_trigger + 1) / 2) * -1
        right_power = (right_trigger + 1) / 2
    
        vx = steer_dir_x * (left_power+right_power) * self.max_speed  # 前进速度
        vy = steer_dir_y * (left_power+right_power) * self.max_speed  # 侧向速度
        vyaw = steer_rotation_x * (left_power+right_power) * self.max_rotation  # 旋转速度
    
        # 发送控制命令
        self.vehicle.velocity_control(int(vx), int(vy), int(vyaw))

    def _handle_arm_control(self, button_states):
        """处理机械臂控制逻辑

        Args:
            button_states (dict): 按钮状态字典，用于防止重复触发
        """
        if not self.arm:
            return

        # 左右扳机控制机械臂高度
        left_trigger = self.active_joystick.get_axis(self.axis_map["lt"])  # 范围从-1到1
        right_trigger = self.active_joystick.get_axis(self.axis_map["rt"])  # 范围从-1到1

        # 将扳机值从[-1,1]映射到[0,1]
        left_power = (left_trigger + 1) / 2
        right_power = (right_trigger + 1) / 2

        # 只有当扳机值大于0.2才触发高度变化，防止误触
        if left_power > 0.2:
            # 左扳机下降
            new_height = max(0, self.arm.arm_height - self.arm_height_increment)
            if new_height != self.arm.arm_height:
                log_message(f"机械臂高度下降: {self.arm.arm_height} -> {new_height}")
                self.arm.arm_height_control(new_height)

        if right_power > 0.2:
            # 右扳机上升
            new_height = min(self.arm.max_height, self.arm.arm_height + self.arm_height_increment)
            if new_height != self.arm.arm_height:
                log_message(f"机械臂高度上升: {self.arm.arm_height} -> {new_height}")
                self.arm.arm_height_control(new_height)

        # 控制旋转盘旋转120°
        plate_rotate_pressed = self.active_joystick.get_button(self.button_map["plate"])
        if plate_rotate_pressed and not button_states.get("plate_rotate", False):
            # 计算新的角度值（当前角度加上120°，并取模确保在0-359范围内）
            new_angle = (self.arm.arm_plate_angle + 120) % 360
            log_message(f"旋转盘旋转: {self.arm.arm_plate_angle}° -> {new_angle}°")
            self.arm.arm_plate_control(new_angle)
        button_states["plate_rotate"] = plate_rotate_pressed

        # 控制机械臂旋转方向
        forward_pressed = self.active_joystick.get_button(self.button_map["forward"])
        backward_pressed = self.active_joystick.get_button(self.button_map["backward"])

        # 确保按钮状态有记录，防止第一次触发错误
        if "forward" not in button_states:
            button_states["forward"] = False
        if "backward" not in button_states:
            button_states["backward"] = False

        if forward_pressed and not button_states.get("forward", False):
            # 机械臂朝后
            self.arm.arm_rotate_control(1)
        button_states["forward"] = forward_pressed

        if backward_pressed and not button_states.get("backward", False):
            # 机械臂朝前
            self.arm.arm_rotate_control(0)
        button_states["backward"] = backward_pressed

        # 控制夹爪抓取或松开（切换状态）
        grab_pressed = self.active_joystick.get_button(self.button_map["grab"])
        if grab_pressed and not button_states.get("grab", False):
            # 切换夹爪状态
            self.arm_grab_state = 1 if self.arm_grab_state == 0 else 0
            self.arm.arm_grab_control(self.arm_grab_state)
            log_message(f"夹爪状态: {'闭合' if self.arm_grab_state == 1 else '打开'}")
        button_states["grab"] = grab_pressed


def keyboard_test(vehicle: VehicleControl):
    """Test vehicle control with keyboard input"""
    # 读取键盘输入
    key = input("请输入控制命令 (w/a/s/d/q): ").strip().lower()

    if key == 'w':
        vehicle.velocity_control(1000, 0, 0)
    elif key == 's':
        vehicle.velocity_control(-1000, 0, 0)
    elif key == 'a':
        vehicle.velocity_control(0, 1000, 0)
    elif key == 'd':
        vehicle.velocity_control(0, -1000, 0)
    elif key == 'x':
        vehicle.velocity_control(0, 0, 0)
    elif key == 'q':
        vehicle.velocity_control(0, 0, 500)
    elif key == 'e':
        vehicle.velocity_control(0, 0, -500)
    elif key == 'i':
        vehicle.position_control(1000, 0, 0)
    elif key == 'j':
        vehicle.position_control(0, 1000, 0)
    elif key == 'k':
        vehicle.position_control(-1000, 0, 0)
    elif key == 'l':
        vehicle.position_control(0, -1000, 0)
    elif key == 'u':
        vehicle.position_control(0, 0, 45)
    elif key == 'o':
        vehicle.position_control(0, 0, -45)
    elif key == 'm':
        vehicle.position_control(0, 0, 0)
    elif key == 'space':
        vehicle.position_control(0, 0, 0)
    else:
        log_message("无效的命令，请重新输入")


if __name__ == '__main__':

    config = settings.get_config()
    log_message(f"加载配置: {config}")
    # serial_comm = SerialCommunication(port=test_port, baudrate=115200, timeout=1.0)
    try:
        # 从配置文件中获取串口参数
        com_port = config["communication"]["com"]
        baudrate = config["communication"]["baudrate"]
        timeout = config["communication"]["timeout"]

        serial_comm = SerialCommunication(
            port=com_port, baudrate=baudrate, timeout=timeout)

        log_message(f"成功初始化串口通信: {com_port}, 波特率: {baudrate}")
        if serial_comm.open():
            log_message("串口已打开")
        else:
            log_message("串口未打开", level="error")
            sys.exit(1)

    except Exception as e:
        log_message(f"初始化串口通信失败: {str(e)}", level="error")
        sys.exit(1)

    # 初始化车辆控制对象
    vehicle = VehicleControl(config)
    vehicle.set_communication(serial_comm)

    sleep(1)

    # 初始化机械臂控制对象
    arm = ArmControl(config)
    arm.set_communication(serial_comm)
    arm.initialize()

    # 初始化游戏控制器
    controller = PyGameController(vehicle, arm)
    controller.set_vehicle_control(vehicle)
    controller.set_arm_control(arm)
    try:
        # 启动控制器监听
        if controller.start():
            log_message("游戏控制器已启动，按Ctrl+C退出...")
            # 保持程序运行
            while True:
                time.sleep(1)
        else:
            log_message("游戏控制器启动失败", level="error")

    # try:
    #     while True:
    #         # keyboard_test(vehicle=vehicle)
    #         sleep(1)
    except KeyboardInterrupt:
        log_message("用户中断程序")
    finally:
        # 停止连续读取并关闭串口
        serial_comm.stop_continuous_read()
        controller.stop()
        serial_comm.close()
        log_message("Control模块测试完成")
    pass
