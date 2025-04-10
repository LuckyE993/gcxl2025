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


class PyGameController:
    def __init__(self, vehicle_control=None):
        """Initialize PyGame controller

        Args:
            vehicle_control (VehicleControl, optional): 车辆控制对象
        """
        pygame.init()
        pygame.joystick.init()

        self.vehicle = vehicle_control
        self.joysticks = []
        self.active_joystick = None
        self.running = False
        self.thread = None

        # 控制参数
        self.deadzone = 0.1  # 摇杆死区
        self.max_speed = 1000  # 最大速度 mm/s
        self.max_rotation = 180  # 最大旋转速度 deg/s

        # 初始化可用的手柄
        self._init_joysticks()

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

        while self.running:
            # 处理pygame事件
            for event in pygame.event.get():
                pass  # 此处可以添加按钮事件处理

            if not self.active_joystick:
                time.sleep(0.1)
                continue

            try:
                # 左摇杆控制方向 (axis 0 是x轴，axis 1是y轴)
                steer_dir_x = self.active_joystick.get_axis(1)* -1
                steer_dir_y = self.active_joystick.get_axis(0)* -1   

                # 右摇杆控制旋转 (axis 2是x轴，axis 3是y轴)
                steer_rotation_x = self.active_joystick.get_axis(2) * -1
                # steer_rotation_y = self.active_joystick.get_axis(3)

                # 扳机控制油门 (axis 4是左扳机，axis 5是右扳机)
                left_trigger = self.active_joystick.get_axis(
                    4)  # 范围从-1到1，未按时为-1
                right_trigger = self.active_joystick.get_axis(
                    5)  # 范围从-1到1，未按时为-1

                # 应用死区
                steer_dir_x = 0 if abs(
                    steer_dir_x) < self.deadzone else steer_dir_x
                steer_dir_y = 0 if abs(
                    steer_dir_y) < self.deadzone else steer_dir_y
                steer_rotation_x = 0 if abs(
                    steer_rotation_x) < self.deadzone else steer_rotation_x
                # steer_rotation_y = 0 if abs(
                #     steer_rotation_y) < self.deadzone else steer_rotation_y
                # 计算运动控制值
                # 将扳机值从[-1,1]映射到[0,1]
                left_power = ((left_trigger + 1) / 2) * -1
                right_power = (right_trigger + 1) / 2

                # 计算前进/后退的合成速度
                # forward_speed = right_power * self.max_speed
                # backward_speed = left_power * -self.max_speed

                vx = steer_dir_x * (left_power+right_power) *  self.max_speed  # 前进速度
                vy = steer_dir_y * (left_power+right_power) * self.max_speed  # 侧向速度
                vyaw = steer_rotation_x * (left_power+right_power)* self.max_rotation  # 旋转速度
                vx = int(vx)
                vy = int(vy)
                vyaw = int(vyaw)
                # 左摇杆控制旋转

                # 发送控制命令
                self.vehicle.velocity_control(int(vx), int(vy), int(vyaw))
                # print(
                #     f"steer_dir_x: {steer_dir_x}, steer_dir_y: {steer_dir_y}, steer_rotation_x: {steer_rotation_x}, left_power: {left_power}, right_power: {right_power}")
                # print(f"vx: {vx}, vy: {vy}, vyaw: {vyaw}")

            except Exception as e:
                log_message(f"控制器处理异常: {str(e)}", level="error")

            # 控制循环频率
            clock.tick(20)  # 20Hz


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
    vehicle = VehicleControl(config)
    vehicle.set_communication(serial_comm)

    # 初始化游戏控制器
    controller = PyGameController(vehicle)
    controller.set_vehicle_control(vehicle)
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
