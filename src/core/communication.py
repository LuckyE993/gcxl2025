from src.utils.helpers import log_message
import sys
import time
from pathlib import Path
from typing import Union, Optional, List, Dict, Any, Tuple
import serial
import serial.tools.list_ports
import struct
import binascii
import threading

# 将项目根目录添加到Python路径
project_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(project_root))


class SerialCommunication:
    """串口通信的封装类，提供串口读写和配置的功能"""

    def __init__(self, port: Optional[str] = None,
                 baudrate: int = 115200,
                 bytesize: int = 8,
                 parity: str = 'N',
                 stopbits: float = 1,
                 timeout: Optional[float] = 1.0,
                 xonxoff: bool = False,
                 rtscts: bool = False,
                 dsrdtr: bool = False):
        """
        初始化串口通信对象

        Args:
            port (Optional[str]): 串口名称，如COM3、/dev/ttyUSB0
            baudrate (int): 波特率，默认115200
            bytesize (int): 数据位，默认8
            parity (str): 校验位，默认'N'（无校验）
            stopbits (float): 停止位，默认1
            timeout (Optional[float]): 读取超时时间，默认1.0秒
            xonxoff (bool): 是否启用软件流控
            rtscts (bool): 是否启用RTS/CTS硬件流控
            dsrdtr (bool): 是否启用DSR/DTR硬件流控
        """
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr

        self.serial = None
        self._is_open = False
        self._read_thread = None
        self._stop_read_thread = threading.Event()
        self._callback = None

    def open(self) -> bool:
        """
        打开串口连接

        Returns:
            bool: 打开成功返回True，否则返回False
        """
        if self._is_open:
            log_message(f"串口 {self.port} 已经打开", level="warning")
            return True

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout,
                xonxoff=self.xonxoff,
                rtscts=self.rtscts,
                dsrdtr=self.dsrdtr
            )
            self._is_open = True
            log_message(f"串口 {self.port} 打开成功，波特率: {self.baudrate}")
            return True
        except Exception as e:
            log_message(f"打开串口 {self.port} 失败: {str(e)}", level="error")
            return False

    def close(self) -> None:
        """
        关闭串口连接
        """
        self.stop_continuous_read()

        if self.serial and self._is_open:
            self.serial.close()
            self._is_open = False
            log_message(f"串口 {self.port} 已关闭")

    def is_open(self) -> bool:
        """
        检查串口是否打开

        Returns:
            bool: 串口打开返回True，否则返回False
        """
        return self._is_open and self.serial and self.serial.is_open

    def write_data(self, data: Union[bytes, bytearray, str], encoding: str = 'utf-8') -> int:
        """
        向串口写入数据

        Args:
            data: 要发送的数据，可以是bytes、bytearray或str
            encoding: 如果data是str，则使用此编码转换为bytes

        Returns:
            int: 成功写入的字节数，失败返回-1
        """
        if not self.is_open():
            log_message("串口未打开，无法写入数据", level="error")
            return -1

        try:
            # 转换字符串为字节
            if isinstance(data, str):
                data = data.encode(encoding)

            # 写入数据
            bytes_written = self.serial.write(data)
            self.serial.flush()  # 确保数据发送完成
            return bytes_written
        except Exception as e:
            log_message(f"写入数据到串口 {self.port} 失败: {str(e)}", level="error")
            return -1

    def read_data(self, size: int = 1024, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        从串口读取指定大小的数据

        Args:
            size: 要读取的最大字节数，默认1024
            timeout: 读取超时时间(秒)，默认None表示使用初始化时设置的超时

        Returns:
            Optional[bytes]: 读取的数据，超时或错误返回None
        """
        if not self.is_open():
            log_message("串口未打开，无法读取数据", level="error")
            return None

        # 保存原始超时设置
        original_timeout = self.serial.timeout

        try:
            # 如果指定了新的超时时间，则临时设置
            if timeout is not None:
                self.serial.timeout = timeout

            # 读取数据
            data = self.serial.read(size)
            return data if data else None
        except Exception as e:
            log_message(f"从串口 {self.port} 读取数据失败: {str(e)}", level="error")
            return None
        finally:
            # 恢复原始超时设置
            if timeout is not None:
                self.serial.timeout = original_timeout

    def read_line(self, timeout: Optional[float] = None, eol: bytes = b'\n') -> Optional[bytes]:
        """
        从串口读取一行数据（直到遇到换行符）

        Args:
            timeout: 读取超时时间(秒)，默认None表示使用初始化时设置的超时
            eol: 行结束标志，默认为换行符

        Returns:
            Optional[bytes]: 读取的数据行，超时或错误返回None
        """
        if not self.is_open():
            log_message("串口未打开，无法读取数据", level="error")
            return None

        # 保存原始超时设置
        original_timeout = self.serial.timeout

        try:
            # 如果指定了新的超时时间，则临时设置
            if timeout is not None:
                self.serial.timeout = timeout

            # 读取一行数据
            data = self.serial.readline()
            return data if data else None
        except Exception as e:
            log_message(f"从串口 {self.port} 读取行数据失败: {str(e)}", level="error")
            return None
        finally:
            # 恢复原始超时设置
            if timeout is not None:
                self.serial.timeout = original_timeout

    def start_continuous_read(self, callback, interval: float = 0.1) -> bool:
        """
        启动连续读取线程，定期检查串口数据并通过回调函数处理

        Args:
            callback: 回调函数，接收读取的数据
            interval: 读取间隔，默认0.1秒

        Returns:
            bool: 启动成功返回True，否则返回False
        """
        if not self.is_open():
            log_message("串口未打开，无法启动连续读取", level="error")
            return False

        if self._read_thread and self._read_thread.is_alive():
            log_message("连续读取线程已经在运行", level="warning")
            return True

        # 设置回调函数和停止标志
        self._callback = callback
        self._stop_read_thread.clear()

        # 创建并启动读取线程
        self._read_thread = threading.Thread(
            target=self._continuous_read_worker,
            args=(interval,),
            daemon=True
        )
        self._read_thread.start()
        log_message(f"启动串口 {self.port} 连续读取线程")
        return True

    def stop_continuous_read(self) -> None:
        """
        停止连续读取线程
        """
        if self._read_thread and self._read_thread.is_alive():
            self._stop_read_thread.set()
            self._read_thread.join(timeout=1.0)
            log_message(f"停止串口 {self.port} 连续读取线程")

    def _continuous_read_worker(self, interval: float) -> None:
        """
        连续读取线程的工作函数

        Args:
            interval: 读取间隔时间
        """
        while not self._stop_read_thread.is_set():
            # 检查是否有数据可读
            if self.serial.in_waiting > 0:
                try:
                    data = self.serial.read(self.serial.in_waiting)
                    if data and self._callback:
                        self._callback(data)
                except Exception as e:
                    log_message(f"连续读取线程异常: {str(e)}", level="error")

            # 等待指定的间隔时间
            time.sleep(interval)

    def flush_input(self) -> None:
        """
        清空输入缓冲区
        """
        if self.is_open():
            self.serial.reset_input_buffer()

    def flush_output(self) -> None:
        """
        清空输出缓冲区
        """
        if self.is_open():
            self.serial.reset_output_buffer()

    def get_settings(self) -> Dict[str, Any]:
        """
        获取当前串口设置

        Returns:
            Dict[str, Any]: 包含串口设置的字典
        """
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "bytesize": self.bytesize,
            "parity": self.parity,
            "stopbits": self.stopbits,
            "timeout": self.timeout,
            "xonxoff": self.xonxoff,
            "rtscts": self.rtscts,
            "dsrdtr": self.dsrdtr,
            "is_open": self.is_open()
        }

    def update_settings(self, **kwargs) -> bool:
        """
        更新串口设置

        Args:
            **kwargs: 需要更新的设置参数

        Returns:
            bool: 更新成功返回True，否则返回False
        """
        # 保存原始设置
        original_settings = self.get_settings()
        was_open = self.is_open()

        try:
            # 如果串口已打开，需要先关闭
            if was_open:
                self.close()

            # 更新设置
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, value)

            # 如果之前是打开的，则重新打开
            if was_open:
                return self.open()
            return True

        except Exception as e:
            log_message(f"更新串口设置失败: {str(e)}", level="error")

            # 恢复原始设置
            for key, value in original_settings.items():
                if hasattr(self, key):
                    setattr(self, key, value)

            # 如果之前是打开的，则尝试重新打开
            if was_open:
                self.open()

            return False

    # 数据格式转换工具函数
    @staticmethod
    def bytes_to_hex(data: bytes) -> str:
        """
        将字节数据转换为十六进制字符串表示

        Args:
            data: 字节数据

        Returns:
            str: 十六进制字符串
        """
        return binascii.hexlify(data).decode('ascii').upper()

    @staticmethod
    def hex_to_bytes(hex_str: str) -> bytes:
        """
        将十六进制字符串转换为字节数据

        Args:
            hex_str: 十六进制字符串

        Returns:
            bytes: 字节数据
        """
        # 移除所有空白字符
        hex_str = ''.join(hex_str.split())
        return binascii.unhexlify(hex_str)

    @staticmethod
    def bytes_to_int(data: bytes, byteorder: str = 'big', signed: bool = False) -> int:
        """
        将字节数据转换为整数

        Args:
            data: 字节数据
            byteorder: 字节顺序，'big'表示大端，'little'表示小端
            signed: 是否有符号

        Returns:
            int: 整数值
        """
        return int.from_bytes(data, byteorder=byteorder, signed=signed)

    @staticmethod
    def int_to_bytes(value: int, length: int, byteorder: str = 'big', signed: bool = False) -> bytes:
        """
        将整数转换为字节数据

        Args:
            value: 整数值
            length: 字节长度
            byteorder: 字节顺序，'big'表示大端，'little'表示小端
            signed: 是否有符号

        Returns:
            bytes: 字节数据
        """
        return value.to_bytes(length, byteorder=byteorder, signed=signed)

    @staticmethod
    def calculate_checksum(data: bytes, method: str = 'sum') -> int:
        """
        计算校验和

        Args:
            data: 要计算校验和的数据
            method: 校验方法，支持'sum'和'xor'

        Returns:
            int: 校验和值
        """
        if method == 'sum':
            return sum(data) & 0xFF  # 8位校验和
        elif method == 'xor':
            checksum = 0
            for byte in data:
                checksum ^= byte
            return checksum
        else:
            raise ValueError(f"不支持的校验方法: {method}")


def list_serial_ports() -> List[Dict[str, Any]]:
    """
    列出系统中的所有串口信息

    Returns:
        List[Dict[str, Any]]: 串口信息列表
    """
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'name': port.name,
            'description': port.description,
            'hwid': port.hwid,
            'vid': port.vid,
            'pid': port.pid,
            'serial_number': port.serial_number,
            'manufacturer': port.manufacturer,
            'product': port.product
        })
    return ports


def create_protocol_message(payload=None):
    """
    创建符合协议的消息
    - 头帧: FF
    - 有效载荷: 默认为8个字节的00
    - 尾帧: FE

    Args:
        payload (bytes, optional): 有效载荷，如果为None则使用8个字节的00

    Returns:
        bytes: 完整的协议消息
    """
    # 默认有效载荷为8个字节的00
    if payload is None:
        payload = bytes([0x00] * 8)

    # 构建完整消息: 头帧 + 有效载荷 + 尾帧
    message = bytes([0xFF]) + payload + bytes([0xFE])
    return message


def send_protocol_message(serial_comm, payload=None):
    """
    发送符合协议的消息

    Args:
        serial_comm (SerialCommunication): 串口通信对象
        payload (bytes, optional): 有效载荷，如果为None则使用8个字节的00

    Returns:
        int: 发送的字节数，失败返回-1
    """
    # 验证payload的长度为8个字节
    if payload is not None:
        if len(payload) != 8:
            log_message(f"无效的payload长度: {len(payload)}，应为8字节", level="error")
            return -1
    message = create_protocol_message(payload)
    log_message(f"发送协议消息: {SerialCommunication.bytes_to_hex(message)}")
    return serial_comm.write_data(message)


# 测试代码
if __name__ == "__main__":
    # 列出所有可用串口
    available_ports = list_serial_ports()
    log_message(f"可用串口列表:")
    for idx, port in enumerate(available_ports):
        log_message(f"[{idx}] {port['device']}: {port['description']}")

    if not available_ports:
        log_message("未找到可用串口", level="warning")
        sys.exit(1)

    # 选择第一个串口进行测试
    test_port = "COM6"

    # 创建串口对象
    serial_comm = SerialCommunication(
        port=test_port, baudrate=115200, timeout=1.0)

    try:
        # 打开串口
        if not serial_comm.open():
            log_message("无法打开串口进行测试", level="error")
            sys.exit(1)

        log_message(f"串口设置: {serial_comm.get_settings()}")

        # 定义数据接收回调函数
        def data_received_callback(data):
            hex_data = SerialCommunication.bytes_to_hex(data)
            log_message(
                f"收到数据: {hex_data} (ASCII: {data.decode('ascii', errors='replace')})")

        # 启动连续读取
        serial_comm.start_continuous_read(data_received_callback)

        # 发送测试数据
        log_message("发送测试数据...")
        test_data = "Hello, Serial Port!\r\n"
        bytes_written = serial_comm.write_data(test_data)
        log_message(f"发送了 {bytes_written} 字节数据")

        # 等待并接收响应
        log_message("等待响应(5秒)...")
        time.sleep(5)

        # 数据格式转换演示
        test_int = 12345
        log_message(
            f"整数 {test_int} 转为字节: {SerialCommunication.bytes_to_hex(SerialCommunication.int_to_bytes(test_int, 4))}")

        test_bytes = b'\x01\x02\x03\x04'
        log_message(
            f"字节 {SerialCommunication.bytes_to_hex(test_bytes)} 转为整数: {SerialCommunication.bytes_to_int(test_bytes)}")

        # 计算校验和演示
        log_message(
            f"数据 {SerialCommunication.bytes_to_hex(test_bytes)} 的校验和: {SerialCommunication.calculate_checksum(test_bytes, 'sum')}")

    except KeyboardInterrupt:
        log_message("用户中断测试")
    except Exception as e:
        log_message(f"测试过程中发生异常: {str(e)}", level="error")
    finally:
        # 停止连续读取并关闭串口
        serial_comm.stop_continuous_read()
        serial_comm.close()
        log_message("串口通信测试完成")
