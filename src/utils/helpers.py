"""
通用工具函数。
"""
import datetime

def log_message(message):
    """简单日志函数"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] {message}")

def read_data_file(filepath):
    """读取数据文件"""
    try:
        with open(filepath, 'r', encoding='utf-8') as file:
            return file.read()
    except Exception as e:
        log_message(f"读取文件错误: {e}")
        return None
