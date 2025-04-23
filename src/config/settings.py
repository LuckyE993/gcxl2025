"""
应用程序配置管理。
"""
import os
import json
from pathlib import Path

DEFAULT_CONFIG = {
    "debug": False,
    "log_level": "INFO",
    "data_path": "../data"
}

def get_config_path():
    """获取配置文件路径"""
    config_dir = os.environ.get("CONFIG_DIR", "config")
    return Path(config_dir) / "config.json"

def get_config():
    """加载配置"""
    config_path = get_config_path()
    print(f"加载配置文件: {config_path}")
    if config_path.exists():
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                print(f"加载配置文件成功")
                return json.load(f)
        except Exception:
            return DEFAULT_CONFIG
    return DEFAULT_CONFIG
