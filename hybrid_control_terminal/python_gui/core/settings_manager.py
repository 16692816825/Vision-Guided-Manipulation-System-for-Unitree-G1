# core/settings_manager.py
import json
import os
import shutil
from pathlib import Path

# 默认配置字典（当文件丢失时使用）
DEFAULT_SETTINGS = {
    "network": {
        "interface": "enp2s0",
        "robot_ip": "192.168.123.164",
        "local_port": 9998,
        "dds_domain": 0
    },
    "ros": {
        "enabled": True,
        "setup_path": "",
        "workspace_root": ""
    },
    "robot": {
        "model_xml_path": "unitree_mujoco/unitree_robots/g1/g1_23dof.xml",
        "rl_policy_path": "",
        "use_right_arm": False
    },
    "ai": {
        "deepseek_api_key": ""
    },
    "calibration": {
        "cam_offset_x": 0.0476,
        "cam_offset_y": 0.0,
        "cam_offset_z": 0.4627,
        "cam_pitch_deg": 42.0
    },
    "control": {
        "sim_only": False,
        "sim_rate": 0.04
    }
}

class SettingsManager:
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.settings = {}
            cls._instance.config_file = Path("settings.json")
            cls._instance.load()
        return cls._instance

    def load(self):
        """加载配置，如果文件不存在则创建默认文件"""
        if not self.config_file.exists():
            print(f"[Settings] 配置文件未找到，正在生成默认文件: {self.config_file}")
            self.settings = DEFAULT_SETTINGS
            self.save()
        else:
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self.settings = json.load(f)
                # 简单的合并逻辑：防止新版本增加了字段但旧配置里没有
                self._merge_defaults(self.settings, DEFAULT_SETTINGS)
            except Exception as e:
                print(f"[Settings] 加载失败: {e}，将使用默认内存配置。")
                self.settings = DEFAULT_SETTINGS

    def _merge_defaults(self, current, defaults):
        """递归合并配置，确保所有键都存在"""
        for key, value in defaults.items():
            if key not in current:
                current[key] = value
            elif isinstance(value, dict) and isinstance(current[key], dict):
                self._merge_defaults(current[key], value)

    def save(self):
        """保存当前配置到文件"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.settings, f, indent=4, ensure_ascii=False)
            print("[Settings] 配置已保存。")
        except Exception as e:
            print(f"[Settings] 保存失败: {e}")

    def get(self, section, key, default=None):
        """安全获取配置项"""
        return self.settings.get(section, {}).get(key, default)

    def set(self, section, key, value):
        """设置配置项并保存"""
        if section not in self.settings:
            self.settings[section] = {}
        self.settings[section][key] = value
        self.save()

# 全局单例
settings = SettingsManager()
