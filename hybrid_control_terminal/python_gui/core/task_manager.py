# core/task_manager.py
import json
import pathlib
from config import DATA_LOG_DIR

class TaskManager:
    """
    任务数据管理单例类 (升级版 V2)
    职责：读写 tasks.json，支持有序的任务链管理
    """
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init_data()
        return cls._instance

    def _init_data(self):
        self.file_path = DATA_LOG_DIR / "tasks.json"
        self.tasks = [] # List of dict (有序列表)
        self.load()

    def load(self):
        """从文件加载任务列表"""
        if not self.file_path.exists():
            self.tasks = []
            self.save() # 创建空文件
            return

        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, list):
                    self.tasks = data
                else:
                    print("[TaskManager] 数据格式旧版，重置为空列表")
                    self.tasks = []
        except Exception as e:
            print(f"[TaskManager] 加载失败: {e}")
            self.tasks = []

    def save(self):
        """保存任务列表到文件"""
        try:
            DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
            with open(self.file_path, 'w', encoding='utf-8') as f:
                json.dump(self.tasks, f, indent=4, ensure_ascii=False)
            print(f"[TaskManager] 已保存 {len(self.tasks)} 个任务")
            return True
        except Exception as e:
            print(f"[TaskManager] 保存失败: {e}")
            return False

    def save_ordered_list(self, task_list):
        """
        [新接口] 直接保存 UI 传来的有序列表
        task_list: list of dict
        """
        self.tasks = task_list
        return self.save()

    def get_all_tasks(self):
        """获取所有任务列表 (有序)"""
        return self.tasks

    # --- 兼容旧接口 (如果其他地方还在用) ---
    def get_task(self, uuid_or_id):
        for t in self.tasks:
            # 兼容 uuid 或 旧版的 trigger_id
            if t.get('uuid') == uuid_or_id or t.get('trigger_id') == uuid_or_id:
                return t
        return None

# 全局单例
task_manager = TaskManager()