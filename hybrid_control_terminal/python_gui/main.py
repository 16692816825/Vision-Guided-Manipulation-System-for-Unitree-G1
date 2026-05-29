import sys
import argparse
import pathlib
import os

# ==========================================================
# 🚀 关键配置：强制使用 GLFW 后端
# ==========================================================
os.environ['MUJOCO_GL'] = 'glx'

# 必须最先导入 PyQt5
from PyQt5.QtWidgets import QApplication

# [新增] 引入配置管理器，用于获取默认参数
from core.settings_manager import settings

def main():
    # 1. 从配置文件获取默认值
    default_model = settings.get("robot", "rl_policy_path", "models/default.zip")
    default_iface = settings.get("network", "interface", "enp2s0")
    default_domain = settings.get("network", "dds_domain", 0)
    
    # 获取布尔值的默认状态
    default_sim_only = settings.get("control", "sim_only", False)
    default_right_arm = settings.get("robot", "use_right_arm", False)
    default_rate = settings.get("control", "sim_rate", 0.04)

    # 2. 定义命令行参数 (CLI 参数优先级最高，但默认值来自 Config)
    parser = argparse.ArgumentParser(description="G1 Hybrid Control Terminal (V7.7)")
    
    parser.add_argument("--model", default=default_model, help="策略模型路径")
    parser.add_argument("--iface", default=default_iface, help="DDS 网络接口 (e.g. enp2s0)")
    parser.add_argument("--domain", type=int, default=default_domain, help="DDS 域 ID")
    parser.add_argument("--rate", type=float, default=default_rate, help="仿真步长")

    # 布尔值处理：如果配置里是 True，这里默认就是 True；
    # 注意：argparse 的 store_true 意味着“只要出现这个参数就变 True”。
    # 为了兼容配置文件的默认值，我们使用 set_defaults
    parser.add_argument("--right-arm", action="store_true", help="强制使用右臂 (覆盖配置)")
    parser.add_argument("--sim-only", action="store_true", help="强制仅仿真模式 (覆盖配置)")
    
    # 应用配置文件的布尔默认值
    parser.set_defaults(right_arm=default_right_arm)
    parser.set_defaults(sim_only=default_sim_only)

    args = parser.parse_args()

    # 3. 路径检查
    if not pathlib.Path(args.model).exists():
        print(f"[警告] 找不到模型文件: {args.model}")
        print("[提示] 您可能需要先运行 train_g1_arm_policy.py 进行训练，或者检查路径配置。")
    
    # 4. 初始化 Qt 应用
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 5. 启动前自检
    print("\n[Boot] 正在检查环境配置...")
    
    # 检查 MuJoCo XML (从配置读取路径)
    try:
        import mujoco
        xml_path = settings.get("robot", "model_xml_path", "unitree_mujoco/unitree_robots/g1/g1_23dof.xml")
        
        if os.path.exists(xml_path):
            print(f"[Boot] XML 模型存在: {xml_path}")
            mujoco.MjModel.from_xml_path(xml_path)
            print("[Boot] XML 预编译通过 ✅")
        else:
            print(f"[ERROR] 找不到 XML 模型: {xml_path}")
            print(f"当前工作目录: {os.getcwd()}")
            return
    except Exception as e:
        print(f"[FATAL] XML 模型加载失败: {e}")
        return

    # 6. 加载 UI 并启动
    print("[Boot] 正在加载 UI 组件...")
    from ui.main_window import MainWindow

    print(f"[Boot] 启动主窗口 (Iface: {args.iface}, Sim: {args.sim_only})...")
    window = MainWindow(args)
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
