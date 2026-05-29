这是一个为 **Unitree G1 Hybrid Control Terminal** 项目编写的详细 `README.md` 文档。内容基于你提供的三部分代码进行了严格的梳理，保持客观、技术导向，没有夸大功能。

---

# Unitree G1 Hybrid Control Terminal (V7.6)

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-orange.svg)](https://mujoco.org/)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)]()

## 项目简介

**Unitree G1 Hybrid Control Terminal** 是专为 **宇树科技 (Unitree) G1 人形机器人** 开发的上位机控制系统。该项目采用 **Sim-to-Real (数字孪生)** 架构，通过多进程设计，在同一终端内实现了基于 MuJoCo 的实时仿真与基于 DDS/ROS 的真机控制。

该系统旨在为开发者提供一个集运动控制、导航规划、灵巧手交互及状态监控于一体的综合调试平台。它支持 23-DOF 全身控制，集成了 RL (强化学习) 策略推理与传统 IK (逆运动学) 算法。

## 核心特性

### 1. 多模态运动控制
*   **混合控制架构**: 支持强化学习 (RL) 端到端策略与微分逆运动学 (IK) 的平滑切换。
*   **全身运控**:
    *   **底盘**: 全向移动控制（前后、横移、旋转）。
    *   **手臂**: 基于 `gymnasium` 和 `stable-baselines3` 的 RL 推理，以及基于 Jacobian 的 IK 轨迹跟随。
*   **轨迹规划**: 内置 S 型速度曲线规划器，支持笛卡尔空间的点到点 (PTP) 平滑移动。
*   **特殊启动流程**: 集成自动化的“悬挂启动 (Hanger)”与“蹲姿启动 (Squat)”逻辑，适配不同的调试环境。

### 2. ROS 导航集成
*   **ROS 1 桥接**: 通过 `rospy` 与底层 ROS Noetic 环境通信。
*   **全功能导航工作流**:
    *   **建图**: 集成 Gmapping/Cartographer 建图指令。
    *   **导航**: 集成 MoveBase，支持全局/局部路径规划可视化。
    *   **交互式地图**: 自定义 OpenGL 控件，支持 Ctrl+拖拽设置目标点，Shift+拖拽进行 AMCL 重定位。

### 3. 视觉与灵巧手交互
*   **点击即抓取 (Click-to-Grasp)**: 基于 Realsense D435i 深度流，实现了从 2D 屏幕像素反推 3D 基座坐标的算法。
*   **L10 灵巧手控制**:
    *   UDP 直连控制。
    *   **触觉可视化**: UI 提供 12x6 阵列的指尖压力热力图 (`FingerHeatmap`)，实时反馈接触力。

### 4. 辅助开发工具
*   **示教与回放**: 支持“零力”拖拽示教，记录关节轨迹并保存为 JSON，支持平滑回放。
*   **系统诊断**: 实时监控关节温度、电压、通信延迟，提供类汽车仪表盘的可视化界面。
*   **安全层**: 内置防自碰撞限位、腰部锁定逻辑及电机过热保护。

---

## 系统架构

系统采用 **Python 多进程 (Multiprocessing)** 架构以保证控制频率的稳定性：

1.  **UI 主进程 (Main Process)**: 基于 PyQt5，负责渲染界面、处理用户输入及 OpenGL 地图绘制。
2.  **核心控制进程 (`RobotProcess`)**:
    *   **Sim Engine**: 运行 MuJoCo 物理引擎，维护机器人的运动学模型。
    *   **Bridge Layer**: 管理 Unitree DDS (真机)、ROS (导航) 和 UDP (灵巧手) 的通信。
    *   **Solver**: 执行 IK 解算和 RL 神经网络推理。
3.  **通信机制**: 进程间通过 `multiprocessing.Queue` 进行指令下发和状态上报。

---

## 环境依赖与安装

### 1. 系统要求
*   **操作系统**: Ubuntu 20.04 (推荐，适配 ROS Noetic) 或 Windows (仅支持仿真模式)。
*   **Python**: 3.8 或更高版本。

### 2. Python 依赖
```bash
pip install numpy scipy PyQt5
pip install mujoco gymnasium stable-baselines3 torch
```

### 3. 硬件 SDK
*   **Unitree SDK**: 需安装 `unitree_sdk2py`。请参考 [Unitree GitHub](https://github.com/unitreerobotics/unitree_sdk2_python) 进行编译和安装。

### 4. ROS 依赖 (仅真机导航需要)
确保已安装 ROS Noetic 基础组件：
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-map-server
```

---

## 目录结构

项目运行依赖于特定的目录结构，请确保文件组织如下：

```text
G1_Controller/
├── main.py                  # 程序入口
├── config.py                # 配置文件 (关节映射、参数)
├── boot.py                  # 启动状态机逻辑
├── utils.py                 # RL 模型加载工具
├── g1_arm_rl_env.py         # RL 环境定义
├── train_g1_arm_policy.py   # 训练脚本
├── models/                  # [需新建] 存放 RL 模型 (.zip)
├── maps/                    # [需新建] 存放导航地图 (.yaml/.pgm)
├── data_logger/             # [自动生成] 存放示教轨迹数据
├── core/                    # 核心逻辑
│   ├── bridge.py            # DDS 通信
│   ├── ik_solver.py         # 逆运动学求解
│   ├── robot_process.py     # 主控进程
│   ├── ros_bridge.py        # ROS 桥接
│   ├── sim_engine.py        # MuJoCo 引擎
│   └── ... (其他核心文件)
├── ui/                      # 界面代码
│   ├── main_window.py       # 主窗口
│   ├── nav_widget.py        # 地图控件
│   └── widgets.py           # 自定义组件
└── unitree_mujoco/          # Unitree 官方 XML 模型库
    └── unitree_robots/g1/g1_23dof.xml
```

---

## 使用说明

### 1. 启动命令

**纯仿真模式 (测试 UI 和逻辑):**
```bash
python3 main.py --sim-only --model models/your_policy.zip
```

**真机控制模式 (需连接机器人):**
请确保电脑 IP 与机器人处于同一网段 (192.168.123.x)，并指定网卡名称。
```bash
python3 main.py --iface enp2s0 --model models/your_policy.zip
```

**参数说明:**
*   `--right-arm`: 启用右臂控制（默认左臂）。
*   `--domain`: 指定 DDS Domain ID (默认 0)。

### 2. 功能模块指南

#### **主控面板 (Left Panel)**
*   **启动/关机**: 长按按钮触发。支持选择“悬挂启动”（用于吊装调试）或“蹲姿启动”（用于地面）。
*   **速度限制**: 调节手臂灵敏度和底盘移动速度。

#### **Tab 1: 数据监控**
*   显示机器人基座的里程计信息 (X/Y/Yaw)。
*   显示末端执行器的目标位置与实际位置的偏差。
*   显示视觉识别到的目标点坐标。

#### **Tab 2: 关节详情**
*   列出全身 23 个关节的实时角度、速度。
*   **温度监控**: 温度 > 60°C 变红报警。

#### **Tab 3: 示教模式**
1.  点击 **"开始录制"**: 机器人进入零力/低阻尼模式。
2.  手动拖拽机械臂到指定位置。
3.  点击 **"结束录制"**: 保存轨迹为 JSON。
4.  选中文件点击 **"回放"**: 机器人将平滑复现动作。

#### **Tab 4: 精确控制**
*   **IK 滑块**: 手动调节末端执行器的 XYZ 和 Yaw 角度。
*   **航点序列 (Mission Sequence)**:
    *   添加当前姿态为关键帧。
    *   设置到达后的动作（抓取/张开）。
    *   点击 "执行序列" 进行多点连续运动。

#### **Tab 5: 智能导航**
*   **建图**: 启动 Gmapping，遥控机器人探索环境，保存地图。
*   **导航**: 加载地图，使用 Ctrl+左键在地图上设定目标点，机器人自动规划路径移动。

#### **Tab 6: 灵巧手 L10**
*   **控制**: 一键握拳、张开，或设定直径进行智能抓取。
*   **热力图**: 实时显示 5 个手指的压力分布矩阵，用于判断抓取是否牢固。

---

## 配置说明 (`config.py`)

你可以在 `config.py` 中修改以下核心参数：

*   `JOINTS` / `G1_JOINT_MAP`: 定义关节 ID 与名称的映射。
*   `IK_KP` / `IK_KD`: 逆运动学控制的刚度与阻尼参数。
*   `CAM_OFFSET_...`: 相机相对于机器人基座的外参（用于视觉坐标转换）。
*   `PASSIVE_KP`: 示教模式下的电机刚度（通常设为 0 以便拖拽）。

---

## 注意事项与故障排除

1.  **MuJoCo 渲染错误**:
    *   如果在无头服务器或特定显卡驱动下报错，`main.py` 已强制设置 `os.environ['MUJOCO_GL'] = 'glfw'`。如仍有问题，请尝试安装 `libglfw3`。
2.  **ROS 话题未收到**:
    *   请检查 `ros_bridge.py` 中的话题名称（如 `/camera/color/image_raw`）是否与实际传感器话题一致。
3.  **权限问题**:
    *   DDS 通信可能需要 root 权限或将用户加入相应网络组。
4.  **安全警告**:
    *   **悬挂关机**会直接切断力矩进入阻尼模式，请务必确保已挂好安全绳，否则机器人会直接摔倒。

---

## 许可证

本项目基于 MIT 许可证开源。请在使用真机调试时务必注意安全，开发者不对因操作不当导致的硬件损坏负责。
