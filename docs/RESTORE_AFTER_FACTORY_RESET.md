# 机器人恢复出厂设置后的继续步骤

更新时间：2026-05-31
恢复前机器人：`unitree@10.88.2.69`
本机完整备份：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408
```

备份里有 3 个压缩包：

| 文件 | 内容 |
| --- | --- |
| `g1_hand_arm_project_full_20260531_151408.tar.gz` | 机器人上的完整 `g1_hand_arm_project`，包含日志/debug 图等 |
| `revo2_python_full_20260531_151408.tar.gz` | Revo2 `python/revo2` 目录完整快照 |
| `handeye_yolo_models_and_dataset_20260531_151408.tar.gz` | YOLO 水瓶模型、训练数据集、可疑相关模型，包括未上传 GitHub 的大模型 |

## 1. 重新准备可下载环境

Unitree Python SDK：

```bash
cd /home/unitree
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

Revo2 SDK/示例包需要恢复到：

```text
/home/unitree/stark-serialport-example/python/revo2
```

这些 SDK 本体能重新下载或从厂商包恢复，所以 GitHub 没有完整上传 SDK。

## 2. 从 GitHub 恢复项目代码

Windows PowerShell：

```powershell
$robot = "unitree@机器人IP"
$repo = "E:\CodexProjects\Unitree_Projects\Vision-Guided-Manipulation-System-for-Unitree-G1"

scp -r "$repo\g1_hand_arm_project" "$robot:/home/unitree/unitree_sdk2_python/"
scp "$repo\hand_control\revo2\left_hand_safe_once.py" "$robot:/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py"
```

如果要把 Revo2 历史手型备份也放回去：

```powershell
scp "$repo\hand_control\revo2\left_hand_safe_once.py.bak*" "$robot:/home/unitree/stark-serialport-example/python/revo2/"
```

## 3. 从本机恢复大模型和数据

完整模型/数据压缩包在：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408\handeye_yolo_models_and_dataset_20260531_151408.tar.gz
```

传回机器人后解压：

```powershell
$backup = "E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408"
scp "$backup\handeye_yolo_models_and_dataset_20260531_151408.tar.gz" "$robot:/home/unitree/"
```

机器人上执行：

```bash
cd /home/unitree
tar -xzf handeye_yolo_models_and_dataset_20260531_151408.tar.gz
```

当前 YOLO 启动脚本默认模型路径是：

```text
/home/unitree/YOLO_Model_Workspace/Models/Training_Runs/bottle_v12/weights/best.pt
```

GitHub 的 `models/` 里也保留了一份可上传的小模型，优先级见 `models/MODEL_MANIFEST.md`。

## 4. 先做不动机器人的验证

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 -m py_compile arm_pick_place_standalone.py right_hand_thumb_test.py test_arm_pick_place_flow.py
python3 -m py_compile vision/detect_bottle_2d.py vision/detect_bottle_depth.py vision/transform_bottle_to_base.py
python3 -m unittest test_arm_pick_place_flow.py
python3 -m unittest tools/test_arm_pregrasp_preview.py
```

视觉测试如果同目录导入报错，到 `vision` 目录跑：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/vision
python3 -m unittest test_detect_bottle_2d.py
```

## 5. 验证 Revo2 左手

先只测手，不动手臂：

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
python3 left_hand_safe_once.py thumb_ready
python3 left_hand_safe_once.py bottle
python3 left_hand_safe_once.py open
```

当前左手默认：

```text
PORT=/dev/ttyUSB1
SLAVE_ID=0x7e
手型顺序=[thumb, thumb_aux, index, middle, ring, pinky]
bottle=[180, 850, 480, 560, 540, 420]
```

也可以只测独立脚本里的手部流程：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place_standalone.py --hand-only-test
```

## 6. 验证视觉窗口

连上机器人 VNC 后：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
bash vision/start_bottle_rgb_fast.sh
```

当前启动参数：

```text
/dev/video4 RGB，相机用 V4L2 后端
640x480，YOLO imgsz 640
conf=0.15，infer-every=2
tracking: max_jump=80px, smooth_alpha=0.35, lock_conf=0.25
```

关闭：

```bash
pkill -f "vision/detect_bottle_2d.py"
```

如果窗口关不掉：

```bash
pkill -9 -f "vision/detect_bottle_2d.py"
```

## 7. 验证手臂和全流程

先确认机器人处于稳定站立/预备模式，桌面、线缆和人手离开运动范围。

空手预抓小幅测试：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 tools/arm_pregrasp_preview.py --net eth0 --offset 16=-0.02,15=0.02,18=0.02 --max-abs-offset 0.08 --hold-seconds 3
```

完整固定轨迹：

```bash
python3 arm_pick_place_standalone.py eth0
```

当前关键机械臂参数在 `arm_pick_place_standalone.py`：

```python
UNFOLD_PREGRASP_DELTA[18] = -0.19
LIFT_BOTTLE_DELTA[18] = -1.0
OUTWARD_RELEASE_SHOULDER_ROLL_DELTA = 0.9
```

`18` 更小，小臂更向上折；`18` 更大，小臂更低。

## 8. 后续自动化顺序

1. 固定轨迹抓放稳定后，再恢复 YOLO + 深度输出。
2. 用 `record_bottle_positions.py` 重新记录目标点和随机点。
3. 准备 ChArUco 或 AprilTag 标定板。
4. 完成手眼标定并验证 `transform_bottle_to_base.py`。
5. 标定稳定后，再做轨迹规划和自动抓取。

在第 4 步之前，不要让视觉坐标直接控制机械臂。
