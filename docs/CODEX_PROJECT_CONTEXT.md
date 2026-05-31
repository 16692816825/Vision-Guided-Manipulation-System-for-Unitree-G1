# Codex 新窗口项目上下文

请先完整阅读本文件和 `docs/RESTORE_AFTER_FACTORY_RESET.md`，再修改代码。这个仓库是“宇树 G1 + 强脑 Revo2 灵巧手 + 视觉辅助抓瓶”项目在机器人恢复出厂设置前的备份。

## 交互要求

- 用中文回答。
- 直接、务实、工程化，优先给可执行命令和明确风险。
- 只有需要用户操作实体机器人、瓶子、相机、标定板、VNC 时再让用户做；其余文件同步、代码检查、文档和普通命令尽量直接完成。
- 涉及学习建议必须附链接。

## 当前来源和备份

恢复前机器人：

```bash
ssh unitree@10.88.2.69
```

核心机器人路径：

```text
/home/unitree/unitree_sdk2_python/g1_hand_arm_project
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
/home/unitree/YOLO_Model_Workspace/Models/Training_Runs/bottle_v12/weights/best.pt
```

本机仓库：

```text
E:\CodexProjects\Unitree_Projects\Vision-Guided-Manipulation-System-for-Unitree-G1
```

完整本机快照：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408
```

其中：

- `g1_hand_arm_project_full_20260531_151408.tar.gz` 是机器人主项目完整快照，含日志/debug 图。
- `revo2_python_full_20260531_151408.tar.gz` 是 Revo2 `python/revo2` 完整快照。
- `handeye_yolo_models_and_dataset_20260531_151408.tar.gz` 是 YOLO 模型、数据集和大模型本地备份。

## 强制安全约束

1. 不要把手臂 `arm_sdk` 控制写进 `hand_control/revo2/left_hand_safe_once.py`。
2. 不要再使用只写 `motor_cmd[29].q = 0` 的空 `LowCmd` 释放方式。
3. 释放 `arm_sdk` 时必须保持当前 `lowstate` 里的关节 `q/kp/kd`，再平滑降低 `WEIGHT=29`。
4. 涉及机器人动作时先小幅、空手、低速测试。
5. 手眼标定稳定前，不要把视觉结果直接闭环到机械臂。

## 当前 G1 手臂控制

23DoF 场景下当前使用：

```python
LEFT_ARM = [15, 16, 17, 18, 19]
OTHER_ARM = [22, 23, 24, 25, 26]
WEIGHT = 29
DT = 0.02
KP = 20.0
KD = 1.0
```

主要关节含义：

- `15`: LeftShoulderPitch，大臂前后/上抬。
- `16`: LeftShoulderRoll，左右外扩/内收。
- `18`: LeftElbow，小臂折叠/伸开。
- `19`: LeftWristRoll，手腕旋转。

当前优先维护脚本：

```text
g1_hand_arm_project/arm_pick_place_standalone.py
```

当前流程：

```text
张手 -> 安全初始位检查 -> 折小臂 -> 大臂送到固定点
-> 小臂展开到瓶子 -> thumb_open_max -> thumb_ready -> bottle
-> 小臂抬瓶 -> 悬停 3 秒 -> 大臂外扩
-> 外扩姿态下放小臂 -> 张手放瓶
-> 空手折回、外扩收回、回初始位 -> 平滑释放 arm_sdk
```

当前关键参数：

```python
UNFOLD_PREGRASP_DELTA[18] = -0.19
LIFT_BOTTLE_DELTA[18] = -1.0
OUTWARD_RELEASE_SHOULDER_ROLL_DELTA = 0.9
```

`18` 更小会让小臂更向上折；`18` 更大则更低。

## Revo2 左手

GitHub 中保留的定制手部脚本：

```text
hand_control/revo2/left_hand_safe_once.py
```

恢复到机器人后应放到：

```text
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

当前配置：

```python
PORT = "/dev/ttyUSB1"
SLAVE_ID = 0x7e
BAUDRATE = libstark.Baudrate.Baud460800
```

手型顺序：

```text
[thumb, thumb_aux, index, middle, ring, pinky]
```

当前手型：

```python
"open": [0, 0, 0, 0, 0, 0]
"thumb_open_max": [0, 0, 0, 0, 0, 0]
"thumb_ready": [0, 1000, 0, 0, 0, 0]
"bottle": [180, 850, 480, 560, 540, 420]
```

独立主流程内置 Revo2 控制，不通过 `left_hand_safe_once.py` 调用，但 `left_hand_safe_once.py` 仍用于单独测手。

## 视觉状态

当前 RGB YOLO 启动脚本：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
bash vision/start_bottle_rgb_fast.sh
```

默认参数：

```text
source=/dev/video4
selected=/dev/video4/v4l2
width=640
height=480
imgsz=640
conf=0.15
infer-every=2
track-max-jump=80
track-smooth-alpha=0.35
track-lost-frames=12
track-switch-frames=8
track-lock-conf=0.25
```

`detect_bottle_2d.py` 已加入目标跟踪逻辑：锁定后优先跟随上一帧附近的框，远处新框必须连续稳定出现才切换，避免水瓶框来回横跳。

视觉相关文件：

```text
vision/detect_bottle_2d.py
vision/start_bottle_rgb_fast.sh
vision/detect_bottle_depth.py
vision/record_bottle_positions.py
vision/compare_bottle_to_target.py
vision/evaluate_grasp_window.py
vision/auto_grasp_decision.py
vision/transform_bottle_to_base.py
vision/handeye_g1_head_camera.json
vision/target_grasp_reference.json
vision/records/*.csv
```

当前还没有正式闭环路径规划。下一步应先稳定 YOLO + 深度输出，再用 ChArUco 或 AprilTag 做手眼标定。

## GitHub 与本机备份策略

GitHub 已保留：

- `g1_hand_arm_project/` 代码、小 JSON/CSV、历史脚本备份。
- `hand_control/revo2/` 中与本项目相关的手部定制脚本和备份。
- `legacy/revo2_arm_hand_experiments/` 中早期手臂/手部混合实验脚本，仅供追溯，不作为当前推荐流程。
- `models/` 中低于 GitHub 单文件限制的小模型。
- `docs/` 中给用户和 Codex 的恢复说明。

没有上传或不应上传：

- Unitree SDK 本体。
- Revo2 SDK 本体。
- Python/ROS2 构建产物和缓存。
- 完整运行日志/debug 图新快照。
- `yolo_v11x_best.pt` 等大于普通 GitHub 单文件限制的大模型。

这些在本机完整备份中。

## 恢复后建议顺序

1. 恢复 SDK 和 `g1_hand_arm_project`，先跑 `py_compile` 和单元测试。
2. 只测 Revo2 手：`left_hand_safe_once.py open/thumb_ready/bottle/open`。
3. 只测独立脚本手部：`python3 arm_pick_place_standalone.py --hand-only-test`。
4. 小幅空手预抓：`tools/arm_pregrasp_preview.py`。
5. 固定轨迹全流程：`python3 arm_pick_place_standalone.py eth0`。
6. 打开 RGB YOLO：`bash vision/start_bottle_rgb_fast.sh`。
7. 恢复深度和记录点。
8. 准备标定板，做手眼标定。
9. 标定稳定后再考虑路径规划和自动抓取。
