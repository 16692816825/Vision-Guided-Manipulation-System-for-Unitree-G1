# Codex 新窗口恢复上下文

请先读完本文件，再修改代码。这个仓库是“宇树 G1 + 强脑 Revo2 左灵巧手 + 视觉辅助抓瓶”项目的恢复备份，用户希望恢复出厂设置后继续做自动化流程。

## 用户沟通要求

- 用中文回答。
- 直接、务实、工程化，优先给可执行命令和明确风险。
- 需要用户操作机器人、相机、标定板或物理环境时再问；其余文件整理、代码检查、文档和普通命令尽量直接做。
- 给学习建议时必须附链接。

## 当前机器人与路径

最近可用 SSH 地址：

```bash
ssh unitree@10.88.2.69
```

旧地址曾经包括 `10.218.112.89`、`10.218.112.69`、`192.168.123.164`。恢复出厂设置后地址可能变化，先以现场网络为准。

机器人恢复前的关键路径：

```text
/home/unitree/unitree_sdk2_python/g1_hand_arm_project
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
/home/unitree/detect/ros2_ws/src
/home/unitree/detect1/ros2_ws/src
/home/unitree/detect2/ros2_ws/src
/home/unitree/detect3/ros2_ws/src
/home/unitree/g1act_ws/manact_ws
/home/unitree/g1act_ws/g1act_ws
/home/unitree/G1_Docker_Deploy/python_gui
/home/unitree/g1_arm_ws/src/g1_arm_action_ros2
```

本机仓库：

```text
E:\CodexProjects\Unitree_Projects\Vision-Guided-Manipulation-System-for-Unitree-G1
```

本机不可上传备份：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260529_195834
```

## 强制安全约束

1. 不要把手臂 `arm_sdk` 控制写进 `hand_control/revo2/left_hand_safe_once.py`。这个脚本只能控制 Revo2 手指。
2. 不要再写或运行“只设置 `motor_cmd[29].q=0` 的空 LowCmd 释放脚本”。之前这样释放导致手臂乱甩和电机卡顿声。
3. 释放 `arm_sdk` 必须读取当前 `lowstate`，保持当前关节 `q/kp/kd`，再平滑降低 `WEIGHT=29`。
4. 涉及机器人动作时先小幅测试、空手测试、低速测试。没有完成手眼标定前，不要让视觉结果直接闭环控制手臂。
5. 用户当前更想推进自动化流程，但仍要把“视觉判断”和“手臂动作”分阶段验证。

## G1 手臂控制关节

当前按 G1 23DoF 场景使用：

```python
LEFT_ARM = [15, 16, 17, 18, 19]
OTHER_ARM = [22, 23, 24, 25, 26]
WEIGHT = 29
DT = 0.02
KP = 20.0
KD = 1.0
```

已知含义：

- `15`: LeftShoulderPitch，影响大臂前后/上抬。
- `16`: LeftShoulderRoll，影响左右外扩/内收。
- `18`: LeftElbow，影响小臂折叠/伸开。
- `19`: LeftWristRoll，影响手腕旋转。

最近一次预抓预览演示用偏移：

```text
15=+0.06
16=-0.42
18=+0.36
```

对应命令形式：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 tools/arm_pregrasp_preview.py --net eth0 --offset 16=-0.42,15=0.06,18=0.36 --max-abs-offset 0.45 --hold-seconds 45 --yes
```

`tools/arm_pregrasp_preview.py` 只预览手臂预抓姿态，不闭合手，不拿瓶子。闭合手需要外部单独调用 Revo2 手脚本。

当前独立全流程脚本：

```text
g1_hand_arm_project/arm_pick_place_standalone.py
```

这是后续优先维护的固定轨迹抓放脚本。它内置 Revo2 左手串口控制，不通过 `left_hand_safe_once.py`，流程为：

```text
open -> 到固定抓取点 -> thumb_open_max -> thumb_ready -> bottle
-> 小臂抬瓶 -> 悬停 3 秒 -> 放回原位 -> open -> 空手收回 -> 平滑释放 arm_sdk
```

当前为了先验证完整抓瓶流程，`thumb_ready` 反馈不到位时只报警，不再中止流程，会继续执行 `bottle` 五指抓握。后续仍需要单独排查左手 `ThumbAux` 通道。

最新小臂高度参数：

```python
UNFOLD_PREGRASP_DELTA = {18: -1.35, ...}
LIFT_BOTTLE_DELTA = {18: -1.80, ...}
```

调参规律：`18` 更小会让小臂更往上折，`18` 更大则小臂更低。后续如果只想调固定点和抬瓶高度，优先改这两个字典里的 `18`。

## Revo2 左手控制

定制脚本在仓库：

```text
hand_control/revo2/left_hand_safe_once.py
```

恢复到机器人后应放回：

```text
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

关键配置：

```python
PORT = "/dev/ttyUSB1"
SLAVE_ID = 0x7e
BAUDRATE = libstark.Baudrate.Baud460800
```

手型数组顺序：

```text
[thumb, thumb_aux, index, middle, ring, pinky]
```

当前抓瓶手型：

```python
"open": [0, 0, 0, 0, 0, 0]
"thumb_open_max": [0, 0, 0, 0, 0, 0]
"thumb_ready": [0, 1000, 0, 0, 0, 0]
"bottle": [180, 850, 480, 560, 540, 420]
```

`grasp` 的动作顺序：

```text
thumb_open_max -> wait -> thumb_ready -> wait -> bottle
```

左手 `ThumbAux` 曾出现“反馈为 1000，但肉眼拇指动作不明显”的现象。右手诊断脚本为：

```text
g1_hand_arm_project/right_hand_thumb_test.py
```

右手配置为 `/dev/ttyUSB2`、`SLAVE_ID=0x7f`，只测试右手 `open -> thumb_ready -> bottle -> open`，不动机械臂。

## 视觉与自动化状态

当前视觉主线在：

```text
g1_hand_arm_project/vision/
```

重要文件：

- `detect_bottle_2d.py`: YOLOv8 2D 检测水瓶，输出 `cx/cy/confidence`，保存 debug 图。
- `detect_bottle_depth.py`: YOLO + 深度相机，输出水瓶 3D 信息。
- `record_bottle_positions.py`: 记录位置点。
- `compare_bottle_to_target.py`: 当前瓶子与目标抓取参考比较。
- `evaluate_grasp_window.py`: 判断是否在抓取窗口内。
- `auto_grasp_decision.py`: 输出是否建议抓取和偏差。
- `target_grasp_reference.json`: 目标抓取参考。
- `records/*.csv`: 位置 1-4、目标点、随机点 1-3。

现在还没有完成正式手眼标定。下一步应该先恢复检测和深度输出，再准备标定板做相机坐标到机器人坐标的转换。

## GitHub/本机备份策略

本仓库已经上传或准备上传：

- 抓瓶主项目源码、小数据、日志和 debug 图。
- Revo2 左手定制脚本。
- ROS2 视觉/标定源码。
- G1 NavGrasp 源码和可上传小模型。
- Hybrid control terminal 核心 Python 源码。
- 可上传的 `.pt` 小模型。

没有上传：

- `unitree_sdk2_python` SDK 本体，可从 Unitree 官方仓库重新下载。
- `stark-serialport-example` SDK 本体，可从强脑/本地备份恢复，仓库只保留定制手脚本。
- ROS2 `build/install/log`。
- 虚拟环境、缓存。
- `yolo_v11x_best.pt`，单文件约 114MB，超过普通 GitHub 单文件限制，已放本机不可上传目录。

## 恢复后优先级

1. 恢复 SDK 和项目目录，先跑 Python 单元测试，不动机器人。
2. 验证 `left_hand_safe_once.py open/grasp`，只控制手。
3. 验证 `arm_pick_place_standalone.py --hand-only-test`，确认手型顺序。
4. 验证 `tools/arm_pregrasp_preview.py --dry-run` 和小幅空手预览。
5. 固定轨迹安全后，再跑 `arm_pick_place_standalone.py eth0`。
6. 恢复 YOLO + 深度相机检测，确认 `latest_bottle_3d.json` 正常生成。
7. 准备 ChArUco 或 AprilTag 标定板做手眼标定。
8. 标定稳定后再做轨迹规划和自动抓取。
