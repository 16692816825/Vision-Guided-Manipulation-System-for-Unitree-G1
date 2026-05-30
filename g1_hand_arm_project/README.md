# G1 左臂 + Revo2 灵巧手抓瓶项目说明

这个目录保存当前用于 **Unitree G1 左臂 + 强脑 Revo2 灵巧手抓取桌面水瓶** 的整理版脚本。当前阶段仍是固定轨迹和人工调参，不是视觉闭环控制机械臂。

当前目标动作是：

```text
左手自然张开 -> 左臂到固定抓取点 -> 大拇指先到预备/垂直位置 -> 五指收缩抓瓶
-> 小臂抬起瓶子 -> 悬停 3 秒 -> 放回原位 -> 张手 -> 空手收回 -> 平滑释放 arm_sdk
```

## 路径

机器人项目目录：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project
```

Revo2 左手单独测试脚本仍在：

```bash
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

## 当前文件结构

```text
g1_hand_arm_project/
├── arm_pick_place_standalone.py   # 当前推荐的独立全流程脚本
├── arm_grasp_hold.py              # 抓住后保持，等待 flag 再张手收回
├── arm_pick_place.py              # 保留版完整流程，仍通过 left_hand_safe_once.py 调手
├── right_hand_thumb_test.py       # 只测右手 ThumbAux/大拇指预备动作
├── test_arm_pick_place_flow.py    # 静态流程约束测试
├── vision/                        # YOLO/深度相机/抓取窗口判断
├── tools/                         # 关节小幅测试、arm_sdk 接管测试等
└── archive_raw/                   # 历史脚本和旧参数备份
```

## 关键安全原则

1. `left_hand_safe_once.py` 只控制手，不允许写入 `rt/arm_sdk` 或机械臂轨迹。
2. 不要再用只写 `motor_cmd[29].q = 0` 的空 `LowCmd` 释放脚本。
3. 释放 `arm_sdk` 时必须保持当前关节 `q/kp/kd`，再平滑降低 `WEIGHT=29`。
4. 视觉检测暂时只用于观察和判断，不直接闭环控制机械臂。
5. 机器人动作测试前先确认桌面、瓶子、线缆和人手都离开运动范围。

## 当前推荐脚本：arm_pick_place_standalone.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/arm_pick_place_standalone.py
```

用途：当前推荐的独立抓放流程脚本。它不再通过 `subprocess` 调用 `left_hand_safe_once.py`，而是在脚本内部直接连接 Revo2 左手。

左手配置：

```python
HAND_PORT = "/dev/ttyUSB1"
HAND_SLAVE_ID = 0x7E
```

手型数组顺序：

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

当前左手 `thumb_ready` 反馈可能读不到预期值。全流程脚本会打印警告，但不会因此中止，会继续执行 `bottle` 五指抓握；这只是为了先验证完整抓瓶流程，后续仍应单独排查 `ThumbAux` 通道。

流程：

```text
1. 等待用户按回车。
2. 初始化 DDS，读取 lowstate。
3. 连接 Revo2 左手并张手。
4. arm_sdk 接管双臂。
5. 检查是否接近安全初始位；不接近时先缓慢回安全位，过远则停止。
6. 折小臂，送大臂，到固定抓取点。
7. 执行 thumb_open_max -> thumb_ready -> bottle。
8. 抓住后小臂抬起，悬停 3 秒。
9. 小臂放回抓取点，张手释放。
10. 空手折回、外扩避让、回初始姿态。
11. 平滑释放 arm_sdk，关闭手部串口。
```

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place_standalone.py eth0
```

只测左手、不动机械臂：

```bash
python3 arm_pick_place_standalone.py --hand-only-test
```

当前小臂高度参数：

```python
UNFOLD_PREGRASP_DELTA[18] = -1.35
LIFT_BOTTLE_DELTA[18] = -1.80
```

调参规律：

```text
UNFOLD_PREGRASP_DELTA[18] 控制到瓶子固定点时的小臂高度。
LIFT_BOTTLE_DELTA[18] 控制抓住后抬瓶时的小臂高度。
18 的值更小，例如 -1.45 / -1.90，小臂更往上折。
18 的值更大，例如 -1.25 / -1.70，小臂会低一些。
```

## 调抓瓶姿态：arm_grasp_hold.py

用途：伸到瓶子位置，抓住后保持不动，方便观察手型和接触位置。

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_grasp_hold.py eth0
```

让它张手并收回：

```bash
touch /tmp/g1_open_retract.flag
```

不要直接用 `Ctrl+C` 当作正常收回方式。脚本在收到 flag 后会先张手，再按原路径收回并平滑释放 `arm_sdk`。

## 保留版完整流程：arm_pick_place.py

这个脚本仍然保留，但它不是当前优先维护版本。它通过 `left_hand_safe_once.py` 调用手部动作，适合对照旧流程，不建议作为后续自动化主脚本继续扩展。

后续新增流程优先改：

```text
arm_pick_place_standalone.py
```

不要再把新逻辑分散到 `left_hand_safe_once.py` 里。

## 右手诊断脚本：right_hand_thumb_test.py

用途：只测试右手 Revo2 的 `ThumbAux` 通道，判断“大拇指预备/垂直”动作在右手上是否正常。不动机械臂。

右手配置：

```python
HAND_PORT = "/dev/ttyUSB2"
HAND_SLAVE_ID = 0x7F
```

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 right_hand_thumb_test.py
```

如果串口偶发超时：

```bash
python3 right_hand_thumb_test.py --connect-retries 5 --connect-retry-delay 1.0
```

## left_hand_safe_once.py 的定位

路径：

```bash
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

用途：只做左手单独测试。

常用命令：

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
python3 left_hand_safe_once.py thumb_ready
python3 left_hand_safe_once.py bottle
python3 left_hand_safe_once.py grasp
```

它不能控制手臂，不能写入 `rt/arm_sdk`。

## 工具脚本

`tools/arm_sdk_hold_test.py`：只测试 `arm_sdk` 接管和释放，不主动改变手臂姿态。

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools
python3 arm_sdk_hold_test.py eth0
```

`tools/arm_joint_probe.py`：单独小幅测试某个关节方向。

```bash
python3 arm_joint_probe.py --net eth0 --joint 18 --delta 0.03
```

`tools/arm_pregrasp_preview.py`：预览预抓姿态，不闭合手。

```bash
python3 tools/arm_pregrasp_preview.py --net eth0 --offset 16=-0.42,15=0.06,18=0.36 --max-abs-offset 0.45 --hold-seconds 10
```

## 视觉脚本

视觉目前用于检测和判断，不直接控制手臂。

常用文件：

```text
vision/detect_bottle_2d.py
vision/detect_bottle_depth.py
vision/record_bottle_positions.py
vision/compare_bottle_to_target.py
vision/evaluate_grasp_window.py
vision/auto_grasp_decision.py
```

下一步如果继续自动化，应先完成稳定检测、深度输出和手眼标定，再把视觉目标接入轨迹规划。

## 测试

不动机器人，只做静态和语法验证：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 -m py_compile arm_pick_place_standalone.py right_hand_thumb_test.py test_arm_pick_place_flow.py
python3 -m unittest test_arm_pick_place_flow.py
```

## 推荐调试顺序

1. 只测 Revo2 左手：`left_hand_safe_once.py open/grasp` 或 `arm_pick_place_standalone.py --hand-only-test`。
2. 测 `arm_sdk` 接管释放：`tools/arm_sdk_hold_test.py eth0`。
3. 用 `arm_grasp_hold.py` 调抓瓶姿态。
4. 瓶子位置和环境确认后，再跑 `arm_pick_place_standalone.py eth0`。
5. 视觉检测稳定和手眼标定完成前，不让视觉结果直接驱动机械臂。
