# 机器人恢复出厂设置后的恢复步骤

下面步骤默认机器人用户名为 `unitree`，网络接口仍用 `eth0`，机器人 IP 以现场实际地址为准。涉及机器人动作前必须先确认周围安全。

## 1. 重新安装可下载 SDK

Unitree Python SDK：

```bash
cd /home/unitree
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

Revo2 手部 SDK/示例需要按强脑 Revo2 的原始包恢复到类似路径：

```text
/home/unitree/stark-serialport-example/python/revo2
```

如果恢复后路径不同，要同步修改抓瓶脚本里的 `HAND_DIR`。

当前推荐的独立全流程脚本 `arm_pick_place_standalone.py` 不使用 `HAND_DIR`，而是直接用：

```python
HAND_PORT = "/dev/ttyUSB1"
HAND_SLAVE_ID = 0x7E
```

如果恢复后串口变化，优先检查并修改这个脚本里的 `HAND_PORT`。

## 2. 上传本仓库项目文件

在 Windows 本机执行，按实际 IP 替换：

```powershell
$robot = "unitree@10.88.2.69"
$repo = "E:\CodexProjects\Unitree_Projects\Vision-Guided-Manipulation-System-for-Unitree-G1"

scp -r "$repo\g1_hand_arm_project" "$robot:/home/unitree/unitree_sdk2_python/"
scp "$repo\hand_control\revo2\left_hand_safe_once.py" "$robot:/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py"
```

ROS2 视觉/标定源码按需要恢复：

```powershell
scp -r "$repo\ros2_vision_workspaces\detect\src"  "$robot:/home/unitree/detect/ros2_ws/"
scp -r "$repo\ros2_vision_workspaces\detect1\src" "$robot:/home/unitree/detect1/ros2_ws/"
scp -r "$repo\ros2_vision_workspaces\detect2\src" "$robot:/home/unitree/detect2/ros2_ws/"
scp -r "$repo\ros2_vision_workspaces\detect3\src" "$robot:/home/unitree/detect3/ros2_ws/"
```

大模型从本机备份恢复，不从 GitHub 恢复：

```powershell
$backup = "E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260529_195834\not_uploaded_large_files\models"
scp "$backup\g1_navgrasp_yolo_v11x_best.pt" "$robot:/home/unitree/g1act_ws/manact_ws/src/g1_yolo_nav_py/yolo_v11x_best.pt"
scp "$backup\g1act_ws_yolo_v11x_best.pt" "$robot:/home/unitree/g1act_ws/g1act_ws/src/g1_yolo_nav_py/yolo_v11x_best.pt"
```

## 3. 验证不动机器人部分

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 -m py_compile arm_pick_place_standalone.py right_hand_thumb_test.py test_arm_pick_place_flow.py
python3 -m unittest test_arm_pick_place_flow.py
python3 -m unittest tools/test_arm_pregrasp_preview.py
python3 -m unittest vision/test_detect_bottle_2d.py
python3 tools/arm_pregrasp_preview.py --dry-run --offset 16=-0.02 --max-abs-offset 0.08
```

如果缺 Python 包，先按报错补装，常见依赖包括：

```bash
pip3 install numpy opencv-python ultralytics
```

深度相机相关可能需要 `pyrealsense2` 和 RealSense 系统驱动，不能只靠 `pip` 保证。

## 4. 验证 Revo2 左手

先只测手，不测手臂：

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
python3 left_hand_safe_once.py thumb_ready
python3 left_hand_safe_once.py bottle
python3 left_hand_safe_once.py open
```

确认串口仍然是：

```text
/dev/ttyUSB1
```

如果变了，先用 `ls /dev/ttyUSB*` 查，再修改 `left_hand_safe_once.py` 的 `PORT`。

也可以用独立全流程脚本的手部测试入口，只动左手、不动机械臂：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place_standalone.py --hand-only-test
```

当前全流程内置手型顺序是：

```text
open -> thumb_open_max -> thumb_ready -> bottle
```

注意：当前版本里 `thumb_ready` 反馈不到位时只报警，不会中止流程，会继续执行 `bottle` 五指抓握。这样是为了优先验证完整固定轨迹，后续仍需要单独排查左手 `ThumbAux` 通道。

如果怀疑左手 `ThumbAux` 通道异常，可以只测右手诊断脚本：

```bash
python3 right_hand_thumb_test.py --connect-retries 5 --connect-retry-delay 1.0
```

## 5. 验证手臂空手预抓

确认 G1 已进入稳定站立/预备状态，周围没有人和障碍物。先用很小偏移：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 tools/arm_pregrasp_preview.py --net eth0 --offset 16=-0.02,15=0.02,18=0.02 --max-abs-offset 0.08 --hold-seconds 3
```

确认安全后，再恢复最近一次人工演示偏移：

```bash
python3 tools/arm_pregrasp_preview.py --net eth0 --offset 16=-0.42,15=0.06,18=0.36 --max-abs-offset 0.45 --hold-seconds 10
```

## 6. 继续自动化流程

推荐顺序：

1. 先验证 `arm_pick_place_standalone.py eth0` 的固定轨迹流程，确认安全初始位、抓瓶、抬小臂、悬停、放回和收手都正常。
2. 再恢复 `vision/detect_bottle_2d.py`，确认水瓶框稳定。
3. 再恢复 `vision/detect_bottle_depth.py`，确认深度有效。
4. 记录新的目标抓取点和随机点。
5. 做 ChArUco 或 AprilTag 手眼标定。
6. 标定稳定后再接机械臂路径规划，不要直接用未标定相机坐标控制手臂。

当前 `arm_pick_place_standalone.py` 里和小臂高度最相关的参数：

```python
UNFOLD_PREGRASP_DELTA[18] = -1.35
LIFT_BOTTLE_DELTA[18] = -1.80
```

如果固定抓取点或抬瓶姿态还需要再高一点，就把对应的 `18` 再调小；如果太高，就把它调大。
