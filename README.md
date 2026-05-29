# Vision-Guided Manipulation System for Unitree G1

这个仓库是当前“宇树 G1 + 强脑 Revo2 灵巧手”手眼协同抓瓶项目的恢复备份。它保存了恢复出厂设置前能直接继续开发的源码、配置、小数据、调试记录和可上传模型；可从网上重新下载的 SDK、ROS2 构建产物、虚拟环境和超过 GitHub 单文件限制的大模型没有放进仓库。

当前阶段不是完整闭环路径规划，而是已经完成了手臂/灵巧手基础联动、YOLO + 深度相机检测、目标点记录和预抓姿态微调。最新主流程使用 `g1_hand_arm_project/arm_pick_place_standalone.py`，该脚本内置 Revo2 左手控制，不再通过 `left_hand_safe_once.py` 间接调用手部动作。

当前固定轨迹流程是：左手自然张开，左臂到固定抓取点，大拇指先执行 `thumb_ready`，随后五指收缩抓瓶；抓住后小臂抬起并悬停 3 秒，再放回原位、张手、空手收回。最新小臂高度参数为 `UNFOLD_PREGRASP_DELTA[18] = -0.70` 和 `LIFT_BOTTLE_DELTA[18] = -1.15`。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `g1_hand_arm_project/` | 当前最重要的固定轨迹抓瓶主项目，包括独立全流程脚本、手臂动作、视觉检测、目标记录、预抓姿态调参和历史备份 |
| `hand_control/revo2/left_hand_safe_once.py` | Revo2 左手定制脚本，只控制手指，不控制手臂 |
| `ros2_vision_workspaces/` | AprilTag/YOLO/深度点云/交互式 TF 标定相关 ROS2 源码，已排除 `build/install/log` |
| `ros2_arm_action/` | G1 手臂 ROS2 action 包源码 |
| `g1_navgrasp/` | G1 NavGrasp / YOLO 导航抓取相关源码，已排除不可上传的 `yolo_v11x_best.pt` |
| `hybrid_control_terminal/` | GUI/深度相机/IK/轨迹控制终端的核心 Python 项目子集 |
| `models/` | 能进 GitHub 的小模型和训练权重，详情见 `models/MODEL_MANIFEST.md` |
| `docs/` | 给人看的说明、恢复步骤、给 Codex 新窗口看的上下文和文件清单 |

## 安全约束

1. `hand_control/revo2/left_hand_safe_once.py` 必须保持为纯手部控制脚本，不要把 `arm_sdk`、手臂轨迹或 DDS 手臂发布写进去。
2. 不要再使用“只写 `motor_cmd[29].q=0` 的空 LowCmd”释放 `arm_sdk`。释放时必须读取当前 `lowstate`，保持当前关节位置，再平滑降低权重。
3. 涉及机器人动作时先小幅、空手、低速测试；不要直接从视觉结果闭环控制手臂。
4. 目前手眼标定还没有完成，视觉检测结果只能作为辅助判断，不应直接当作机械臂目标位姿。

## 当前主线

最关键的主项目在机器人上原本位于：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project
```

主要脚本：

- `arm_grasp_hold.py`：伸手到瓶子附近，抓住后保持，等待触发再张手并收回。
- `arm_pick_place_standalone.py`：当前推荐完整流程，内置 Revo2 左手串口控制、手臂安全初始位检查、抓瓶后抬小臂、悬停 3 秒、放回原位、张手、收回和平滑释放。
- `arm_pick_place.py`：保留版完整流程，仍通过 `left_hand_safe_once.py` 调手，后续不优先扩展。
- `right_hand_thumb_test.py`：只测试右手 `ThumbAux`/大拇指预备动作，不动机械臂。
- `tools/arm_pregrasp_preview.py`：只测试预抓姿态，支持 `--offset 16=-0.42,15=0.06,18=0.36` 这类小幅偏移。
- `vision/detect_bottle_2d.py`：YOLOv8 2D 水瓶检测。
- `vision/detect_bottle_depth.py`：YOLO + 深度相机输出水瓶 3D 信息。
- `vision/record_bottle_positions.py`：记录目标点/随机点。
- `vision/auto_grasp_decision.py`：根据目标点和当前检测结果给出是否适合抓取的判断。

Revo2 左手脚本原路径：

```bash
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

当前左手抓瓶手型：

```python
open = [0, 0, 0, 0, 0, 0]
thumb_ready = [0, 1000, 0, 0, 0, 0]
bottle = [180, 850, 480, 560, 540, 420]
```

数组顺序为 `[thumb, thumb_aux, index, middle, ring, pinky]`。

## 恢复出厂设置后

先看：

- `docs/RESTORE_AFTER_FACTORY_RESET.md`
- `docs/CODEX_PROJECT_CONTEXT.md`
- `docs/EXCLUDED_FILES_MANIFEST.md`

本机不可上传文件备份目录：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260529_195834
```

其中保存了 GitHub 不能直接上传的 `yolo_v11x_best.pt` 大模型。
