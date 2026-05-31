# Vision-Guided Manipulation System for Unitree G1

这个仓库是“宇树 G1 + 强脑 Revo2 灵巧手 + RGB/深度相机”手眼协同抓瓶项目的恢复备份。当前文件以机器人 `unitree@10.88.2.69` 在 2026-05-31 的状态为基准整理。

仓库保存适合上传 GitHub 的源码、小数据、标定/记录 JSON/CSV、Revo2 定制脚本和小模型。完整机器人快照、训练数据集和超过 GitHub 普通限制的大模型放在本机：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408
```

## 当前状态

当前还不是视觉闭环路径规划。已经完成：

- G1 左臂固定轨迹抓瓶、抬瓶、外扩、下放、张手、收回。
- Revo2 左手手型联动，主流程已内置手部串口控制。
- YOLOv8 RGB 水瓶检测，增加了目标跟踪，减少识别框跳到其他地方。
- 深度检测、目标点/随机点记录、相机点到 G1 base 的初步转换脚本。

当前主流程脚本：

```text
g1_hand_arm_project/arm_pick_place_standalone.py
```

当前视觉启动脚本：

```text
g1_hand_arm_project/vision/start_bottle_rgb_fast.sh
```

视觉窗口默认参数：

```text
/dev/video4 RGB, V4L2 backend, 640x480, YOLO imgsz 640
conf=0.15, infer-every=2
tracking: max_jump=80px, smooth_alpha=0.35, lock_conf=0.25
```

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `g1_hand_arm_project/` | 当前最重要的固定轨迹抓瓶主项目 |
| `g1_hand_arm_project/vision/` | YOLO、深度、记录点、抓取窗口判断、坐标转换 |
| `hand_control/revo2/` | Revo2 左手定制脚本和历史备份，不含完整 SDK |
| `legacy/revo2_arm_hand_experiments/` | 早期手臂/手部混合实验脚本，仅作历史参考，不作为当前流程运行 |
| `models/` | 可上传的小模型，详情见 `models/MODEL_MANIFEST.md` |
| `docs/RESTORE_AFTER_FACTORY_RESET.md` | 给用户看的恢复步骤 |
| `docs/CODEX_PROJECT_CONTEXT.md` | 给新 Codex 对话读取的上下文 |
| `docs/EXCLUDED_FILES_MANIFEST.md` | 没上传 GitHub 的本机备份内容 |

## 安全约束

1. `hand_control/revo2/left_hand_safe_once.py` 只允许控制手指，不要写入 `arm_sdk` 或机械臂轨迹。
2. 不要再使用只写 `motor_cmd[29].q = 0` 的空 `LowCmd` 释放脚本。
3. 释放 `arm_sdk` 必须保持当前关节 `q/kp/kd`，再平滑降低 `WEIGHT=29`。
4. 涉及机器人动作时先空手、小幅、低速测试。
5. 手眼标定稳定前，不要让视觉结果直接闭环控制机械臂。

## 恢复后先看

恢复出厂设置后，先读：

```text
docs/RESTORE_AFTER_FACTORY_RESET.md
docs/CODEX_PROJECT_CONTEXT.md
docs/EXCLUDED_FILES_MANIFEST.md
```

然后按顺序做：

1. 重新下载 Unitree SDK 和 Revo2 SDK。
2. 从 GitHub 恢复 `g1_hand_arm_project/` 和 `hand_control/revo2/left_hand_safe_once.py`。
3. 从本机备份恢复大模型和完整训练数据。
4. 先跑 `py_compile` 和单元测试，不动机器人。
5. 先测手，再测空手预抓，再测完整固定轨迹。
6. 最后恢复 YOLO/深度相机和手眼标定流程。
