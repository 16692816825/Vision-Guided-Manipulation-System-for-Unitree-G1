# 项目概况（给自己看）

这个项目是在宇树 G1 上做“视觉辅助抓取水瓶”。硬件包括 G1 左臂、强脑 Revo2 左灵巧手、RGB/深度相机。当前目标是先把固定轨迹抓放和视觉检测稳定下来，再继续做手眼标定、轨迹规划和自动抓取。

## 现在已经做到什么程度

机械臂和灵巧手已经能联动完成固定轨迹流程。当前主脚本是：

```text
g1_hand_arm_project/arm_pick_place_standalone.py
```

它内部已经包含：

- G1 左臂固定轨迹；
- Revo2 左手串口控制；
- 安全初始位检查；
- 抓瓶、抬小臂、悬停 3 秒；
- 抬瓶后大臂外扩；
- 外扩姿态下放小臂并张手；
- 空手收回；
- 平滑释放 `arm_sdk`。

视觉部分已经能打开 RGB YOLO 识别窗口，并且做了目标跟踪，减少识别框跳到其他地方。当前启动脚本是：

```text
g1_hand_arm_project/vision/start_bottle_rgb_fast.sh
```

它使用 `/dev/video4` RGB 相机，分辨率 `640x480`，YOLO 输入 `imgsz 640`，并开启了跟踪参数。

深度和手眼相关代码也已经整理进仓库，包括：

```text
vision/detect_bottle_depth.py
vision/record_bottle_positions.py
vision/compare_bottle_to_target.py
vision/auto_grasp_decision.py
vision/transform_bottle_to_base.py
vision/handeye_g1_head_camera.json
```

## 备份情况

GitHub 仓库里已经保存适合上传的代码、小数据、JSON/CSV 记录、Revo2 定制脚本和小模型。

完整本地备份在：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408
```

这个目录里保存了 GitHub 不适合放的完整项目快照、Revo2 完整目录、YOLO 训练数据和大模型。

恢复出厂后优先看：

```text
docs/RESTORE_AFTER_FACTORY_RESET.md
docs/CODEX_PROJECT_CONTEXT.md
docs/EXCLUDED_FILES_MANIFEST.md
```

## 恢复后建议顺序

1. 重新下载 Unitree SDK 和 Revo2 SDK。
2. 从 GitHub 恢复 `g1_hand_arm_project` 和 `left_hand_safe_once.py`。
3. 从本机备份恢复大模型和训练数据。
4. 先跑 Python 编译和单元测试，不动机器人。
5. 单独测 Revo2 左手。
6. 空手小幅测 G1 左臂预抓姿态。
7. 再测 `arm_pick_place_standalone.py eth0` 固定轨迹全流程。
8. 恢复 YOLO + 深度相机检测。
9. 做 ChArUco 或 AprilTag 手眼标定。
10. 标定稳定后再接路径规划和自动抓取。

在手眼标定完成前，不要把视觉坐标直接拿去控制机械臂。
