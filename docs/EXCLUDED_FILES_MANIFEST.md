# 未上传到 GitHub 的文件和原因

本仓库尽量保存了继续项目所需的源码、小数据、配置和可上传模型。以下内容没有上传。

## 超过普通 GitHub 单文件限制的大模型

已复制到本机：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260529_195834\not_uploaded_large_files\models
```

| 文件 | 原机器人路径 | 大小 | 处理方式 |
| --- | --- | ---: | --- |
| `g1_navgrasp_yolo_v11x_best.pt` | `/home/unitree/g1act_ws/manact_ws/src/g1_yolo_nav_py/yolo_v11x_best.pt` | 114,404,889 bytes | 本机保存，不进 GitHub |
| `g1act_ws_yolo_v11x_best.pt` | `/home/unitree/g1act_ws/g1act_ws/src/g1_yolo_nav_py/yolo_v11x_best.pt` | 114,404,889 bytes | 本机保存，不进 GitHub |

`/home/unitree/g1act_ws/manact_ws/install/.../yolo_v11x_best.pt` 属于安装/构建输出中的重复副本，没有单独保存。

## 可重新下载或重建的内容

| 内容 | 原因 |
| --- | --- |
| `/home/unitree/unitree_sdk2_python` SDK 本体 | 可从 Unitree 官方 GitHub 重新下载；仓库只保存项目目录 `g1_hand_arm_project/` |
| `/home/unitree/stark-serialport-example` SDK 本体 | 属于 Revo2 SDK/示例包；仓库只保存定制脚本 `left_hand_safe_once.py` |
| ROS2 `build/`, `install/`, `log/` | 可由 `colcon build` 重新生成 |
| Python 虚拟环境、`__pycache__`、`.pyc` | 可重新生成 |
| RealSense ROS2 SDK 工作区 | 可按 Intel RealSense 官方 ROS2 包重新安装 |

## 上传前检查

本仓库已按以下原则过滤：

- 没有上传大于 100MB 的单文件。
- 没有上传 ROS2 构建产物。
- 没有上传 Python 缓存。
- 小模型 `.pt` 文件保留在 `models/`，用于后续快速恢复检测。
