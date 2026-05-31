# 未上传到 GitHub 的文件和原因

更新时间：2026-05-31
完整本机备份目录：

```text
E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408
```

## 本机完整备份

| 文件 | 大小 bytes | 内容 |
| --- | ---: | --- |
| `g1_hand_arm_project_full_20260531_151408.tar.gz` | 25,071,256 | 机器人完整主项目，含日志、debug 图、缓存等 |
| `revo2_python_full_20260531_151408.tar.gz` | 90,607 | Revo2 `python/revo2` 完整快照 |
| `handeye_yolo_models_and_dataset_20260531_151408.tar.gz` | 258,935,404 | 水瓶 YOLO 训练 run、数据集、多个相关模型和大模型 |
| `handeye_backup_sha256_20260531_151408.txt` | 357 | 上述压缩包 SHA256 |

## 没进 GitHub 的主要内容

| 内容 | 原因 | 恢复方式 |
| --- | --- | --- |
| `yolo_v11x_best.pt`，约 110MB | 超过普通 GitHub 单文件限制 | 从本机 `handeye_yolo_models_and_dataset_20260531_151408.tar.gz` 解压 |
| `YOLO_Model_Workspace/Datasets/bottle_dataset_v1` 完整数据集 | 训练数据较大，仓库只保留必要小数据和模型 | 从本机压缩包恢复 |
| 新增运行日志和完整 debug 图片 | 运行产物，适合本地归档，不适合继续扩大 Git 历史 | 从 `g1_hand_arm_project_full_20260531_151408.tar.gz` 恢复 |
| Unitree SDK 本体 | 可重新下载 | `git clone https://github.com/unitreerobotics/unitree_sdk2_python.git` |
| Revo2 SDK 本体 | 属于厂商 SDK/示例包 | 从强脑 SDK 包或本机完整快照恢复 |
| ROS2 `build/`, `install/`, `log/` | 可重建 | 恢复源码后 `colcon build` |
| Python `__pycache__`, `.pyc`, 虚拟环境 | 可重建 | 重新安装依赖 |

## 已上传 GitHub 的替代内容

- `g1_hand_arm_project/`：可继续开发的主项目代码、小 JSON/CSV、历史脚本备份。
- `hand_control/revo2/`：本项目相关的 Revo2 手部定制脚本和历史备份，不含完整 SDK。
- `models/`：低于普通 GitHub 单文件限制的小模型，含 `bottle_v12_best.pt`。
- `docs/`：恢复步骤、Codex 上下文、文件说明。

## 恢复大文件

Windows PowerShell：

```powershell
$robot = "unitree@机器人IP"
$backup = "E:\CodexProjects\Unitree_Projects\手眼协同\factory_reset_backup_20260531_151408"
scp "$backup\handeye_yolo_models_and_dataset_20260531_151408.tar.gz" "$robot:/home/unitree/"
```

机器人上：

```bash
cd /home/unitree
tar -xzf handeye_yolo_models_and_dataset_20260531_151408.tar.gz
```
