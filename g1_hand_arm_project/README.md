# G1 左臂 + 强脑 Revo2 左手抓瓶项目脚本说明

这个文件夹保存的是当前用于 **Unitree G1 左臂 + BrainCo/Revo2 左手抓取桌面水瓶** 的整理版脚本。

当前项目核心目标是：

1. 让 G1 左臂安全伸到桌面水瓶附近。
2. 让 Revo2 左手按正确顺序完成拇指准备和五指抓握。
3. 抓住水瓶后收回，避免撞桌子和大腿。
4. 后续完整流程中，再伸出、张手放回水瓶、空手收回。

## 目录位置

机器人上的项目目录：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project
```

Windows 桌面副本：

```text
D:\桌面\g1_hand_arm_project
```

左手 Revo2 原始控制脚本仍然保留在：

```bash
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

## 当前文件结构

```text
g1_hand_arm_project/
├── arm_grasp_hold.py
├── arm_pick_place.py
├── README.md
├── manifest.txt
├── tools/
│   ├── arm_joint_probe.py
│   ├── arm_sdk_hold_test.py
│   └── left_wrist_roll_test.py
└── archive_raw/
    └── 历史脚本和旧参数备份
```

## 为什么这样整理

现在采用“手臂脚本”和“手部脚本”分离的结构。

这样做的原因是：之前曾经把大臂前送逻辑写进 `left_hand_safe_once.py`，导致一个本来只应该控制手指的脚本也去接管 `rt/arm_sdk`，结果出现大臂下砸桌面的危险动作。

现在的原则是：

```text
left_hand_safe_once.py 只控制手，不控制大臂。
arm_grasp_hold.py / arm_pick_place.py 统一控制手臂，并按时序调用手部脚本。
```

这样可以避免：

1. 手部脚本和手臂脚本同时抢 `arm_sdk` 控制权。
2. 在错误姿态下只改某一个肩部关节，导致手臂下砸。
3. 调手型时误触发手臂运动。

## 主脚本 1：arm_grasp_hold.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/arm_grasp_hold.py
```

用途：**调试抓瓶姿态的主脚本**。

它会让手臂伸到瓶子位置，完成抓握，然后保持不动。你可以在保持阶段继续调手型、观察瓶子是否倾斜、判断拇指/食指/小指是否过紧。

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_grasp_hold.py eth0
```

流程：

```text
1. 调用左手 open，让手张开。
2. arm_sdk 接管双臂。
3. 左臂先折小臂，缩短手臂半径。
4. 大臂带着折叠小臂向目标方向移动。
5. 小臂下放到瓶子侧方。
6. 调用左手 thumb_open_max。
7. 调用左手 thumb_ready。
8. 调用左手 bottle，完成抓瓶。
9. 保持抓瓶姿态不动。
10. 等待触发命令。
11. 收到触发命令后，左手 open。
12. 手臂折回并收回。
13. release arm_sdk。
```

触发“张手并收回”的命令：

```bash
touch /tmp/g1_open_retract.flag
```

推荐用途：

```text
调瓶子抓握姿态时优先用这个脚本。
不要一开始就跑完整放回流程。
```

## 主脚本 2：arm_pick_place.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/arm_pick_place.py
```

用途：**完整抓取 + 放回流程**。

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place.py eth0
```

流程：

```text
1. 左手 open。
2. arm_sdk 接管双臂。
3. 手臂伸到瓶子位置。
4. 左手 thumb_open_max。
5. 左手 thumb_ready。
6. 左手 bottle 抓瓶。
7. 抓住后大臂外扩，带瓶子避开身体和大腿。
8. 折小臂并收回。
9. 手臂再次伸到放瓶位置。
10. 左手 open，放开瓶子。
11. 空手折回并收回。
12. release arm_sdk。
```

推荐用途：

```text
只有当 arm_grasp_hold.py 已经验证安全后，再跑 arm_pick_place.py。
```

## 左手脚本：left_hand_safe_once.py

路径：

```bash
/home/unitree/stark-serialport-example/python/revo2/left_hand_safe_once.py
```

用途：**只控制左手，不控制手臂**。

支持命令：

```text
open
thumb_open_max
thumb_ready
bottle
grasp
```

运行示例：

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
python3 left_hand_safe_once.py thumb_ready
python3 left_hand_safe_once.py bottle
python3 left_hand_safe_once.py grasp
```

当前手型数组顺序：

```text
[thumb, thumb_aux, index, middle, ring, pinky]
```

当前参数：

```python
"open": [0, 0, 0, 0, 0, 0]
"thumb_open_max": [0, 0, 0, 0, 0, 0]
"thumb_ready": [0, 1000, 0, 0, 0, 0]
"bottle": [180, 850, 480, 560, 540, 420]
```

`grasp` 的内部流程：

```text
thumb_open_max -> 等待 1 秒
thumb_ready -> 等待 2 秒
bottle -> 等待 2 秒
```

各参数含义：

```text
thumb      大拇指弯曲/内扣程度
thumb_aux  大拇指侧摆/对掌程度
index      食指弯曲程度
middle     中指弯曲程度
ring       无名指弯曲程度
pinky      小指弯曲程度
```

当前重要原则：

```text
left_hand_safe_once.py 不允许加入 rt/arm_sdk。
它只能控制手，不能控制大臂。
```

## 工具脚本：tools/arm_joint_probe.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools/arm_joint_probe.py
```

用途：单独测试某一个手臂关节的正负方向。

运行示例：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools
python3 arm_joint_probe.py --net eth0 --joint 15 --delta 0.03
```

流程：

```text
1. 读取当前双臂位置。
2. arm_sdk 接管。
3. 只移动指定关节一点点。
4. 保持短时间。
5. 回到初始位置。
6. release arm_sdk。
```

适合用来确认：

```text
15 号肩部 pitch 的正负方向
16 号肩部 roll 的正负方向
18 号小臂/肘部方向
19 号腕部 roll 方向
```

## 工具脚本：tools/arm_sdk_hold_test.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools/arm_sdk_hold_test.py
```

用途：只测试 `arm_sdk` 接管和释放，不主动改变手臂姿态。

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools
python3 arm_sdk_hold_test.py eth0
```

流程：

```text
1. 读取当前手臂位置。
2. arm_sdk 权重从 0 慢慢升到 1。
3. 保持当前姿态 2 秒。
4. arm_sdk 权重从 1 慢慢降到 0。
```

用途：

```text
如果这个脚本都出现明显抖动、乱甩或卡顿声，说明当前机器人模式不适合继续跑低层手臂控制。
```

## 工具脚本：tools/left_wrist_roll_test.py

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools/left_wrist_roll_test.py
```

用途：测试左腕 19 号关节旋转方向。

运行：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools
python3 left_wrist_roll_test.py eth0
```

当前测试关节：

```python
TEST_JOINT = 19
DELTA = 0.25
```

如果想反方向测试，把 `DELTA` 改成负数。

## archive_raw 目录

路径：

```bash
/home/unitree/unitree_sdk2_python/g1_hand_arm_project/archive_raw
```

用途：旧脚本和历史参数备份。

这里面的文件不作为当前主流程使用。

它的作用是：

```text
1. 回滚旧参数。
2. 查看历史版本。
3. 对比之前调过的姿态。
```

不建议直接运行 `archive_raw` 里的脚本。

## 推荐调试顺序

### 1. 只测试左手是否正常

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
python3 left_hand_safe_once.py grasp
```

### 2. 测试 arm_sdk 接管是否稳定

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project/tools
python3 arm_sdk_hold_test.py eth0
```

### 3. 调试抓瓶并保持

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_grasp_hold.py eth0
```

触发张手并收回：

```bash
touch /tmp/g1_open_retract.flag
```

### 4. 最后跑完整抓放

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place.py eth0
```

## 安全注意事项

1. 桌面、瓶子、机器人手臂周围先清空。
2. 初次测试不要放满瓶水，先用空瓶或半瓶。
3. 如果动作异常，立即 `Ctrl+C`。
4. 如果手臂已经有撞桌或卡顿声，不要继续发低层关节命令。
5. 不要把手臂控制代码写进 `left_hand_safe_once.py`。
6. 不要运行 `revo2_cfg.py`，它可能修改设备配置。
7. 不要用旧的 `revo2_ctrl_left.py` / `revo2_ctrl_right.py` 做当前项目，它们是循环示例，速度较大。

## 当前最常用命令

调试抓瓶：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_grasp_hold.py eth0
```

张手并收回：

```bash
touch /tmp/g1_open_retract.flag
```

单独张开左手：

```bash
cd /home/unitree/stark-serialport-example/python/revo2
python3 left_hand_safe_once.py open
```

完整抓放：

```bash
cd /home/unitree/unitree_sdk2_python/g1_hand_arm_project
python3 arm_pick_place.py eth0
```
