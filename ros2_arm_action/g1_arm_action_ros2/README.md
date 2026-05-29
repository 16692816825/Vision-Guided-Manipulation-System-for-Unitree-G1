# G1 Arm Action ROS2 Package

ROS2 功能包，用于控制宇树 G1 人形机器人的手臂动作。

## 概述

本功能包将 `unitree_sdk2` 的 `G1ArmActionClient` 封装为 ROS2 服务接口，方便在 ROS2 生态系统中控制 G1 机器人的手臂动作。

## 功能

- 执行预设手臂动作（通过动作 ID）
- 执行自定义示教动作（通过动作名称）
- 停止当前自定义动作
- 获取可用动作列表

## 依赖

- ROS2 Foxy
- unitree_sdk2
- CycloneDDS

## 编译

### 1. 设置 unitree_sdk2 路径

确保 `unitree_sdk2` 已正确安装或位于可访问的路径。默认情况下，CMakeLists.txt 假设 `unitree_sdk2` 位于同级目录。

如需修改路径，可以在编译时指定：

```bash
colcon build --cmake-args -DUNITREE_SDK2_PATH=/path/to/unitree_sdk2
```

### 2. 编译功能包

```bash
cd ~/ros2_ws
colcon build --packages-select g1_arm_action_ros2
source install/setup.bash
```

## 使用方法

### 启动节点

**基本启动：**
```bash
ros2 launch g1_arm_action_ros2 g1_arm_action.launch.py
```

**指定网络接口：**
```bash
ros2 launch g1_arm_action_ros2 g1_arm_action.launch.py network_interface:=eth0
```

**使用配置文件：**
```bash
ros2 run g1_arm_action_ros2 g1_arm_action_node --ros-args --params-file config/arm_action_params.yaml
```

### 服务接口

#### 1. 执行动作服务 `~/execute_arm_action`

**服务类型：** `g1_arm_action_ros2/srv/ExecuteArmAction`

**请求字段：**
| 字段 | 类型 | 说明 |
|------|------|------|
| command_type | uint8 | 命令类型：0=执行预设动作, 1=执行自定义动作, 2=停止动作 |
| action_id | int32 | 预设动作 ID（command_type=0 时使用） |
| action_name | string | 自定义动作名称（command_type=1 时使用） |

**响应字段：**
| 字段 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否执行成功 |
| error_code | int32 | 错误码（0=成功） |
| message | string | 返回信息或错误描述 |

#### 2. 获取动作列表服务 `~/get_arm_action_list`

**服务类型：** `g1_arm_action_ros2/srv/GetArmActionList`

**响应字段：**
| 字段 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| error_code | int32 | 错误码 |
| action_list | string | JSON 格式的动作列表 |

### 命令行示例

**执行预设动作（打招呼 - ID 26）：**
```bash
ros2 service call /g1_arm_action_node/execute_arm_action g1_arm_action_ros2/srv/ExecuteArmAction "{command_type: 0, action_id: 26, action_name: ''}"
```

**释放手臂（ID 99）：**
```bash
ros2 service call /g1_arm_action_node/execute_arm_action g1_arm_action_ros2/srv/ExecuteArmAction "{command_type: 0, action_id: 99, action_name: ''}"
```

**执行自定义动作：**
```bash
ros2 service call /g1_arm_action_node/execute_arm_action g1_arm_action_ros2/srv/ExecuteArmAction "{command_type: 1, action_id: 0, action_name: 'my_custom_action'}"
```

**停止当前动作：**
```bash
ros2 service call /g1_arm_action_node/execute_arm_action g1_arm_action_ros2/srv/ExecuteArmAction "{command_type: 2, action_id: 0, action_name: ''}"
```

**获取动作列表：**
```bash
ros2 service call /g1_arm_action_node/get_arm_action_list g1_arm_action_ros2/srv/GetArmActionList "{}"
```

## 预设动作 ID 列表

| ID | 动作名称 | 说明 |
|----|---------|------|
| 99 | release arm | 释放手臂 |
| 11 | two-hand kiss | 双手飞吻 |
| 12 | left kiss / right kiss | 单手飞吻 |
| 15 | hands up | 举手 |
| 17 | clap | 鼓掌 |
| 18 | high five | 击掌 |
| 19 | hug | 拥抱 |
| 20 | heart | 比心 |
| 21 | right heart | 右手比心 |
| 22 | reject | 拒绝 |
| 23 | right hand up | 右手举起 |
| 24 | x-ray | X光姿势 |
| 25 | face wave | 面部挥手 |
| 26 | high wave | 高位挥手 |
| 27 | shake hand | 握手 |

> **注意：** 部分动作可能不会在 APP 上显示，但可以通过程序执行。这些动作可能导致机器人摔倒，请谨慎使用。

## 错误码说明

| 错误码 | 说明 |
|--------|------|
| 0 | 成功 |
| 7400 | rt/armsdk 话题被占用 |
| 7401 | 手臂处于保持状态，需要发送释放指令(99)或相同的动作 ID |
| 7402 | 无效的动作 ID |
| 7404 | 无效的 FSM 状态（动作仅在 FSM ID {500, 501, 801} 下支持） |

## 注意事项

1. **FSM 状态**：手臂动作仅在特定的 FSM 状态下支持。可以订阅 `rt/sportmodestate` 话题检查当前 FSM ID。

2. **动作保持**：某些动作执行完成后会保持在最后一帧的姿态，最多保持 20 秒。可以发送 ID=99 或相同的动作 ID 来释放。

3. **超时设置**：宇树预设动作通常在 10 秒内完成，自定义示教动作可能需要更长时间，请相应调整 `timeout` 参数。

4. **网络配置**：部署到机器人内部时，需要正确配置 `network_interface` 参数以匹配机器人的网卡。

## License

Apache-2.0
