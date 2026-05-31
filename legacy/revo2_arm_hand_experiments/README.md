# Revo2 与手臂混合实验历史脚本

这个目录保存从机器人 Revo2 示例目录中备份出来的早期实验脚本，仅用于追溯历史参数和实现思路。

不要把这里的脚本当作当前推荐流程直接运行。当前推荐流程是：

```text
g1_hand_arm_project/arm_pick_place_standalone.py
```

注意事项：

- 这些历史脚本可能同时控制 Revo2 手和 G1 手臂。
- 恢复出厂后调试手部时，优先使用 `hand_control/revo2/left_hand_safe_once.py` 或 `arm_pick_place_standalone.py --hand-only-test`。
- 不要再使用空 `LowCmd` 释放 `arm_sdk`；释放必须保持当前关节 `q/kp/kd` 后平滑降权。
