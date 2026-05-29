# boot.py
import time
import json
from typing import Optional

# 定义必要的 API ID
ROBOT_API_ID_LOCO_GET_FSM_ID = 7001
ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002

def get_fsm_id(client) -> Optional[int]:
    """获取当前状态机 ID"""
    try:
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception as e:
        print(f"[Boot] Error getting FSM ID: {e}")
    return None

def get_fsm_mode(client) -> Optional[int]:
    """获取当前 FSM 的子模式"""
    try:
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None

def run_hanger_start_logic(client, log_callback=print):
    """
    悬挂启动核心逻辑
    client: 已经初始化的 LocoClient 实例
    log_callback: 用于回传日志的函数
    """
    log_callback("[-] [Boot] 开始执行悬挂启动程序...")
    
    # 1. 状态检查
    current_id = get_fsm_id(client)
    current_mode = get_fsm_mode(client)
    
    if current_id == 200 and current_mode is not None and current_mode != 2:
        log_callback("[!] [Boot] 机器人已在运控状态，无需启动。")
        return True

    # 2. 切换阻尼 (安全第一)
    log_callback(" -> [Boot] 切换至阻尼模式 (Damp)")
    client.Damp()
    time.sleep(1.0)

    # 3. 站立预备态 (FSM 4)
    log_callback(" -> [Boot] 切换至站立预备态 (FSM 4)")
    client.SetFsmId(4) 
    time.sleep(1.0)

    # 4. 触地探测
    step_height = 0.02
    max_height = 0.55
    current_h = 0.0
    feet_contacted = False

    log_callback(f" -> [Boot] 开始地面探测 (Max: {max_height}m)...")

    while current_h < max_height:
        current_h += step_height
        client.SetStandHeight(current_h)
        time.sleep(0.1) 
        
        mode = get_fsm_mode(client)
        if mode == 0 and current_h > 0.15:
            feet_contacted = True
            log_callback(f"[+] [Boot] 检测到触地! 高度: {current_h:.2f}m")
            break
    
    if not feet_contacted:
        log_callback("[WARN] [Boot] 腿已伸直仍未触地，为了安全将缩回。")
        client.SetStandHeight(0.0)
        time.sleep(1.0)
        client.Damp()
        return False

    # 5. 锁定并启动
    log_callback(" -> [Boot] 锁定姿态并启动运控...")
    client.BalanceStand(0)
    client.SetStandHeight(current_h)
    time.sleep(0.5)

    client.Start()
    time.sleep(1.0)
    
    log_callback("[SUCCESS] [Boot] 悬挂启动完成！")
    return True
