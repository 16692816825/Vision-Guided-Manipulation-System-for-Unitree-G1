"""
DDS 兼容层 — 解决 unitree_sdk2py (CycloneDDS) 与 ROS2 (CycloneDDS) 的 domain 冲突。

本模块提供两种隔离方案：

1. build_isolated_env() — 为 arm 子进程构建隔离环境变量
   用于 web_panel / grasp_task 通过 subprocess 启动 armup.py / armdown.py 时，
   确保子进程的 CycloneDDS 不会干扰父进程的 ROS2 DDS。

2. init_unitree_dds_before_ros2() — 同进程内先 SDK 后 ROS2（历史方案，目前未使用）
   如果未来需要在同一进程内同时使用 unitree SDK 和 ROS2，可用此函数。

核心原理：
    - unitree_sdk2py 的 ChannelFactoryInitialize 和 ROS2 的 rclpy.init()
      都基于 CycloneDDS，shared memory 机制会导致互相干扰（segfault）
    - 方案 1：子进程注入独立 CYCLONEDDS_URI（禁用 shared memory + 指定网卡）
    - 方案 2：同进程内用不同 DomainId 隔离

使用示例（方案 1 — 推荐）：
    from g1_yolo_nav_py._dds_compat import build_isolated_env
    env = build_isolated_env(network_interface="enp4s0", cyclonedds_home="...", sdk_path="...")
    subprocess.run([python, "armup.py", "enp4s0"], env=env)

使用示例（方案 2 — 同进程，历史兼容）：
    from g1_yolo_nav_py._dds_compat import init_unitree_dds_before_ros2
    init_unitree_dds_before_ros2(iface="eth0")
    rclpy.init(args=args)
"""

import os
import sys
import tempfile


# ======================================================================
# 方案 1：子进程环境隔离（推荐）
# ======================================================================

def build_isolated_env(
    network_interface: str = "",
    cyclonedds_home: str = "",
    sdk_python_path: str = "",
) -> dict:
    """构建 arm 子进程的隔离环境变量。

    关键隔离措施：
    1. 移除父进程的 CYCLONEDDS_URI / ROS_DOMAIN_ID / RMW_IMPLEMENTATION
    2. 注入独立的 CYCLONEDDS_URI：禁用 shared memory + 指定网卡
       → 子进程的 CycloneDDS 不会通过 shared memory 干扰父进程
    3. 注入 CYCLONEDDS_HOME（C 库路径）+ PYTHONPATH（SDK 路径）

    Args:
        network_interface: 网卡名（如 enp4s0），空则用 "lo"
        cyclonedds_home: CycloneDDS C 库安装目录，空则自动探测
        sdk_python_path: unitree_sdk2_python 源码目录，空则自动探测

    Returns:
        隔离后的环境变量 dict，可直接传给 subprocess.run(env=...)
    """
    env = os.environ.copy()

    # ★ 移除父进程的 ROS2 DDS 配置
    env.pop("CYCLONEDDS_URI", None)
    env.pop("ROS_DOMAIN_ID", None)
    env.pop("RMW_IMPLEMENTATION", None)

    # ★ 注入独立的 CycloneDDS 配置：
    # - 禁用 shared memory（避免跟父进程冲突）
    # - 禁用多播发现（避免父进程收到不兼容的 type 信息导致 segfault）
    # - 使用单播直连机器人 192.168.123.1（宇树 G1 默认 IP）
    # - 使用 DomainId 42（与 ROS2 默认 domain 0 隔离）
    net_iface = network_interface or "lo"
    cyclonedds_xml = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<CycloneDDS>'
        '<Domain id="42">'
        '<General>'
        f'<Interfaces><NetworkInterface name="{net_iface}" priority="default" multicast="false"/></Interfaces>'
        '<AllowMulticast>false</AllowMulticast>'
        '</General>'
        '<Discovery>'
        '<Peers><Peer address="192.168.123.1"/></Peers>'
        '<ParticipantIndex>auto</ParticipantIndex>'
        '</Discovery>'
        '<SharedMemory><Enable>false</Enable></SharedMemory>'
        '<Tracing><Verbosity>warning</Verbosity></Tracing>'
        '</Domain>'
        '</CycloneDDS>'
    )
    env["CYCLONEDDS_URI"] = cyclonedds_xml

    # --- CycloneDDS C 库路径 ---
    dds_home = cyclonedds_home or auto_detect_cyclonedds()
    if dds_home:
        env["CYCLONEDDS_HOME"] = dds_home
        lib_dir = os.path.join(dds_home, "lib")
        if os.path.isdir(lib_dir):
            env["LD_LIBRARY_PATH"] = lib_dir + ":" + env.get("LD_LIBRARY_PATH", "")
        env["CMAKE_PREFIX_PATH"] = dds_home + ":" + env.get("CMAKE_PREFIX_PATH", "")

    # --- SDK PYTHONPATH ---
    sdk_path = sdk_python_path or auto_detect_sdk_path()
    if sdk_path:
        env["PYTHONPATH"] = sdk_path + ":" + env.get("PYTHONPATH", "")

    return env


def get_venv_python() -> str:
    """获取 venv 的 Python 解释器路径（能访问 venv site-packages）。

    优先使用 VIRTUAL_ENV/bin/python，而不是 sys.executable
    （后者可能解析为 /usr/bin/python3，看不到 venv 包）。
    """
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        venv_python = os.path.join(venv, "bin", "python")
        if os.path.isfile(venv_python):
            return venv_python
    return sys.executable


# ======================================================================
# 自动探测
# ======================================================================

def auto_detect_cyclonedds() -> str:
    """自动探测 CycloneDDS 安装目录（检查 lib/libddsc.so 是否存在）。"""
    home = os.path.expanduser("~")
    candidates = [
        os.path.join(home, "unitree_ros2/cyclonedds_ws/install/cyclonedds"),
        os.path.join(home, "cyclonedds_ws/install/cyclonedds"),
        "/opt/cyclonedds",
        "/usr/local",
    ]
    for c in candidates:
        if os.path.isfile(os.path.join(c, "lib", "libddsc.so")):
            return c
        if os.path.isfile(os.path.join(c, "lib64", "libddsc.so")):
            return c
        if os.path.isfile(os.path.join(c, "lib", "cmake", "CycloneDDS", "CycloneDDSConfig.cmake")):
            return c
    return ""


def auto_detect_sdk_path() -> str:
    """自动探测 unitree_sdk2_python 目录。"""
    home = os.path.expanduser("~")
    candidates = [
        os.path.join(home, "unitree_sdk2_python"),
        os.path.join(home, "G1DWAQ_Lab/unitree_sdk2_python"),
    ]
    for c in candidates:
        if os.path.isfile(os.path.join(c, "unitree_sdk2py", "__init__.py")):
            return c
    return ""


# ======================================================================
# 方案 2：同进程隔离（历史兼容，目前未使用）
# ======================================================================

# CycloneDDS XML 配置模板 — 为 ROS2 使用独立的 DomainId
_CYCLONEDDS_XML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain Id="{domain_id}">
  </Domain>
</CycloneDDS>
"""

_dds_initialized = False


def init_unitree_dds_before_ros2(iface: str = "") -> bool:
    """
    在 rclpy.init() 之前初始化 unitree SDK DDS（同进程方案）。

    ⚠️ 注意：当前架构已改为 subprocess 隔离（方案 1），本函数仅保留向后兼容。
    主控制节点（web_panel / grasp_task / control_panel）不需要调用此函数。

    关键：必须先调用 ChannelFactoryInitialize，再设置 CYCLONEDDS_URI。
    如果反过来，unitree SDK 创建 CycloneDDS domain 时会读到错误的配置。

    Args:
        iface: 网络接口名（空=自动检测）

    Returns:
        True 如果 DDS 初始化成功，False 否则
    """
    global _dds_initialized
    if _dds_initialized:
        return True

    # 1. 先初始化 unitree SDK（在 CYCLONEDDS_URI 未设置时，使用默认 DomainId 0）
    dds_ok = False
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, iface)
        _dds_initialized = True
        dds_ok = True
    except Exception as e:
        print(f"[WARNING] ChannelFactoryInitialize 失败: {e}", file=sys.stderr)

    # 2. 只有 unitree SDK 初始化成功时，才设置 CYCLONEDDS_URI 让 ROS2 使用 DomainId 1
    if dds_ok:
        _xml_content = _CYCLONEDDS_XML_TEMPLATE.format(domain_id=1)
        try:
            _xml_path = os.path.join(tempfile.gettempdir(), "ros2_cyclonedds_domain1.xml")
            with open(_xml_path, "w") as f:
                f.write(_xml_content)
            os.environ["CYCLONEDDS_URI"] = f"file://{_xml_path}"
        except Exception:
            os.environ["CYCLONEDDS_URI"] = _xml_content.strip()

        # 同时设置 ROS_DOMAIN_ID 与 CYCLONEDDS_URI 保持一致
        os.environ.setdefault("ROS_DOMAIN_ID", "1")

    return dds_ok
