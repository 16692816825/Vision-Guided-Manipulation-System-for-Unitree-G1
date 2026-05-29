# core/ros_manager.py
import subprocess
import os
import signal
import sys
import time
from core.settings_manager import settings
class RosManager:
    """
    专门负责 ROS 后端进程的生命周期管理
    包括: roscore 守护, launch 启动, 节点清理, 环境加载
    """
    def __init__(self, log_callback=print):
        self.log = log_callback
        self.ros_proc = None
        self.roscore_proc = None

        # [修改] 不再硬编码，而是从 settings 读取
        # 如果读取为空，则给一个警告性的默认值
        self.setup_path = settings.get("ros", "setup_path") or "/opt/ros/noetic/setup.bash"

    def start_roscore(self):
        """启动并守护 ROS Master"""
        # 1. 检查是否已经有 roscore 在运行
        try:
            # 尝试列出话题，如果成功说明 roscore 活着
            subprocess.check_output("rostopic list", shell=True, stderr=subprocess.DEVNULL)
            self.log("[System] 检测到外部 roscore 已在运行，复用之。")
            return
        except:
            pass

        # 2. 如果没活，就启动一个
        self.log("[System] 正在启动后台 roscore...")
        try:
            self.roscore_proc = subprocess.Popen(
                "roscore", 
                shell=True, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            # 给它 3 秒钟初始化，这很重要！
            time.sleep(3.0)
            self.log("[System] roscore 启动就绪。")
        except Exception as e:
            self.log(f"[System] roscore 启动失败: {e}")

    def stop_ros(self):
        """
        优雅退出业务节点，但保留 ROS Master。
        解决端口占用问题的关键在于：先尝试 SIGINT (Ctrl+C)，让驱动自己释放端口。
        """
        self.log("正在停止导航业务...")

        # 1. 停止当前运行的 Launch 进程 (发送 SIGINT 信号)
        # 这相当于你在终端按下了 Ctrl+C，让 launch 文件里的节点有机会执行析构函数
        if self.ros_proc:
            try:
                # 获取进程组 ID
                pgid = os.getpgid(self.ros_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                # 等待 2 秒让它们自己死
                self.ros_proc.wait(timeout=2.0)
            except Exception:
                # 如果超时或报错，强制杀掉
                try:
                    if self.ros_proc:
                        os.killpg(os.getpgid(self.ros_proc.pid), signal.SIGKILL)
                except: pass
            self.ros_proc = None

        # 2. 定点清理残留的 C++ 业务节点
        # 注意：不再包含 rosmaster 和 rosout
        nodes_to_kill = [
            "livox_ros_driver2_node", # 雷达驱动
            "map_builder_node",       # 建图算法
            "localizer_node",         # 定位算法
            "octomap_server_node",    # 地图转换
            "move_base",              # 导航控制
            "map_server",             # 地图加载
            "slam_reloc.py",          # 重定位脚本
            "rviz"                    # 如果有的话
        ]
        
        # 使用 -2 (SIGINT) 尝试温柔杀死，释放端口
        cmd_soft = f"killall -2 {' '.join(nodes_to_kill)}"
        subprocess.run(cmd_soft, shell=True, stderr=subprocess.DEVNULL)
        
        # 再次等待 1 秒
        time.sleep(1.0)
        
        # 使用 -9 (SIGKILL) 补刀，确保没有僵尸
        cmd_hard = f"killall -9 {' '.join(nodes_to_kill)}"
        subprocess.run(cmd_hard, shell=True, stderr=subprocess.DEVNULL)
        
        self.log("业务进程清理完毕。")

    def launch_ros(self, launch_file, package="fastlio", args_list=[]): 
        """
        启动 ROS launch 文件 (带自动环境加载 + 端口冷却)
        """
        # 1. 先清理旧的业务进程 (保留 roscore)
        self.stop_ros()
        
        # 2. 【核心修改】强制冷却 2 秒
        # 这是给 Linux 内核回收雷达 UDP 端口(56100)的时间
        self.log("等待系统资源释放 (2s)...")
        time.sleep(2.0)
        
        # 3. 检查环境文件
        if not os.path.exists(self.setup_path):
            self.log(f"严重错误: 找不到环境文件 {self.setup_path}")
            return False

        # 4. 构造 Shell 命令
        # 格式: source setup.bash && roslaunch package file args...
        ros_args = " ".join(args_list)
        full_cmd = f"source {self.setup_path} && roslaunch {package} {launch_file} {ros_args}"
        
        self.log(f"正在启动后端: {package}/{launch_file} ...")
        
        try:
            # 5. 执行命令
            self.ros_proc = subprocess.Popen(
                full_cmd, 
                shell=True,             # 关键：必须为 True 才能运行 source
                executable="/bin/bash", # 指定 bash 环境
                preexec_fn=os.setsid,   # 创建进程组，方便后续优雅退出
                stdout=sys.stdout,      # 输出到终端以便调试
                stderr=sys.stderr
            )
            return True
        except Exception as e:
            self.log(f"启动失败: {e}")
            return False

    def kill_all(self):
        """退出清理：暴力且彻底地杀死 ROS 进程组（包括 Master）"""
        # 先清理业务
        self.stop_ros()
        
        # 再清理 Master (如果有的话)
        if self.roscore_proc:
            self.log("正在清理 roscore...")
            try:
                os.killpg(os.getpgid(self.roscore_proc.pid), signal.SIGKILL)
            except: pass
            self.roscore_proc = None
        
        self.log("ROS 后端已彻底终止")

    def save_map_command(self, filename_base):
        """
        执行保存地图的 Shell 命令逻辑
        注意：这部分原本在 RobotProcess 主循环里，现在封装在这里
        """
        if not os.path.exists(self.setup_path):
            self.log("错误: 找不到环境文件，无法保存地图")
            return False

        # 1. 保存 3D (PCD)
        pcd_path = f"{filename_base}.pcd"
        save_3d_cmd = f"source {self.setup_path} && rosservice call /save_map \"{pcd_path}\" 0.0"
        
        # 2. 保存 2D (YAML)
        # map_server 会自动加上 .yaml 和 .pgm，所以这里传入不带后缀的基础名或完整路径
        save_2d_cmd = f"source {self.setup_path} && rosrun map_server map_saver -f {filename_base}"

        try:
            self.log(f"正在保存 3D 地图...")
            subprocess.run(
                ["/bin/bash", "-c", save_3d_cmd],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.log("✅ 3D PCD 地图保存成功！")
            
            self.log(f"正在保存 2D 地图...")
            subprocess.run(
                ["/bin/bash", "-c", save_2d_cmd],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.log("✅ 2D 栅格地图保存成功！")
            return True
        except Exception as e:
            self.log(f"❌ 地图保存失败: {e}")
            return False
