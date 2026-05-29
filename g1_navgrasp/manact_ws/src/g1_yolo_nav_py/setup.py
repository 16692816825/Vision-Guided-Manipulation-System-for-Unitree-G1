import glob
import os
from setuptools import find_packages, setup

package_name = "g1_yolo_nav_py"


def _collect_web_frontend():
    """收集 src/web_frontend/ 下所有文件，保留目录结构打包到 share/。

    源目录约定在 ../web_frontend 相对于本 setup.py。
    返回的源路径必须是相对于 setup.py 所在目录的相对路径
    （colcon 要求 data_files 源不能是绝对路径）。
    输出格式：[(share_subdir, [relative_file_paths]), ...]
    """
    # setup.py 位于 src/g1_yolo_nav_py/，前端在 src/web_frontend/
    here = os.path.dirname(os.path.abspath(__file__))
    frontend_src = os.path.normpath(os.path.join(here, "..", "web_frontend"))
    if not os.path.isdir(frontend_src):
        return []

    share_base = "share/" + package_name + "/web_frontend"
    result = []
    for root, _dirs, files in os.walk(frontend_src):
        if not files:
            continue
        rel = os.path.relpath(root, frontend_src)
        # 跳过 __pycache__ / 隐藏目录（注意 rel == "." 是根目录，不能跳）
        if rel != ".":
            segs = rel.split(os.sep)
            if any(s.startswith("__pycache__") or s.startswith(".") for s in segs):
                continue
        # share 子目录（保留层级）
        sub = share_base if rel == "." else share_base + "/" + rel.replace(os.sep, "/")
        # 源路径：相对于 setup.py 所在目录（here），形如 "../web_frontend/index.html"
        paths = []
        for f in files:
            # 跳过 .pyc 等编译产物
            if f.endswith(".pyc"):
                continue
            abs_path = os.path.join(root, f)
            rel_path = os.path.relpath(abs_path, here).replace(os.sep, "/")
            paths.append(rel_path)
        if paths:
            result.append((sub, paths))
    return result


def _collect_arm_scripts():
    """收集 arm/ 下所有脚本，打包到 share/g1_yolo_nav_py/arm/。

    arm/ 目录与 setup.py 同级（src/g1_yolo_nav_py/arm/）。
    返回相对路径（colcon 要求）。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    arm_dir = os.path.join(here, "arm")
    if not os.path.isdir(arm_dir):
        return []
    paths = []
    for f in os.listdir(arm_dir):
        if f.endswith(".py"):
            paths.append("arm/" + f)
    if not paths:
        return []
    return [("share/" + package_name + "/arm", paths)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob.glob("*.pt")),
    ] + _collect_arm_scripts() + _collect_web_frontend(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="xhunli@qq.com",
    description="宇树 G1 机器人 YOLO 目标识别与路径规划导航功能包",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_detector = g1_yolo_nav_py.yolo_detector:main",
            "detection_visualizer = g1_yolo_nav_py.detection_visualizer:main",
            "spatial_target = g1_yolo_nav_py.spatial_target:main",
            "loco_forward = g1_yolo_nav_py.loco_forward:main",
            "yaw_align = g1_yolo_nav_py.yaw_align:main",
            "rgbd_capture = g1_yolo_nav_py.rgbd_capture:main",
            "grasp_task = g1_yolo_nav_py.grasp_task:main",
            "control_panel = g1_yolo_nav_py.control_panel:main",
            "web_panel = g1_yolo_nav_py.web_panel:main",
        ],
    },
)
