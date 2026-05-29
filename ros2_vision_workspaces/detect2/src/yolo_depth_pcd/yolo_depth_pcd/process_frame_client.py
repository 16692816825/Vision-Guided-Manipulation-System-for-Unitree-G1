from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from yolo_depth_pcd_interfaces.srv import ProcessFrame


class ProcessFrameClient(Node):
    def __init__(self):
        super().__init__('process_frame_client')

        self.declare_parameter('dataset_root', '/home/xinghg/detect/datasets/demo1')
        self.declare_parameter('start_idx', 0)
        self.declare_parameter('end_idx', 0)
        self.declare_parameter('step', 1)
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('save_debug', False)

        self._cli = self.create_client(ProcessFrame, 'process_frame')

    def run(self):
        dataset_root = str(self.get_parameter('dataset_root').value)
        start_idx = int(self.get_parameter('start_idx').value)
        end_idx = int(self.get_parameter('end_idx').value)
        step = int(self.get_parameter('step').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        save_debug = bool(self.get_parameter('save_debug').value)

        if rate_hz <= 0:
            rate_hz = 1.0
        period = 1.0 / rate_hz

        while rclpy.ok() and not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /process_frame service...')

        i = start_idx
        while rclpy.ok() and i <= end_idx:
            req = ProcessFrame.Request()
            req.dataset_root = dataset_root
            req.frame_idx = f"{i:05d}"
            req.save_debug = save_debug

            t0 = time.time()
            fut = self._cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            dt = time.time() - t0

            if fut.result() is None:
                self.get_logger().error(f'call failed for frame {req.frame_idx}')
            else:
                resp = fut.result()
                if resp.success:
                    self.get_logger().info(
                        f"frame={req.frame_idx} ok cls={resp.class_name} conf={resp.confidence:.3f} inliers={resp.pcd_inlier_count} dt={dt:.2f}s"
                    )
                else:
                    self.get_logger().warn(f"frame={req.frame_idx} failed: {resp.message}")

            time.sleep(period)
            i += step


def main():
    rclpy.init()
    node = ProcessFrameClient()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
