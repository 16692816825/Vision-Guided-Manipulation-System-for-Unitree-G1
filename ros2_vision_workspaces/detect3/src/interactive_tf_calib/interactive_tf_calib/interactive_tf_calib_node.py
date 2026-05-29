import json
import math
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from std_srvs.srv import Trigger

from tf2_ros import TransformBroadcaster

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    # REP-103: intrinsic rotations about fixed axes: roll(X), pitch(Y), yaw(Z)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class InteractiveTfCalibNode(Node):
    def __init__(self) -> None:
        super().__init__('interactive_tf_calib')

        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame', 'livox_frame')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('save_path', str(Path.home() / 'lidar_to_camera_extrinsics.json'))
        self.declare_parameter('initial_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_rpy_rad', [0.0, 0.0, 0.0])

        self._parent_frame = str(self.get_parameter('parent_frame').value)
        self._child_frame = str(self.get_parameter('child_frame').value)
        self._publish_hz = float(self.get_parameter('publish_hz').value)
        self._save_path = Path(str(self.get_parameter('save_path').value)).expanduser()

        xyz = self.get_parameter('initial_xyz_m').value
        rpy = self.get_parameter('initial_rpy_rad').value

        if isinstance(xyz, str):
            try:
                xyz = json.loads(xyz)
            except Exception:
                xyz = None
        if isinstance(rpy, str):
            try:
                rpy = json.loads(rpy)
            except Exception:
                rpy = None

        if not (isinstance(xyz, list) and len(xyz) == 3):
            xyz = [0.0, 0.0, 0.0]
        if not (isinstance(rpy, list) and len(rpy) == 3):
            rpy = [0.0, 0.0, 0.0]

        qx, qy, qz, qw = _quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        self._pose = Pose()
        self._pose.position.x = float(xyz[0])
        self._pose.position.y = float(xyz[1])
        self._pose.position.z = float(xyz[2])
        self._pose.orientation.x = qx
        self._pose.orientation.y = qy
        self._pose.orientation.z = qz
        self._pose.orientation.w = qw

        self._tf_broadcaster = TransformBroadcaster(self)

        self._server = InteractiveMarkerServer(self, 'interactive_tf_calib')
        self._make_marker()

        self._save_srv = self.create_service(Trigger, 'save_extrinsics', self._on_save)

        period = 1.0 / max(1e-6, self._publish_hz)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Interactive TF calib ready. Drag marker in RViz. Publishing TF {self._parent_frame} -> {self._child_frame}. "
            f"Save via /interactive_tf_calib/save_extrinsics to {self._save_path}"
        )

    def _make_marker(self) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._parent_frame
        int_marker.name = 'lidar_frame_pose'
        int_marker.description = f'{self._child_frame} pose in {self._parent_frame}'
        int_marker.scale = 0.4
        int_marker.pose = self._pose

        # 6DOF controls
        controls = [
            ('rotate_x', 1.0, 0.0, 0.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS),
            ('move_x', 1.0, 0.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS),
            ('rotate_y', 0.0, 1.0, 0.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS),
            ('move_y', 0.0, 1.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS),
            ('rotate_z', 0.0, 0.0, 1.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS),
            ('move_z', 0.0, 0.0, 1.0, 1.0, InteractiveMarkerControl.MOVE_AXIS),
        ]

        for name, ox, oy, oz, ow, mode in controls:
            c = InteractiveMarkerControl()
            c.name = name
            c.orientation.x = ox
            c.orientation.y = oy
            c.orientation.z = oz
            c.orientation.w = ow
            c.interaction_mode = mode
            c.always_visible = True
            int_marker.controls.append(c)

        self._server.insert(int_marker)
        self._server.applyChanges()

    def _on_feedback(self, feedback) -> None:
        # feedback.pose is geometry_msgs/Pose
        self._pose = feedback.pose

    def _build_tf(self) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._child_frame

        t.transform.translation.x = float(self._pose.position.x)
        t.transform.translation.y = float(self._pose.position.y)
        t.transform.translation.z = float(self._pose.position.z)

        t.transform.rotation.x = float(self._pose.orientation.x)
        t.transform.rotation.y = float(self._pose.orientation.y)
        t.transform.rotation.z = float(self._pose.orientation.z)
        t.transform.rotation.w = float(self._pose.orientation.w)
        return t

    def _on_timer(self) -> None:
        self._tf_broadcaster.sendTransform(self._build_tf())

    def _on_save(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        _ = request
        try:
            self._save_path.parent.mkdir(parents=True, exist_ok=True)

            data = {
                'parent_frame': self._parent_frame,
                'child_frame': self._child_frame,
                't_xyz_m': [
                    float(self._pose.position.x),
                    float(self._pose.position.y),
                    float(self._pose.position.z),
                ],
                'q_xyzw': [
                    float(self._pose.orientation.x),
                    float(self._pose.orientation.y),
                    float(self._pose.orientation.z),
                    float(self._pose.orientation.w),
                ],
            }
            self._save_path.write_text(json.dumps(data, indent=2, sort_keys=True))

            response.success = True
            response.message = f'Saved extrinsics to {self._save_path}'
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f'Failed to save extrinsics: {e}'
            self.get_logger().error(response.message)
            return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = InteractiveTfCalibNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
