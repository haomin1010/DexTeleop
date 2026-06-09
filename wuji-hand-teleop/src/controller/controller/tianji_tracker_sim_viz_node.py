"""RViz marker visualization for tracker-driven Tianji arm debugging."""

from __future__ import annotations

import argparse
import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


class TianjiTrackerSimViz(Node):
    """Publishes lightweight RViz markers for tracker and Tianji target frames."""

    def __init__(self, base_frame: str = "world", side: str = "right"):
        super().__init__("tianji_tracker_sim_viz")
        self.base_frame = base_frame
        self.side = side
        self.target_frame = f"tianji_{side}"
        self.chest_frame = f"{side}_chest"
        self.wrist_frame = f"{side}_wrist"
        self.arm_frame = f"{side}_arm"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, "/tianji_tracker_sim/markers", 10)
        self.timer = self.create_timer(0.05, self._publish)

        self.get_logger().info(
            f"Publishing RViz markers on /tianji_tracker_sim/markers "
            f"({self.base_frame}: {self.chest_frame}, {self.wrist_frame}, "
            f"{self.target_frame}, {self.arm_frame})"
        )

    def _lookup(self, frame: str):
        try:
            return self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def _sphere(self, marker_id: int, frame: str, color: ColorRGBA, scale: float = 0.04) -> Optional[Marker]:
        tf = self._lookup(frame)
        if tf is None:
            return None
        t = tf.transform.translation
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frames"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = _point(t.x, t.y, t.z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def _text(self, marker_id: int, frame: str, label: str, color: ColorRGBA) -> Optional[Marker]:
        tf = self._lookup(frame)
        if tf is None:
            return None
        t = tf.transform.translation
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = _point(t.x, t.y, t.z + 0.06)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.04
        marker.color = color
        marker.text = label
        return marker

    def _line(self, marker_id: int, from_frame: str, to_frame: str, color: ColorRGBA) -> Optional[Marker]:
        a = self._lookup(from_frame)
        b = self._lookup(to_frame)
        if a is None or b is None:
            return None
        at = a.transform.translation
        bt = b.transform.translation
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "links"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.012
        marker.color = color
        marker.points = [_point(at.x, at.y, at.z), _point(bt.x, bt.y, bt.z)]
        return marker

    def _axes(self, marker_id: int, frame: str, length: float = 0.12) -> Optional[MarkerArray]:
        tf = self._lookup(frame)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = _quat_to_matrix(q.x, q.y, q.z, q.w)
        origin = [t.x, t.y, t.z]
        colors = [_color(1.0, 0.1, 0.1), _color(0.1, 0.9, 0.1), _color(0.1, 0.3, 1.0)]
        out = MarkerArray()
        for axis in range(3):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"axes_{frame}"
            marker.id = marker_id + axis
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.color = colors[axis]
            end = [
                origin[0] + rot[0][axis] * length,
                origin[1] + rot[1][axis] * length,
                origin[2] + rot[2][axis] * length,
            ]
            marker.points = [_point(*origin), _point(*end)]
            out.markers.append(marker)
        return out

    def _publish(self) -> None:
        markers = MarkerArray()
        items = [
            (1, self.chest_frame, "right_chest", _color(1.0, 0.9, 0.1)),
            (2, self.wrist_frame, "right_wrist tracker", _color(0.1, 0.8, 1.0)),
            (3, self.target_frame, "tianji_right target", _color(1.0, 0.2, 0.8)),
            (4, self.arm_frame, "right_arm tracker", _color(0.8, 0.6, 1.0)),
        ]
        for marker_id, frame, label, color in items:
            sphere = self._sphere(marker_id, frame, color)
            text = self._text(marker_id + 100, frame, label, color)
            if sphere:
                markers.markers.append(sphere)
            if text:
                markers.markers.append(text)

        for line in [
            self._line(20, self.chest_frame, self.target_frame, _color(1.0, 0.2, 0.8)),
            self._line(21, self.chest_frame, self.arm_frame, _color(0.8, 0.6, 1.0)),
        ]:
            if line:
                markers.markers.append(line)

        for axes in [self._axes(30, self.target_frame), self._axes(40, self.chest_frame)]:
            if axes:
                markers.markers.extend(axes.markers)

        self.pub.publish(markers)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize tracker/Tianji debug markers in RViz.")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--side", default="right", choices=["left", "right"])
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "tianji_tracker_sim_viz"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])

    rclpy.init(args=raw_argv)
    node = TianjiTrackerSimViz(base_frame=args.base_frame, side=args.side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
