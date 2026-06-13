import json
import math
import threading
from typing import Any, Dict, Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from mighty_sdk import MightyClient, MightyWebDevice


def _finite(value: Any, fallback: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return fallback
    return out if math.isfinite(out) else fallback


def _time_from_ns(ns: Optional[int]) -> Time:
    msg = Time()
    if ns is None:
        return msg
    value = int(ns)
    msg.sec = value // 1_000_000_000
    msg.nanosec = value % 1_000_000_000
    return msg


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class MightyRos2Publisher(Node):
    def __init__(self):
        super().__init__("mighty_ros2_publisher")

        self.declare_parameter("base_url", "")
        self.declare_parameter("start_vio", True)
        self.declare_parameter("stop_vio_on_shutdown", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_device_time", False)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("image_frame_id", "camera")

        self.base_url = self.get_parameter("base_url").value or ""
        self.start_vio_on_start = _as_bool(self.get_parameter("start_vio").value)
        self.stop_vio_on_shutdown = _as_bool(self.get_parameter("stop_vio_on_shutdown").value)
        self.publish_tf = _as_bool(self.get_parameter("publish_tf").value)
        self.use_device_time = _as_bool(self.get_parameter("use_device_time").value)
        self.frame_id = str(self.get_parameter("frame_id").value or "odom")
        self.child_frame_id = str(self.get_parameter("child_frame_id").value or "base_link")
        self.image_frame_id = str(self.get_parameter("image_frame_id").value or "camera")

        self.odom_pub = self.create_publisher(Odometry, "mighty/odom", 20)
        self.imu_pub = self.create_publisher(Imu, "mighty/imu", 100)
        self.image_pub = self.create_publisher(Image, "mighty/image_raw", 10)
        self.status_pub = self.create_publisher(String, "mighty/status", 10)
        self.vio_state_pub = self.create_publisher(String, "mighty/vio_state", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if self.base_url:
            device = MightyWebDevice(base_url=self.base_url)
        else:
            device = MightyWebDevice()
        self.client = MightyClient(device)
        self._shutdown_lock = threading.Lock()
        self._closed = False

        self.client.on_pose(self._on_pose)
        self.client.on_imu(self._on_imu)
        self.client.on_image(self._on_image)
        self.client.on_status(self._on_status)
        self.client.on_vio_state(self._on_vio_state)
        self.client.on_error(self._on_error)

        self.client.connect()
        info = device.get_info()
        self.get_logger().info(f"connected transport={info.get('transport')} source={info.get('source')}")

        if self.start_vio_on_start:
            res = self.client.start_vio()
            if res.get("ok"):
                self.get_logger().info("start_vio accepted")
            else:
                self.get_logger().warn(f"start_vio failed: {res.get('message', '')}")

    def close(self):
        with self._shutdown_lock:
            if self._closed:
                return
            self._closed = True

        if self.stop_vio_on_shutdown:
            try:
                res = self.client.stop_vio()
                if not res.get("ok"):
                    self.get_logger().warn(f"stop_vio failed: {res.get('message', '')}")
            except BaseException as exc:
                try:
                    self.get_logger().warn(f"stop_vio exception: {exc}")
                except BaseException:
                    pass

        try:
            self.client.disconnect()
        except BaseException as exc:
            try:
                self.get_logger().warn(f"disconnect exception: {exc}")
            except BaseException:
                pass

    def _stamp(self, timestamp_ns: Optional[int]) -> Time:
        if self.use_device_time and timestamp_ns is not None:
            return _time_from_ns(timestamp_ns)
        return self.get_clock().now().to_msg()

    def _on_pose(self, pose: Dict[str, Any]):
        stamp = self._stamp(pose.get("timestamp_ns"))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        pos = pose.get("position_m") or (0.0, 0.0, 0.0)
        odom.pose.pose.position.x = _finite(pos[0] if len(pos) > 0 else 0.0)
        odom.pose.pose.position.y = _finite(pos[1] if len(pos) > 1 else 0.0)
        odom.pose.pose.position.z = _finite(pos[2] if len(pos) > 2 else 0.0)

        quat = pose.get("orientation_xyzw")
        if quat and len(quat) == 4:
            odom.pose.pose.orientation.x = _finite(quat[0])
            odom.pose.pose.orientation.y = _finite(quat[1])
            odom.pose.pose.orientation.z = _finite(quat[2])
            odom.pose.pose.orientation.w = _finite(quat[3], 1.0)
        else:
            odom.pose.pose.orientation.w = 1.0

        v = pose.get("linear_velocity_body_mps")
        if v and len(v) == 3:
            odom.twist.twist.linear.x = _finite(v[0])
            odom.twist.twist.linear.y = _finite(v[1])
            odom.twist.twist.linear.z = _finite(v[2])

        w = pose.get("angular_velocity_body_rps")
        if w and len(w) == 3:
            odom.twist.twist.angular.x = _finite(w[0])
            odom.twist.twist.angular.y = _finite(w[1])
            odom.twist.twist.angular.z = _finite(w[2])

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = odom.pose.pose.position.z
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _on_imu(self, imu: Dict[str, Any]):
        for sample in imu.get("samples", []):
            msg = Imu()
            msg.header.stamp = self._stamp(sample.get("timestamp_ns"))
            msg.header.frame_id = self.child_frame_id
            msg.linear_acceleration.x = _finite(sample.get("ax"))
            msg.linear_acceleration.y = _finite(sample.get("ay"))
            msg.linear_acceleration.z = _finite(sample.get("az"))
            msg.angular_velocity.x = _finite(sample.get("gx"))
            msg.angular_velocity.y = _finite(sample.get("gy"))
            msg.angular_velocity.z = _finite(sample.get("gz"))
            self.imu_pub.publish(msg)

    def _on_image(self, image: Dict[str, Any]):
        if image.get("kind") == "stereo_raw":
            return

        width = int(image.get("width") or 0)
        height = int(image.get("height") or 0)
        data = bytes(image.get("data") or b"")
        if width <= 0 or height <= 0 or not data:
            return

        msg = Image()
        msg.header.stamp = self._stamp(image.get("timestamp_ns"))
        msg.header.frame_id = self.image_frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = width
        msg.data = data
        self.image_pub.publish(msg)

    def _on_status(self, status: Dict[str, Any]):
        msg = String()
        msg.data = str(status.get("text", ""))
        self.status_pub.publish(msg)

    def _on_vio_state(self, state: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(state, sort_keys=True, default=str)
        self.vio_state_pub.publish(msg)

    def _on_error(self, error: Dict[str, Any]):
        self.get_logger().warn(
            f"sdk error scope={error.get('scope')} code={error.get('code')} message={error.get('message')}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MightyRos2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
