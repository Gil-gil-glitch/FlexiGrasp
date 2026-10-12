"""
flexigrasp_gui: live two-finger view. Subscribes to the TactileFrame topics published by
flexigrasp_node and shows both pads side by side. Works with the real node or with a bag.

Needs a display and python3-tk (matplotlib's TkAgg backend).
"""
from __future__ import annotations

import threading

import numpy as np
import rclpy
from flexigrasp_msgs.msg import TactileFrame
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from .viewer import ROWS, COLS, DualViewer, ViewFrame


class GuiNode(Node):
    def __init__(self) -> None:
        super().__init__("flexigrasp_gui")
        self.declare_parameter("finger_names", ["left", "right"])
        self.declare_parameter("topic_prefix", "flexigrasp")
        self.declare_parameter("window_s", 15.0)
        self.declare_parameter("delta_range", 120.0)
        self.declare_parameter("contact_threshold", 15.0)
        self.names = list(self.get_parameter("finger_names").value)
        self.prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.viewer = None  # set by main()
        self.rezero_clients = {}
        for name in self.names:
            self.create_subscription(TactileFrame, f"/{self.prefix}/{name}/frame",
                                     lambda m, n=name: self._on_frame(n, m), qos_profile_sensor_data)
            self.rezero_clients[name] = self.create_client(Trigger, f"/{self.prefix}/{name}/rezero")
        self.rezero_clients[None] = self.create_client(Trigger, f"/{self.prefix}/rezero_all")

    def _on_frame(self, name: str, m: TactileFrame) -> None:
        if self.viewer is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        raw = np.frombuffer(bytes(m.raw), dtype=np.uint8).astype(np.float32)
        delta = np.asarray(m.delta, dtype=np.float32)
        if raw.size != ROWS * COLS or delta.size != ROWS * COLS:
            return
        self.viewer.views[name].push(ViewFrame(
            t=t, seq=m.seq, raw=raw.reshape(ROWS, COLS), delta=delta.reshape(ROWS, COLS),
            calibrated=m.calibrated, peak=m.peak, total=m.total, contact_taxels=m.contact_taxels,
            in_contact=m.in_contact, centroid_row=m.centroid_row, centroid_col=m.centroid_col,
            bad_frames=m.bad_frames, dropped_bytes=m.dropped_bytes, rate_hz=m.rate_hz))

    def rezero(self, name) -> None:
        cli = self.rezero_clients[name]
        if not cli.service_is_ready():
            self.get_logger().warn("re-zero service not available (is flexigrasp_node running?)")
            return
        cli.call_async(Trigger.Request())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuiNode()
    node.viewer = DualViewer(
        node.names, node.rezero,
        window_s=float(node.get_parameter("window_s").value),
        delta_range=float(node.get_parameter("delta_range").value),
        threshold=float(node.get_parameter("contact_threshold").value))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        node.viewer.run()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
