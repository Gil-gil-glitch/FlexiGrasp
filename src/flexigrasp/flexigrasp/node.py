"""
flexigrasp_node: reads both FlexiTac finger pads and publishes them.

Topics (per finger, namespace /flexigrasp/<name>/ by default):
  frame   flexigrasp_msgs/TactileFrame   raw + delta + features + link health
  image   sensor_msgs/Image (mono8)      delta clipped to 0..255, for rqt_image_view
Services:
  <name>/rezero      std_srvs/Trigger    new baseline for one finger (pad must be unloaded)
  rezero_all         std_srvs/Trigger    new baseline for every finger
"""
from __future__ import annotations

import array
import time
from typing import Dict

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from flexigrasp_msgs.msg import TactileFrame
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from .core import (COLS, DEFAULT_BAUD, ROWS, DemoStream, FingerChannel, ProcessedFrame,
                   parse_mask)


def _stamp(ns: int) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(ns // 1_000_000_000)
    msg.nanosec = int(ns % 1_000_000_000)
    return msg


class FlexiGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("flexigrasp_node")
        self.declare_parameter("finger_names", ["left", "right"])
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("baseline_frames", 30)
        self.declare_parameter("contact_threshold", 15.0)
        self.declare_parameter("contact_min_taxels", 2)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("demo", False)            # simulated pads, no hardware needed
        self.declare_parameter("demo_glitch", False)     # also drop random bytes in demo mode
        self.declare_parameter("topic_prefix", "flexigrasp")

        names = list(self.get_parameter("finger_names").value)
        baud = int(self.get_parameter("baud").value)
        demo = bool(self.get_parameter("demo").value)
        glitch = bool(self.get_parameter("demo_glitch").value)
        prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.publish_image = bool(self.get_parameter("publish_image").value)

        defaults = {"left": "/dev/ttyUSB0", "right": "/dev/ttyUSB1"}
        self.channels: Dict[str, FingerChannel] = {}
        self.pub_frame = {}
        self.pub_image = {}
        self.frame_ids = {}
        self._last_warn = {}

        for i, name in enumerate(names):
            self.declare_parameter(f"{name}.port", defaults.get(name, f"/dev/ttyUSB{i}"))
            self.declare_parameter(f"{name}.mask", "")
            self.declare_parameter(f"{name}.frame_id", f"{name}_finger_tactile")
            port = str(self.get_parameter(f"{name}.port").value)
            mask = parse_mask(str(self.get_parameter(f"{name}.mask").value))
            self.frame_ids[name] = str(self.get_parameter(f"{name}.frame_id").value)

            opener = None
            if demo:
                stream = DemoStream(glitch=glitch, seed=i, phase=1.7 * i)
                opener = (lambda s=stream: s)
            ch = FingerChannel(
                name, port, baud,
                baseline_frames=int(self.get_parameter("baseline_frames").value),
                threshold=float(self.get_parameter("contact_threshold").value),
                min_taxels=int(self.get_parameter("contact_min_taxels").value),
                mask=mask, opener=opener, log=self.get_logger().info)
            self.channels[name] = ch
            base = f"/{prefix}/{name}"
            self.pub_frame[name] = self.create_publisher(TactileFrame, f"{base}/frame",
                                                         qos_profile_sensor_data)
            if self.publish_image:
                self.pub_image[name] = self.create_publisher(Image, f"{base}/image",
                                                             qos_profile_sensor_data)
            self.create_service(Trigger, f"{base}/rezero",
                                lambda req, res, n=name: self._rezero([n], res))
            self._last_warn[name] = 0.0

        self.create_service(Trigger, f"/{prefix}/rezero_all",
                            lambda req, res: self._rezero(list(self.channels), res))

        for ch in self.channels.values():
            ch.start()
        self.create_timer(0.002, self._publish)        # drain the reader threads
        self.create_timer(2.0, self._watchdog)
        mode = "DEMO (simulated)" if demo else "serial"
        self.get_logger().info(
            f"FlexiGrasp up ({mode}): " +
            ", ".join(f"{n}={c.port}" for n, c in self.channels.items()) +
            f" @ {baud}. Keep the pads unloaded while the baselines are collected.")

    # -- services
    def _rezero(self, names, res):
        res.success = True
        res.message = " | ".join(self.channels[n].rezero() for n in names)
        self.get_logger().info(res.message)
        return res

    # -- publishing
    def _publish(self) -> None:
        for name, ch in self.channels.items():
            for pf in ch.drain():
                self.pub_frame[name].publish(self._frame_msg(name, pf))
                if self.publish_image:
                    self.pub_image[name].publish(self._image_msg(name, pf))

    def _frame_msg(self, name: str, pf: ProcessedFrame) -> TactileFrame:
        m = TactileFrame()
        m.header.stamp = _stamp(pf.stamp_ns)
        m.header.frame_id = self.frame_ids[name]
        m.finger = name
        m.seq = pf.seq
        m.rows = ROWS
        m.cols = COLS
        m.raw = pf.raw.tobytes()
        m.calibrated = pf.calibrated
        m.delta = array.array("f", pf.feat.delta.astype(np.float32).ravel().tolist())
        m.peak = pf.feat.peak
        m.total = pf.feat.total
        m.contact_taxels = pf.feat.contact_taxels
        m.in_contact = pf.feat.in_contact
        m.centroid_row = pf.feat.centroid_row
        m.centroid_col = pf.feat.centroid_col
        m.bad_frames = pf.bad_frames
        m.dropped_bytes = pf.dropped_bytes
        m.rate_hz = pf.rate_hz
        return m

    def _image_msg(self, name: str, pf: ProcessedFrame) -> Image:
        img = Image()
        img.header.stamp = _stamp(pf.stamp_ns)
        img.header.frame_id = self.frame_ids[name]
        img.height, img.width = ROWS, COLS
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = COLS
        img.data = np.clip(pf.feat.delta, 0, 255).astype(np.uint8).tobytes()
        return img

    def _watchdog(self) -> None:
        now = time.time()
        for name, ch in self.channels.items():
            if now - self._last_warn[name] < 10.0:
                continue
            if not ch.connected:
                self._last_warn[name] = now
                self.get_logger().warn(f"[{name}] not connected to {ch.port}: {ch.last_error or 'opening...'}")
            elif ch.last_frame_time and now - ch.last_frame_time > 1.0:
                self._last_warn[name] = now
                self.get_logger().warn(f"[{name}] port open but no frames for {now - ch.last_frame_time:.1f} s "
                                       "(check power, firmware, baud)")

    def destroy_node(self):
        for ch in self.channels.values():
            ch.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlexiGraspNode()
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
