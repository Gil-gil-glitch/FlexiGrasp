"""
Exercises flexigrasp.node end to end against tiny stand-ins for rclpy and the message classes,
so the node logic can be tested on a machine without ROS 2. Skipped when real rclpy is present
(use `colcon test` / the launch file there).
"""
import importlib.util
import sys
import time
import types

import pytest

if importlib.util.find_spec("rclpy") is not None:
    pytest.skip("real rclpy available; stub test not needed", allow_module_level=True)


class _NS:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _install_stubs(overrides):
    class Param:
        def __init__(self, v):
            self.value = v

    class Pub:
        def __init__(self):
            self.msgs = []

        def publish(self, m):
            self.msgs.append(m)

    class Log:
        def info(self, m):
            pass

        warn = info

    class Node:
        def __init__(self, name):
            self._p, self.pubs, self.timers, self.services = {}, {}, [], {}

        def declare_parameter(self, name, default=None):
            self._p[name] = overrides.get(name, default)

        def get_parameter(self, name):
            return Param(self._p[name])

        def create_publisher(self, typ, topic, qos):
            self.pubs[topic] = Pub()
            return self.pubs[topic]

        def create_timer(self, period, cb):
            self.timers.append(cb)

        def create_service(self, typ, name, cb):
            self.services[name] = cb

        def get_logger(self):
            return Log()

        def destroy_node(self):
            pass

    class Header:
        def __init__(self):
            self.stamp, self.frame_id = None, ""

    class TactileFrame:
        def __init__(self):
            self.header = Header()

    class Image(TactileFrame):
        pass

    class TimeMsg:
        pass

    class Trigger:
        class Request:
            pass

    mods = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "builtin_interfaces": types.ModuleType("builtin_interfaces"),
        "builtin_interfaces.msg": types.ModuleType("builtin_interfaces.msg"),
        "flexigrasp_msgs": types.ModuleType("flexigrasp_msgs"),
        "flexigrasp_msgs.msg": types.ModuleType("flexigrasp_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_srvs": types.ModuleType("std_srvs"),
        "std_srvs.srv": types.ModuleType("std_srvs.srv"),
    }
    mods["rclpy.node"].Node = Node
    mods["rclpy.qos"].qos_profile_sensor_data = object()
    mods["builtin_interfaces.msg"].Time = TimeMsg
    mods["flexigrasp_msgs.msg"].TactileFrame = TactileFrame
    mods["sensor_msgs.msg"].Image = Image
    mods["std_srvs.srv"].Trigger = Trigger
    sys.modules.update(mods)
    for n in [m for m in sys.modules if m.startswith("flexigrasp.node")]:
        del sys.modules[n]


def test_node_publishes_both_fingers_in_demo_mode():
    _install_stubs({"demo": True, "baseline_frames": 10, "left.mask": "0,10;1,30"})
    from flexigrasp import node as nodemod

    n = nodemod.FlexiGraspNode()
    try:
        t0 = time.time()
        while time.time() - t0 < 6:
            for cb in n.timers[:1]:
                cb()
            lf = n.pubs["/flexigrasp/left/frame"].msgs
            rf = n.pubs["/flexigrasp/right/frame"].msgs
            if lf and rf and lf[-1].calibrated and rf[-1].calibrated and len(lf) > 80:
                break
            time.sleep(0.01)
        lf = n.pubs["/flexigrasp/left/frame"].msgs
        rf = n.pubs["/flexigrasp/right/frame"].msgs
        assert len(lf) > 50 and len(rf) > 50
        m = lf[-1]
        assert m.finger == "left" and m.header.frame_id == "left_finger_tactile"
        assert m.rows == 16 and m.cols == 32 and len(m.raw) == 512 and len(m.delta) == 512
        assert m.calibrated and rf[-1].finger == "right"
        seqs = [x.seq for x in lf]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
        assert len(n.pubs["/flexigrasp/left/image"].msgs[-1].data) == 512
        assert m.header.stamp.sec > 1_600_000_000

        res = types.SimpleNamespace(success=False, message="")
        out = n.services["/flexigrasp/rezero_all"](None, res)
        assert out.success and "left" in out.message and "right" in out.message
        assert n.channels["left"].calibrating and n.channels["right"].calibrating
        res = types.SimpleNamespace(success=False, message="")
        out = n.services["/flexigrasp/left/rezero"](None, res)
        assert out.success and "right" not in out.message
    finally:
        n.destroy_node()
