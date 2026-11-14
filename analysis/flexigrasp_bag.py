"""
Read FlexiGrasp / X6 rosbag2 (.db3) files without ROS, for the analysis notebooks.

Decodes the CDR bytes directly, so it runs in any Python with numpy (no sourced workspace).
Topics handled:
    flexigrasp_msgs/TactileFrame   read_frames()
    sensor_msgs/JointState         read_joint_states()
    std_msgs/Float32               read_float32()
    /sequence_step (JSON String)   read_steps()

Pad signals returned per frame:
    pos     sum of positive baseline-subtracted taxels (the force signal; see the coin calibration)
    total   the node's published total (sum above the contact threshold; has a dead zone)
    contact in_contact flag
    cxy     contact centroid (row, col), NaN when there is no contact
    vals    full 16 x 32 baseline-subtracted image (only when images=True; large)
"""
import json
import sqlite3
import struct

import numpy as np


def _align(o, a):
    return (o + a - 1) // a * a


def _topics(con):
    return {name: tid for tid, name, _ in con.execute("select id, name, type from topics")}


def _rows(con, topic):
    tid = _topics(con)[topic]
    return con.execute("select timestamp, data from messages where topic_id=? order by timestamp",
                       (tid,)).fetchall()


def list_topics(db):
    con = sqlite3.connect(db)
    return {name: typ for _, name, typ in con.execute("select id, name, type from topics")}


# ---- TactileFrame --------------------------------------------------------------------------------
def _frame_layout(data):
    """Byte offsets of the value array and the fields after it. The frame_id string length differs
    between pads ('left_...' vs 'right_...'), which shifts everything after it."""
    b = data[4:]                              # skip the 4-byte CDR encapsulation header
    o = 8                                     # header.stamp
    for _ in range(2):                        # header.frame_id, finger name
        n = struct.unpack_from("<I", b, o)[0]
        o = _align(o + 4 + n, 4)
    o = _align(o + 4 + 2, 4)                  # seq (uint32), rows, cols (uint8)
    n = struct.unpack_from("<I", b, o)[0]     # raw uint8[]
    o = _align(o + 4 + n, 4)
    o += 4                                    # bool + padding
    count = struct.unpack_from("<I", b, o)[0]
    vals = o + 4 + 4                          # +4 for the encapsulation header
    tail = vals + 4 * count
    return vals, count, tail


def read_frames(db, side, images=False):
    con = sqlite3.connect(db)
    rows = _rows(con, f"/flexigrasp/{side}/frame")
    vals_off, count, tail = _frame_layout(rows[0][1])
    t = np.array([r[0] for r in rows]) * 1e-9
    vals = np.array([np.frombuffer(r[1], "<f4", count, vals_off) for r in rows])
    out = dict(
        t=t,
        pos=np.clip(vals, 0, None).sum(axis=1),
        peak=np.array([struct.unpack_from("<f", r[1], tail)[0] for r in rows]),
        total=np.array([struct.unpack_from("<f", r[1], tail + 4)[0] for r in rows]),
        contact=np.array([r[1][tail + 10] for r in rows], dtype=bool),
        cxy=np.array([struct.unpack_from("<ff", r[1], tail + 12) for r in rows]),
    )
    if images:
        out["vals"] = vals.reshape(len(rows), 16, -1)
    return out


# ---- JointState ----------------------------------------------------------------------------------
def _joint_state(data):
    b = data[4:]
    o = 8
    n = struct.unpack_from("<I", b, o)[0]
    o = _align(o + 4 + n, 4)
    count = struct.unpack_from("<I", b, o)[0]
    o += 4
    names = []
    for _ in range(count):
        n = struct.unpack_from("<I", b, o)[0]
        names.append(b[o + 4:o + 4 + n - 1].decode())
        o = _align(o + 4 + n, 4)
    arrays = []
    for _ in range(3):                        # position, velocity, effort
        o = _align(o, 4)
        if o + 4 > len(b):
            arrays.append(())
            continue
        k = struct.unpack_from("<I", b, o)[0]
        o += 4
        if k:
            o = _align(o, 8)
        arrays.append(struct.unpack_from(f"<{k}d", b, o))
        o += 8 * k
    return names, arrays


def read_joint_states(db, topic="/joint_states"):
    """Returns t, names, and position / velocity / effort arrays of shape (n, joints)."""
    con = sqlite3.connect(db)
    rows = _rows(con, topic)
    names = _joint_state(rows[0][1])[0]
    pos, vel, eff = [], [], []
    for _, d in rows:
        n, (p, v, e) = _joint_state(d)
        idx = [n.index(j) for j in names]
        pos.append([p[i] for i in idx])
        vel.append([v[i] for i in idx] if v else [np.nan] * len(names))
        eff.append([e[i] for i in idx] if e else [np.nan] * len(names))
    return dict(t=np.array([r[0] for r in rows]) * 1e-9, names=names,
                pos=np.array(pos), vel=np.array(vel), eff=np.array(eff))


def read_commands(db, topic="/arm_command"):
    """Each /arm_command as (t, {joint: target})."""
    con = sqlite3.connect(db)
    out = []
    for ts, d in _rows(con, topic):
        n, (p, _, _) = _joint_state(d)
        out.append((ts * 1e-9, dict(zip(n, p))))
    return out


# ---- simple messages -----------------------------------------------------------------------------
def read_float32(db, topic):
    con = sqlite3.connect(db)
    rows = _rows(con, topic)
    return (np.array([r[0] for r in rows]) * 1e-9,
            np.array([struct.unpack_from("<f", r[1], 4)[0] for r in rows]))


def read_steps(db, topic="/sequence_step"):
    """Sequence runner events as a list of dicts with a 't' (bag time, s) field."""
    con = sqlite3.connect(db)
    out = []
    for ts, d in _rows(con, topic):
        n = struct.unpack_from("<I", d, 4)[0]
        ev = json.loads(d[8:8 + n - 1].decode())
        ev["t_bag"] = ts * 1e-9
        out.append(ev)
    return out


def step_times(steps, phase="steps"):
    """{step name: (start, end)} for one phase of a single-repeat run."""
    start, out = {}, {}
    for ev in steps:
        if ev.get("phase") != phase:
            continue
        if ev["status"] == "start":
            start[ev["step"]] = ev["t_bag"]
        elif ev["step"] in start:
            out[ev["step"]] = (start[ev["step"]], ev["t_bag"], ev["status"])
    return out
