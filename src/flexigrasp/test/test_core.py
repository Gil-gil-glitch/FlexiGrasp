import os
import threading
import time

import numpy as np
import pytest

from flexigrasp.core import (COLS, FRAME_BYTES, MAGIC, ROWS, DemoStream, FingerChannel,
                             FrameParser, compute_features, parse_mask)


def make_frame(seed=0, value=None):
    rng = np.random.default_rng(seed)
    if value is not None:
        return np.full((ROWS, COLS), value, dtype=np.uint8)
    return rng.integers(0, 255, (ROWS, COLS), dtype=np.uint8)


def wire(frame):
    return MAGIC + frame.tobytes()


def test_clean_stream_split_into_odd_chunks():
    frames = [make_frame(i) for i in range(6)]
    stream = b"".join(wire(f) for f in frames)
    p = FrameParser()
    got = []
    for i in range(0, len(stream), 97):
        got += p.feed(stream[i:i + 97])
    # the last frame is only released once the next header arrives
    assert len(got) == 5
    for a, b in zip(got, frames):
        assert np.array_equal(a, b)
    assert p.bad_frames == 0 and p.dropped_bytes == 0


def test_lost_byte_rejects_that_frame_and_never_shifts_the_next():
    frames = [make_frame(i) for i in range(5)]
    parts = [wire(f) for f in frames]
    parts[2] = parts[2][:100] + parts[2][101:]          # lose one byte inside frame 2
    p = FrameParser()
    got = p.feed(b"".join(parts) + wire(make_frame(99)))
    # frames 0,1,3,4 survive unshifted; 2 is rejected
    assert p.bad_frames >= 1
    assert all(np.array_equal(g, f) for g, f in zip(got, [frames[0], frames[1], frames[3], frames[4]]))
    assert not any(g[-1, -1] == 170 and (g == 170).sum() == 1 for g in got)


def test_garbage_before_header_is_skipped():
    f = make_frame(3)
    p = FrameParser()
    p.feed(b"\x01\x02\x03" + wire(f) + wire(f))
    got = p.feed(wire(f))
    assert len(got) >= 1


def test_parse_mask():
    m = parse_mask("0,10; 0,11 ;1,30")
    assert m.sum() == 3 and m[0, 10] and m[1, 30]
    assert parse_mask("").sum() == 0
    with pytest.raises(ValueError):
        parse_mask("16,0")


def test_features_centroid_and_mask():
    base = np.full((ROWS, COLS), 3.0, np.float32)
    raw = base.copy()
    raw[4:7, 10:13] += 60.0
    mask = np.zeros((ROWS, COLS), bool)
    mask[0, 5] = True
    raw[0, 5] += 200.0                                   # masked taxel must be ignored
    f = compute_features(raw, base, mask, 15.0, 2)
    assert f.in_contact and f.contact_taxels == 9
    assert abs(f.centroid_row - 5.0) < 1e-4 and abs(f.centroid_col - 11.0) < 1e-4
    assert f.peak == pytest.approx(60.0)
    assert f.delta[0, 5] == 0.0
    f0 = compute_features(base.copy(), base, mask, 15.0, 2)
    assert not f0.in_contact and np.isnan(f0.centroid_row) and f0.total == 0.0


def test_channel_baseline_then_contact_then_rezero():
    stream = DemoStream(rate_hz=400.0, seed=1)
    ch = FingerChannel("left", "demo", baseline_frames=20, threshold=15.0, opener=lambda: stream,
                       log=lambda m: None)
    ch.start()
    try:
        deadline = time.time() + 10
        seen_cal = seen_contact = False
        while time.time() < deadline and not (seen_cal and seen_contact):
            for pf in ch.drain():
                seen_cal |= pf.calibrated
                seen_contact |= pf.calibrated and pf.feat.in_contact
            time.sleep(0.02)
        assert seen_cal, "never calibrated"
        assert seen_contact, "demo press never detected"
        assert not ch.calibrating
        ch.rezero()
        assert ch.calibrating
        time.sleep(0.5)
        assert ch.connected
    finally:
        ch.stop()


def test_channel_retries_until_the_port_appears():
    calls = {"n": 0}
    stream = DemoStream(rate_hz=400.0)

    def opener():
        calls["n"] += 1
        if calls["n"] < 3:
            raise OSError("no such device")
        return stream

    ch = FingerChannel("right", "/dev/nope", baseline_frames=5, opener=opener, log=lambda m: None)
    ch.start()
    try:
        t0 = time.time()
        while time.time() - t0 < 8 and ch.seq == 0:
            time.sleep(0.05)
        assert ch.seq > 0 and calls["n"] >= 3
    finally:
        ch.stop()


def test_real_pyserial_over_a_pty():
    serial = pytest.importorskip("serial")
    master, slave = os.openpty()
    path = os.ttyname(slave)
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            os.write(master, wire(make_frame(i % 7)))
            i += 1
            time.sleep(0.004)

    th = threading.Thread(target=writer, daemon=True)
    th.start()
    ch = FingerChannel("left", path, baud=115200, baseline_frames=5, log=lambda m: None)
    ch.start()
    try:
        t0 = time.time()
        while time.time() - t0 < 8 and ch.seq < 40:
            time.sleep(0.05)
        assert ch.seq >= 40
        assert ch.parser.bad_frames == 0
    finally:
        stop.set()
        ch.stop()
        os.close(master)
