import os

import numpy as np
import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from flexigrasp.viewer import COLS, ROWS, DualViewer, ViewFrame  # noqa: E402


def _frame(t, seq, blob_at=None, total=0.0):
    raw = np.full((ROWS, COLS), 4.0, np.float32)
    delta = np.zeros((ROWS, COLS), np.float32)
    cr = cc = float("nan")
    n = 0
    if blob_at:
        r, c = blob_at
        rr, cc_ = np.indices((ROWS, COLS))
        delta = (90 * np.exp(-((rr - r) ** 2 + (cc_ - c) ** 2) / 8.0)).astype(np.float32)
        raw = raw + delta
        n = int((delta > 15).sum())
        cr, cc = float(r), float(c)
        total = float(np.clip(delta - 15, 0, None).sum())
    return ViewFrame(t=t, seq=seq, raw=raw, delta=delta, calibrated=True, peak=float(delta.max()),
                     total=total, contact_taxels=n, in_contact=n >= 2, centroid_row=cr, centroid_col=cc,
                     bad_frames=0, dropped_bytes=0, rate_hz=98.0)


def test_render_two_fingers(tmp_path):
    import time
    calls = []
    v = DualViewer(["left", "right"], lambda n: calls.append(n))
    now = time.time()
    for i in range(300):
        t = now - 3.0 + i * 0.01
        v.views["left"].push(_frame(t, i, (6, 10) if i > 100 else None))
        v.views["right"].push(_frame(t, i, (9, 22) if i > 150 else None))
    v.update()
    out = tmp_path / "viewer.png"
    v.fig.savefig(out, dpi=90)
    assert out.stat().st_size > 20_000
    v.buttons[0].on_clicked  # buttons exist
    v.rezero(None)
    assert calls == [None]
    os.makedirs("/tmp/claude-0/-home-claude/638f6bc2-14cc-56be-9ba7-925dcfe44bcd/scratchpad", exist_ok=True)
    v.fig.savefig("/tmp/claude-0/-home-claude/638f6bc2-14cc-56be-9ba7-925dcfe44bcd/scratchpad/viewer.png", dpi=90)
