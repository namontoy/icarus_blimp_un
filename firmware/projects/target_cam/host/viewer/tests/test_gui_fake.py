"""The whole viewer against the fake camera on a pty: the protocol paths a
bench session uses, without hardware. Slow-ish (~20 s)."""
import time

import pytest
from PyQt6.QtWidgets import QApplication

from tcview import protocol as P
from tcview.__main__ import MainWindow
from tcview.fake_cam import FakeCamera


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def rig(app):
    cam = FakeCamera(corrupt_every=4).start()
    win = MainWindow(cam.port, " test")
    win.show()
    win.connect_link()
    yield win, cam
    win.close()
    cam.stop()


def wait(app, cond, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_bench_session(app, rig):
    win, cam = rig
    cal, flight = win.calib, win.flight
    assert wait(app, lambda: cal.cfg.get("hue") == 0), "no #!CFG from the camera"
    assert wait(app, lambda: flight.last_row is not None)
    assert cal.cfg["fps"] == 12.5 and cal.cfg["period_ms"] == 80

    # The sensor is the clock: a rate is rounded to whole sensor frames, and
    # #!CFG says what was really set. Nothing is faster than every frame.
    assert cal.cfg["sensor_us"] == 79968
    win.send("f15")
    assert wait(app, lambda: win.link.pending() == 0)
    assert cal.cfg["period_ms"] == 80
    win.send("f6.25")
    assert wait(app, lambda: cal.cfg.get("period_ms") == 159.9)
    win.send("f12.5")
    assert wait(app, lambda: cal.cfg.get("period_ms") == 80 and win.link.pending() == 0)
    # A letter that takes no argument, with one, is refused, not obeyed.
    win.send("s=1")
    assert wait(app, lambda: win.link.counters.naks >= 1)
    assert cam.stream

    # Grab one frame: it arrives, its row pairs with it by t_ms.
    win.send("g")
    assert wait(app, lambda: cal.frame is not None)
    assert wait(app, lambda: cal.frame_row is not None)
    assert cal.frame_row["t_ms"] == cal.frame.t_ms

    # Move the hue control: applied only once the camera's #!CFG agrees.
    cal.p["hue"].set_local(350, send=True)
    assert wait(app, lambda: cam.cfg["hue"] == 350)
    assert wait(app, lambda: cal.p["hue"].camera == 350 and not cal.p["hue"].in_flight)
    assert cal.cfg["settings"] == "modified"

    # Out of range is refused, and the control goes back to the camera's value.
    cal.p["tol"].set_local(12)
    win.send("T95")
    assert wait(app, lambda: win.link.counters.naks >= 2)

    # Use a region as the target: same rule as 'c'.
    cal.pane.roi.setPos((60, 40))
    app.processEvents()
    cal._use_region()
    assert wait(app, lambda: cal.p["val"].camera == cal.p["val"].local
                and not any(c.in_flight for c in cal.p.values()), timeout=15)

    # Faster link, live frames, with every 4th frame corrupted: dropped, not shown.
    win.send("B921600")
    # pending() clears on the ACK of the confirming 'k'; the #!CFG it prints
    # comes on the next line, so wait for that too rather than racing it.
    assert wait(app, lambda: win.link.baud == 921600 and win.link.pending() == 0
                and cal.cfg.get("baud") == 921600)
    win.send("v1")
    n0 = win.link.counters.frames_ok
    assert wait(app, lambda: win.link.counters.frames_ok >= n0 + 5, timeout=20)
    assert win.link.counters.frames_bad >= 1

    # Save, then the flight tab returns the link to flight state.
    win.send("!save")
    assert wait(app, lambda: cal.cfg.get("settings") == "saved")
    win.tabs.setCurrentWidget(flight)
    assert wait(app, lambda: win.link.baud == P.BOOT_BAUD and cal.cfg.get("image") == 0
                and win.link.pending() == 0, timeout=15)
    assert wait(app, lambda: flight.banner.text().startswith("FLIGHT-IDENTICAL"), timeout=6)
    flight.tick()
    assert "rows/s" in flight.health.text()

    # A stage with no model is accepted, and falls back visibly: #!CFG and
    # every v2 row say what really runs.
    win.send("!stage 3")
    assert wait(app, lambda: cal.cfg.get("stage") == 3)
    assert cal.cfg["stage_eff"] == 0 and cal.cfg["csv"] == P.CSV_VERSION
    assert wait(app, lambda: flight.last_row.v["csv_v"] == 2)
    assert flight.last_row.v["stage"] == 0
    win.send("!stage 9")
    assert wait(app, lambda: win.link.counters.naks >= 3)
    flight.tick()
    assert "dt != 80" in flight.health.text() and "stage      0 running" in flight.health.text()


def test_camera_revert_when_pc_never_confirms(app):
    """Firmware side of the handshake, on the fake: no command at the new
    speed within 3 s -> back to 115200."""
    cam = FakeCamera().start()
    import os
    fd = os.open(cam.port, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b"\nB921600\n")
        assert wait(app, lambda: cam.baud == 921600, timeout=3)
        assert wait(app, lambda: cam.baud == P.BOOT_BAUD, timeout=5)
    finally:
        os.close(fd)
        cam.stop()
