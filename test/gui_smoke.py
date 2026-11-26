#!/usr/bin/env python3
"""Offscreen smoke test of both GUIs in demo mode.

The ground-station GUI flies a whole simulated mission (PREPARE, COUNTDOWN,
liftoff, apogee, drogue, main) through its real command and parsing paths;
the diagnostic GUI connects to a simulated Master and must fill every tab.

    QT_QPA_PLATFORM=offscreen python3 test/gui_smoke.py [screenshot_dir]
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gui"))

from PyQt6 import QtWidgets  # noqa: E402
import zenith_common as zc  # noqa: E402
import ground_station_gui as gsg  # noqa: E402
import zenith_diag as zd  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else None
fails = 0


def check(cond, msg):
    global fails
    print(("   ok:   " if cond else "   FAIL: ") + msg)
    if not cond:
        fails += 1


def spin(app, seconds):
    end = time.time() + seconds
    while time.time() < end:
        app.processEvents()
        time.sleep(0.005)


app = QtWidgets.QApplication(sys.argv)
app.setStyleSheet(zc.STYLE)
QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.StandardButton.Yes)

print("== ground station GUI, simulated mission (x10 speed)")
w = gsg.GroundStation(None, True, demo_speed=10.0)
w.show()
spin(app, 0.5)
check(w.t.get("state") == "1", "telemetry parsed, rocket IDLE")
w.cmd("PREPARE")
spin(app, 1.3)
check(w.t.get("state") == "2", "PREPARE acknowledged -> READY")
check("POTVRZENO" in w.ul_lb.text(), "uplink ACK shown: " + w.ul_lb.text())
w.cmd("COUNTDOWN 30")
spin(app, 1.2)
check(w.t.get("state") == "3", "COUNTDOWN running")
check(w.clock_lb.text().startswith("T-"), "countdown clock " + w.clock_lb.text())
spin(app, 4.0)
check(w.t.get("state") in ("4", "5", "6"), "liftoff -> boost/coast/drogue (state %s)" % w.t.get("state"))
if OUT:
    w.grab().save(os.path.join(OUT, "gs_flight.png"))
spin(app, 2.5)
check(int(w.t.get("pyro", "0")) & 0x08, "drogue fired flag")
check(float(w.t.get("max_alt", "0")) > 2500, "max altitude %s m" % w.t.get("max_alt"))
w.close()

print("== diagnostic GUI against a simulated Master")
import voltino_style as vs  # noqa: E402
vs.setup_app(app, "VoltinoLabs.Zenith.Diagnostika")
d = zd.DiagWindow(None, True)
d.resize(1500, 980)
d.show()
spin(app, 1.5)
check(d.s.get("state") is not None, "status line parsed")
check(len(d.param_items) >= 30, "parameter tree filled (%d parameters)" % len(d.param_items))
check(d.board_kv["M"].v["Online"].text() == "ano", "Master card online")
check(d.board_kv["P"].v["Online"].text() == "ano", "Position card online")
check(d.problems.count() >= 1, "problem list: " + d.problems.item(0).text())
check(d.nav.get("alt") is not None, "navigation parsed")
check(d.rocket._valid, "3-D rocket has an attitude")
# parameter edit -> send -> the board's $PRM echo clears the edit
d.param_set("main_alt_m", 350)
check("main_alt_m" in d.param_edits, "main_alt_m marked for sending")
d.param_send()
spin(app, 0.4)
check(not d.param_edits and d.params[d.param_rows["main_alt_m"]]["val"] == 350, "PARAM SET acknowledged")
spin(app, 0.4)
check(d.s.get("main_alt") == "350", "Master reports main_alt 350")
# DIAG -> rail on -> fire channel 2 (the demo Master needs RBF out, which it reports)
d.diag_btn.setChecked(True)
d.toggle_diag()
spin(app, 0.5)
check(d._state() == "DIAG", "DIAG mode")
d.send("ARM 1")
spin(app, 0.5)
check(d.arm_chip.text() == "LIŠTA ZAPNUTA", "pyro rail on")
d.fire(2)
spin(app, 0.6)
check(not d.pyro_rows[1]["fired"].isHidden(), "PYRO2 shows fired")
d.disarm()
d.send("DIAG 0")
spin(app, 0.5)
check(d._state() == "IDLE", "back to IDLE")
# raw stream
d.raw_btn.setChecked(True)
spin(app, 0.5)
check(d.raw.get("ax") is not None, "raw IMU stream")
if OUT:
    for i in range(d.tabs.count()):
        d.tabs.setCurrentIndex(i)
        spin(app, 0.4)
        d.grab().save(os.path.join(OUT, f"diag_{i}.png"))
import tempfile  # noqa: E402
tmp = tempfile.mkdtemp()
os.environ["ZENITH_LOG_OUT"] = os.path.join(tmp, "log.csv")
d.dump_log()
spin(app, 0.5)
rows = open(os.environ["ZENITH_LOG_OUT"]).read().splitlines()
check(len(rows) == 3, "log dump decoded to CSV (%d lines incl. header)" % len(rows))
check("BOOT demo" in open(os.path.join(tmp, "log_events.txt")).read(), "log events decoded")
d.close()

print("\n%s (%d failures)" % ("GUI SMOKE PASSED" if not fails else "GUI SMOKE FAILED", fails))
sys.exit(1 if fails else 0)
