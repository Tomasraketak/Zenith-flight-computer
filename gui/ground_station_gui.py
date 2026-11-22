#!/usr/bin/env python3
"""
Zenith — GROUND STATION GUI

Plug the ground station (Pico 2 W + Ra-02 + OLED) into the laptop. This window
shows the rocket's LoRa telemetry (or the launch pad's beacon when no rocket is
on the pogo) and sends commands. The rocket listens for 300 ms every 10 s, so
a command waits in the station until the next window; this window shows how
long that is and whether the rocket acknowledged it.

    pip install PyQt6 pyserial
    python ground_station_gui.py                # pick the port in the window
    python ground_station_gui.py --port COM7
    python ground_station_gui.py --demo         # no hardware: simulated flight

Every received line is also written to logs/gs_<date>.csv.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zenith_common import (STYLE, PALETTE, set_label_style, STATES_CZ, FLIGHT_STATES, RAMPA_STATES, RAMPA_FAULTS, UL_RC, TF, HL,  # noqa: E402
                           Chip, StripChart, SerialReader, label, list_ports, state_name, err_name, bits,
                           haversine_m, to_float, to_int)

T_FIELDS = ["rssi", "snr", "state", "flags", "t_ms", "lat", "lon", "alt", "t_rem_ms", "vz", "vh", "acc_g", "max_alt",
            "vbat", "next_rx", "main_alt", "tilt", "mach", "sats", "pyro", "health", "rampa_state", "rampa_flags",
            "ul_seq", "ul_rc", "err", "win_final"]
B_FIELDS = ["rssi", "snr", "state", "flags", "t_ms", "vbat", "umb_ma", "c1", "c2", "c3", "next_rx", "faults",
            "ul_seq", "ul_rc", "fw"]


# =====================================================================
#  Demo: a simulated station + rocket
# =====================================================================
class DemoStation(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, speed: float = 1.0):
        super().__init__()
        self.speed = speed
        self.t0 = time.time()
        self.state = 1
        self.t_launch = None
        self.cd_t0 = None
        self.pending = None
        self.seq = 0
        self.last_seq, self.last_rc = 0, 0
        self.main_alt = 300
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(20, int(200 / speed)))
        self.status.emit("DEMO")

    def send(self, text: str):
        p = text.split()
        if p[:1] == ["CMD"] and len(p) >= 2:
            self.seq = (self.seq % 255) + 1
            self.pending = (self.seq, p[1].upper(), p[2] if len(p) > 2 else "0", time.time())
            self.line.emit(f"$U,{self.seq},{p[1].upper()},0,QUEUED,0,0")
        elif p[:1] == ["CANCEL"]:
            self.pending = None

    def now(self):
        return (time.time() - self.t0) * self.speed

    def tick(self):
        t = self.now()
        self._tick_n = getattr(self, "_tick_n", 0) + 1
        # deliver the pending command at the next 10 s window
        win = int(t // 10.0)
        crossed = win != getattr(self, "_win", win)
        self._win = win
        if self.pending and crossed:
            seq, cmd, arg, _ = self.pending
            rc = 1
            if cmd == "PREPARE" and self.state in (1, 2):
                self.state = 2
            elif cmd == "COUNTDOWN" and self.state == 2:
                self.state, self.cd_t0 = 3, t + float(arg)
            elif cmd == "ABORT" and self.state in (2, 3):
                self.state, rc = 9, 6
            elif cmd == "SAFE" and self.state in (2, 9):
                self.state = 1
            elif cmd == "MAINALT":
                self.main_alt = int(arg)
            elif cmd in ("COUNTDOWN", "PREPARE"):
                rc = 3
            self.line.emit(f"$U,{seq},{cmd},0,SENT,0,1")
            self.last_seq, self.last_rc = seq, rc
            self.line.emit(f"$U,{seq},{cmd},0,ACK,{rc},1")
            self.pending = None
        alt = vz = acc = mach = 0.0
        tf = -1
        if self.state == 3 and t >= self.cd_t0:
            self.state, self.t_launch = 4, t
        if self.t_launch is not None:
            tf = t - self.t_launch
            if tf < 2.4:
                acc, vz = 21, 206 * tf
                alt = 103 * tf * tf
            elif tf < 15.0:
                c = tf - 2.4
                vz = 494 * (1 - c / 12.6) ** 1.6
                alt = 593 + 494 * 12.6 / 2.6 * (1 - (1 - c / 12.6) ** 2.6)
                acc = -2.0 * math.exp(-c / 4)
            else:
                c = tf - 15.0
                top = getattr(self, "apo", 3000.0)
                alt = max(0.0, top - 25 * c) if top - 25 * c > self.main_alt else max(0.0, self.main_alt - 6 * (c - (top - self.main_alt) / 25))
                vz = -25 if alt > self.main_alt else (-6 if alt > 0 else 0)
                acc = 1.0
            if tf < 15.0:
                self.apo = alt
            self.max_alt = max(getattr(self, "max_alt", 0.0), alt)
            mach = abs(vz) / 340
            self.state = 4 if tf < 2.4 else 5 if tf < 15.0 else 6 if alt > self.main_alt else 7 if alt > 0 else 8
        rem = int((self.cd_t0 - t) * 1000) if self.state == 3 else (int(tf * 1000) if tf >= 0 else -1)
        nxt = int((10.0 - t % 10.0) * 1000)
        pyro = 0x07 | (0x40 if self.state >= 3 and self.state != 9 else 0)
        if self.state >= 6 and self.state != 9:
            pyro |= 0x08
        if self.state >= 7 and self.state != 9:
            pyro |= 0x10
        flags = 0x02 | 0x04 | 0x08 | (0x10 if self.state < 4 else 0) | 0x40 | (0x80 if self.state in (1, 2, 3) else 0)
        if pyro & 0x40:
            flags |= 0x01
        lat = 49.7985411 + (alt / 111000) * 0.05
        lon = 16.6912772 + (tf if tf > 0 else 0) * 0.00002
        rampa = 3 if self.state == 3 else 2 if self.state == 2 else 5 if self.state == 4 else 1 if self.state < 4 else 0
        vals = [-62 - int(alt / 200), 9.0, self.state, flags, int(t * 1000), f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
                rem, f"{vz:.1f}", "3.0", f"{acc:.2f}", int(max(getattr(self, "max_alt", 0.0), alt, 0)), "8.05", nxt, self.main_alt, "3.2",
                f"{mach:.2f}", 13, pyro, 0x3F, rampa, 0x87 if rampa else 0, self.last_seq, self.last_rc, 0, 0]
        self.line.emit("$T," + ",".join(str(v) for v in vals))
        if self._tick_n % 5 == 0:
            self.line.emit(f"$G,{int(t * 5)},0,100,1,{nxt},{1 if self.pending else 0},0,-1")

    def stop(self):
        self.timer.stop()


# =====================================================================
#  Main window
# =====================================================================
class GroundStation(QtWidgets.QMainWindow):
    def __init__(self, port: str | None, demo: bool, demo_speed: float = 1.0):
        super().__init__()
        self.setWindowTitle("Zenith – pozemní stanice")
        self.resize(1400, 900)
        self.reader = None
        self.t: dict = {}
        self.b: dict = {}
        self.t_rx = 0.0
        self.b_rx = 0.0
        self.win_in_ms = -1
        self.win_rx = 0.0
        self.pad = None
        self.max_seen = 0.0
        self.t_start = time.time()
        os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"), exist_ok=True)
        self.csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs",
                                     time.strftime("gs_%Y%m%d_%H%M%S.csv"))
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(["pc_time", "type"] + T_FIELDS)

        self._build()
        QtCore.QTimer(self, timeout=self.refresh, interval=100).start()
        self.demo_speed = demo_speed
        if demo:
            self.start_demo()
        elif port:
            self.connect_port(port)

    # ------------------------------------------------------------------ layout
    def _build(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        v = QtWidgets.QVBoxLayout(root)

        # --- connection bar
        bar = QtWidgets.QHBoxLayout()
        self.port_cb = QtWidgets.QComboBox()
        self.port_cb.setMinimumWidth(160)
        self.port_cb.addItems(list_ports())
        bt_ref = QtWidgets.QPushButton("↻")
        bt_ref.clicked.connect(lambda: (self.port_cb.clear(), self.port_cb.addItems(list_ports())))
        bt_con = QtWidgets.QPushButton("Připojit")
        bt_con.clicked.connect(lambda: self.connect_port(self.port_cb.currentText()))
        bt_demo = QtWidgets.QPushButton("Demo")
        bt_demo.clicked.connect(self.start_demo)
        self.conn_lb = label("nepřipojeno", color=PALETTE["muted"])
        self.link_chip = Chip("LINK")
        self.rssi_lb = label("RSSI –")
        self.win_lb = label("okno: –")
        for w in (label("Port:"), self.port_cb, bt_ref, bt_con, bt_demo, self.conn_lb):
            bar.addWidget(w)
        bar.addStretch(1)
        for w in (self.link_chip, self.rssi_lb, self.win_lb):
            bar.addWidget(w)
        v.addLayout(bar)

        main = QtWidgets.QHBoxLayout()
        v.addLayout(main, 1)

        # --- left: big flight numbers + charts
        left = QtWidgets.QVBoxLayout()
        main.addLayout(left, 3)
        top = QtWidgets.QGroupBox("Let")
        g = QtWidgets.QGridLayout(top)
        self.state_lb = label("—", 26, True)
        self.state_cz = label("", 13, color=PALETTE["muted"])
        self.clock_lb = label("", 30, True, PALETTE["warn"])
        g.addWidget(self.state_lb, 0, 0, 1, 2)
        g.addWidget(self.state_cz, 1, 0, 1, 2)
        g.addWidget(self.clock_lb, 0, 2, 2, 2, Qt.AlignmentFlag.AlignRight)
        self.big = {}
        for i, (k, name, unit) in enumerate([("alt", "Výška", "m"), ("vz", "Vert. rychlost", "m/s"),
                                             ("max_alt", "Max. výška", "m"), ("mach", "Mach", ""),
                                             ("acc_g", "Zrychlení", "g"), ("tilt", "Náklon", "°"),
                                             ("vh", "Horiz. rychlost", "m/s"), ("dist", "Vzdálenost od rampy", "m")]):
            box = QtWidgets.QVBoxLayout()
            box.addWidget(label(name, 11, color=PALETTE["muted"]))
            val = label("—", 22, True)
            box.addWidget(val)
            box.addWidget(label(unit, 10, color=PALETTE["muted"]))
            self.big[k] = val
            g.addLayout(box, 2 + i // 4, i % 4)
        left.addWidget(top)
        self.ch_alt = StripChart("Výška", [("alt", PALETTE["info"]), ("max", PALETTE["muted"])], 120, "m")
        self.ch_vz = StripChart("Vertikální rychlost", [("vz", PALETTE["accent"])], 120, "m/s")
        left.addWidget(self.ch_alt, 1)
        left.addWidget(self.ch_vz, 1)

        # --- right: status + commands
        right = QtWidgets.QVBoxLayout()
        main.addLayout(right, 2)

        st = QtWidgets.QGroupBox("Stav rakety")
        sg = QtWidgets.QGridLayout(st)
        self.go_chip = Chip("GO / NO-GO")
        self.go_chip.setMinimumHeight(34)
        self.err_lb = label("", 12, True)
        self.err_lb.setWordWrap(True)
        sg.addWidget(self.go_chip, 0, 0, 1, 4)
        sg.addWidget(self.err_lb, 1, 0, 1, 4)
        self.chips = {}
        names = [("c1", "PYRO1 drogue"), ("c2", "PYRO2 main"), ("c3", "PYRO3 záloha"), ("rail", "Pyro lišta"),
                 ("f1", "Drogue vypuštěn"), ("f2", "Main vypuštěn"), ("f3", "Záloha vypálena"), ("rbf", "RBF venku"),
                 ("pos", "Position MCU"), ("nav", "Navigace"), ("baro", "Baro důvěra"), ("gps", "GPS fix"),
                 ("umb", "Pogo 5 V"), ("batt", "Na baterii"), ("flash", "Log flash"), ("cam", "Kamera")]
        for i, (k, n) in enumerate(names):
            c = Chip(n)
            self.chips[k] = c
            sg.addWidget(c, 2 + i // 4, i % 4)
        self.info_lb = label("", 12)
        self.info_lb.setWordWrap(True)
        sg.addWidget(self.info_lb, 6, 0, 1, 4)
        right.addWidget(st)

        rp = QtWidgets.QGroupBox("Rampa")
        rl = QtWidgets.QVBoxLayout(rp)
        self.rampa_lb = label("—", 13, True)
        self.rampa_lb.setWordWrap(True)
        rl.addWidget(self.rampa_lb)
        rr = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Napájení rakety ZAP")
        b1.clicked.connect(lambda: self.cmd("ROCKETPWR 1 RAMPA"))
        b2 = QtWidgets.QPushButton("Napájení rakety VYP")
        b2.clicked.connect(lambda: self.confirm("Vypnout napájení rakety z rampy?", "ROCKETPWR 0 RAMPA"))
        rr.addWidget(b1)
        rr.addWidget(b2)
        rl.addLayout(rr)
        right.addWidget(rp)

        cm = QtWidgets.QGroupBox("Příkazy (1 příkaz za 10 s okno)")
        cg = QtWidgets.QGridLayout(cm)
        self.abort_bt = QtWidgets.QPushButton("ABORT")
        self.abort_bt.setObjectName("abort")
        self.abort_bt.clicked.connect(lambda: self.cmd("ABORT"))
        cg.addWidget(self.abort_bt, 0, 0, 1, 4)
        bp = QtWidgets.QPushButton("PŘIPRAVIT (launch soon)")
        bp.setObjectName("go")
        bp.clicked.connect(lambda: self.confirm("Přepnout raketu do READY (pad reference, kamera, rampa PREPARE)?", "PREPARE"))
        cg.addWidget(bp, 1, 0, 1, 2)
        self.cd_spin = QtWidgets.QSpinBox()
        self.cd_spin.setRange(10, 3600)
        self.cd_spin.setValue(30)
        self.cd_spin.setSuffix(" s")
        bcd = QtWidgets.QPushButton("START ODPOČTU")
        bcd.setObjectName("go")
        bcd.clicked.connect(lambda: self.confirm(
            f"Spustit odpočet {self.cd_spin.value()} s?\nRampa zažehne motor v T-0, pokud raketa i rampa budou stále GO.",
            f"COUNTDOWN {self.cd_spin.value()}"))
        cg.addWidget(self.cd_spin, 1, 2)
        cg.addWidget(bcd, 1, 3)
        for i, (txt, c) in enumerate([("SAFE / zpět do IDLE", "SAFE"), ("Pad reference", "PADREF"),
                                       ("Bzučák ZAP", "LOCATE 1"), ("Bzučák VYP", "LOCATE 0"),
                                       ("Kamera nahrávat", "CAMERA 1"), ("Kamera stop", "CAMERA 0")]):
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(lambda _=False, c=c: self.cmd(c))
            cg.addWidget(b, 2 + i // 2, (i % 2) * 2, 1, 2)
        self.main_spin = QtWidgets.QSpinBox()
        self.main_spin.setRange(100, 1000)
        self.main_spin.setSingleStep(25)
        self.main_spin.setValue(300)
        self.main_spin.setSuffix(" m")
        bm = QtWidgets.QPushButton("Nastavit výšku hlavního padáku")
        bm.clicked.connect(lambda: self.cmd(f"MAINALT {self.main_spin.value()}"))
        cg.addWidget(self.main_spin, 5, 0)
        cg.addWidget(bm, 5, 1, 1, 3)
        brb = QtWidgets.QPushButton("Restart rakety")
        brb.clicked.connect(lambda: self.confirm("Restartovat Master MCU rakety? (jen na zemi)", "REBOOT"))
        bcan = QtWidgets.QPushButton("Zrušit čekající příkaz")
        bcan.clicked.connect(lambda: self.send("CANCEL"))
        cg.addWidget(brb, 6, 0, 1, 2)
        cg.addWidget(bcan, 6, 2, 1, 2)
        self.ul_lb = label("žádný příkaz", 12, True)
        self.ul_lb.setWordWrap(True)
        cg.addWidget(self.ul_lb, 7, 0, 1, 4)
        right.addWidget(cm)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        right.addWidget(self.log, 1)

        sc = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Shift+A"), self)
        sc.activated.connect(lambda: self.cmd("ABORT"))

    # ------------------------------------------------------------------ comms
    def connect_port(self, port: str):
        if not port:
            return
        self.stop_reader()
        self.reader = SerialReader(port)
        self.reader.line.connect(self.on_line)
        self.reader.status.connect(lambda s: self.conn_lb.setText(s))
        self.reader.start()
        QtCore.QTimer.singleShot(800, lambda: self.send("HELLO"))

    def start_demo(self):
        self.stop_reader()
        self.reader = DemoStation(getattr(self, "demo_speed", 1.0))
        self.reader.line.connect(self.on_line)
        self.conn_lb.setText("DEMO – simulovaný let")

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None

    def send(self, text: str):
        if self.reader is None:
            self.note("nepřipojeno")
            return
        self.reader.send(text)

    def cmd(self, c: str):
        self.send("CMD " + c)
        self.note(f"→ {c} (čeká na okno)")

    def confirm(self, question: str, c: str):
        if QtWidgets.QMessageBox.question(self, "Potvrdit", question) == QtWidgets.QMessageBox.StandardButton.Yes:
            self.cmd(c)

    def note(self, s: str):
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + s)

    # ------------------------------------------------------------------ parsing
    def on_line(self, s: str):
        p = s.split(",")
        tag = p[0]
        if tag == "$T" and len(p) >= len(T_FIELDS) + 1:
            prev_state = self.t.get("state")
            self.t = {k: p[i + 1] for i, k in enumerate(T_FIELDS)}
            self.t_rx = time.time()
            self.win_in_ms, self.win_rx = to_int(self.t["next_rx"]), time.time()
            self.csv.writerow([f"{time.time():.3f}", "T"] + [self.t[k] for k in T_FIELDS])
            st = to_int(self.t["state"])
            if prev_state is not None and to_int(prev_state) != st:
                self.note(f"STAV: {state_name(st)}")
            if st in (1, 2) and to_float(self.t["lat"]) != 0:
                self.pad = (to_float(self.t["lat"]), to_float(self.t["lon"]))
            tt = time.time() - self.t_start
            self.ch_alt.add(tt, alt=to_float(self.t["alt"]), max=to_float(self.t["max_alt"]))
            self.ch_vz.add(tt, vz=to_float(self.t["vz"]))
        elif tag == "$B" and len(p) >= len(B_FIELDS) + 1:
            self.b = {k: p[i + 1] for i, k in enumerate(B_FIELDS)}
            self.b_rx = time.time()
            if not self.t or time.time() - self.t_rx > 5:
                self.win_in_ms, self.win_rx = to_int(self.b["next_rx"]), time.time()
            self.csv.writerow([f"{time.time():.3f}", "B"] + [self.b[k] for k in B_FIELDS])
        elif tag == "$U" and len(p) >= 7:
            seq, name, what, rc = p[1], p[2], p[4], to_int(p[5])
            txt = {"QUEUED": "ve frontě – čeká na okno", "SENT": "odesláno", "SENT2": "odesláno 2×",
                   "FAILED": "NEPOTVRZENO po 3 oknech", "CANCELLED": "zrušeno"}.get(what, what)
            if what == "ACK":
                txt = "POTVRZENO: " + (UL_RC[rc] if rc < len(UL_RC) else str(rc))
            col = PALETTE["ok"] if what == "ACK" and rc in (1, 6) else PALETTE["bad"] if what in ("ACK", "FAILED") else PALETTE["warn"]
            self.ul_lb.setText(f"#{seq} {name}: {txt}")
            set_label_style(self.ul_lb, col)
            self.note(f"#{seq} {name}: {txt}")
        elif tag == "$G" and len(p) >= 6:
            w = to_int(p[5], -1)
            if to_int(p[4]) == 1 and w >= 0 and (not self.t or time.time() - self.t_rx > 1.5):
                self.win_in_ms, self.win_rx = w, time.time()
        elif tag.startswith("#") or tag == "$ID":
            self.note(s)

    # ------------------------------------------------------------------ display
    def refresh(self):
        now = time.time()
        age = now - self.t_rx if self.t else 1e9
        bage = now - self.b_rx if self.b else 1e9
        self.link_chip.set(age < 2 or bage < 3, "LINK" if (age < 2 or bage < 3) else f"ZTRÁTA {min(age, bage):.0f}s")
        if self.win_in_ms >= 0:
            rem = self.win_in_ms / 1000 - (now - self.win_rx)
            while rem < -0.3:
                rem += 10
            self.win_lb.setText(f"příští okno za {max(rem, 0):4.1f} s")
        if not self.t:
            if self.b:
                self.show_rampa_only()
            return
        t = self.t
        self.rssi_lb.setText(f"RSSI {t['rssi']} dBm  SNR {t['snr']} dB  ({age:.1f} s)")
        st = to_int(t["state"])
        name = state_name(st)
        self.state_lb.setText(name)
        set_label_style(self.state_lb, PALETTE['bad'] if name == 'ABORT' else PALETTE['info'] if name in FLIGHT_STATES else PALETTE['text'])
        self.state_cz.setText(STATES_CZ.get(name, ""))
        rem = to_int(t["t_rem_ms"], -1)
        if name == "COUNTDOWN" and rem >= 0:
            self.clock_lb.setText(f"T-{max(0.0, rem / 1000 - age):6.1f} s")
        elif rem >= 0 and (name in FLIGHT_STATES or name == "LANDED"):
            self.clock_lb.setText(f"T+{rem / 1000 + (age if name != 'LANDED' else 0):6.1f} s")
        else:
            self.clock_lb.setText("")
        for k in ("alt", "vz", "max_alt", "mach", "acc_g", "tilt", "vh"):
            v = to_float(t[k])
            self.big[k].setText(f"{v:.0f}" if k in ("alt", "max_alt") else f"{v:.2f}" if k == "mach" else f"{v:.1f}")
        lat, lon = to_float(t["lat"]), to_float(t["lon"])
        if self.pad and lat:
            self.big["dist"].setText(f"{haversine_m(self.pad[0], self.pad[1], lat, lon):.0f}")
        flags, health, pyro = to_int(t["flags"]), to_int(t["health"]), to_int(t["pyro"])
        err = to_int(t["err"])
        go = bool(flags & 0x80)
        self.go_chip.set(go if name in ("IDLE", "READY", "COUNTDOWN") else None,
                         "GO" if go else "NO-GO" if name in ("IDLE", "READY", "COUNTDOWN") else name)
        self.err_lb.setText("" if err == 0 else ("Důvod přerušení: " if name == "ABORT" else "NO-GO: ") + err_name(err))
        set_label_style(self.err_lb, PALETTE['bad'])
        c = self.chips
        for i in range(3):
            c[f"c{i + 1}"].set(bool(pyro & (1 << i)))
            fired = bool(pyro & (8 << i))
            c[f"f{i + 1}"].set(True if fired else None)
        c["rail"].set("warn" if pyro & 0x40 else None, "Pyro lišta ZAP" if pyro & 0x40 else "Pyro lišta vyp")
        c["rbf"].set(bool(flags & 0x02))
        c["pos"].set(bool(health & 1))
        c["nav"].set(bool(health & 2))
        c["baro"].set(True if health & 4 else "warn")
        c["gps"].set(bool(health & 8))
        c["umb"].set(True if flags & 0x10 else None)
        c["batt"].set("info" if flags & 0x20 else None)
        c["flash"].set(bool(health & 16))
        c["cam"].set(True if health & 64 else None)
        self.info_lb.setText(
            f"Baterie {to_float(t['vbat']):.2f} V · GPS {t['sats']} sat · {lat:.6f}, {lon:.6f} · "
            f"main padák {t['main_alt']} m · breakwire {'1' if flags & 4 else '-'}{'2' if flags & 8 else '-'} · "
            f"bzučák {'ZAP' if health & 128 else 'vyp'}")
        rs = to_int(t["rampa_state"])
        rf = to_int(t["rampa_flags"])
        if rs == 0:
            self.rampa_lb.setText("Rampa: OFFLINE (pogo odpojeno – po startu normální)")
            return
        self.rampa_lb.setText(f"Rampa (přes raketu): {RAMPA_STATES[rs] if rs < len(RAMPA_STATES) else rs} · "
                              f"klíč {'ZAP' if rf & 1 else 'vyp'} · rozbuška {'OK' if rf & 0x0E else 'chybí'} · "
                              f"{'připravena' if rf & 0x80 else 'nepřipravena'}")

    def show_rampa_only(self):
        b = self.b
        st = to_int(b["state"])
        flt = bits(to_int(b["faults"]), RAMPA_FAULTS)
        self.state_lb.setText("RAMPA BEZ RAKETY")
        self.state_cz.setText("raketa není na pogo konektoru – rampa vysílá sama")
        self.rssi_lb.setText(f"RSSI {b['rssi']} dBm  SNR {b['snr']} dB")
        self.rampa_lb.setText(
            f"{RAMPA_STATES[st] if st < len(RAMPA_STATES) else st} · baterie {to_float(b['vbat']):.2f} V · "
            f"pogo {b['umb_ma']} mA · kontinuita {b['c1']}/{b['c2']}/{b['c3']} mV · napájení rakety "
            f"{'ZAP' if to_int(b['flags']) & 0x40 else 'VYP'}" + (" · chyby: " + ", ".join(flt) if flt else ""))

    def closeEvent(self, ev):
        self.stop_reader()
        self.csv_f.close()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--demo-speed", type=float, default=1.0, help="simulated time runs this many times faster")
    ap.add_argument("--screenshot", help="save a screenshot after N seconds: path")
    ap.add_argument("--seconds", type=float, default=3.0)
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = GroundStation(a.port, a.demo, a.demo_speed)
    w.show()
    if a.screenshot:
        QtCore.QTimer.singleShot(int(a.seconds * 1000), lambda: (w.grab().save(a.screenshot), app.quit()))
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
