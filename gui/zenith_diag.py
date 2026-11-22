#!/usr/bin/env python3
"""
Zenith — DIAGNOSTIC CONSOLE (flight computer on a USB cable)

Plug the Zenith's MASTER MCU (Raspberry Pi Pico 2, U15) into the laptop. The
window shows the whole vehicle: Go/No-Go checklist, power, pyro continuity,
rampa, the navigation solution (attitude, altitude, Mach, flags), raw sensors,
GPS, the parameter table (edit + save to flash — e.g. the main parachute
altitude), ground tests in DIAG mode (pyro fire with confirmation, buzzer,
camera, battery switch) and the flight log download.

    pip install PyQt6 pyserial
    python zenith_diag.py                # pick the port in the window
    python zenith_diag.py --port COM5
    python zenith_diag.py --demo         # no hardware: a simulated Master

The protocol is described in docs/USB_PROTOCOL.md.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import random
import struct
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zenith_common as zc  # noqa: E402
import voltino_style as vs  # noqa: E402
from voltino_style import Chip, Dot, Card  # noqa: E402
vs.apply_to_common(zc)     # charts and attitude view in the VoltinoLabs colours
from zenith_common import (PALETTE, STATES_CZ, RAMPA_STATES, RAMPA_FAULTS, NAV_FLAGS, NAV_MODES,  # noqa: E402
                           SENS_FLAGS, WHY, StripChart, AttitudeView, SerialReader, label, set_label_style,
                           list_ports, state_name, err_name, bits, to_float, to_int)

S_FIELDS = ["state", "err", "flags", "health", "t_ms", "vbat_mv", "curr_ma", "pyro", "ads0", "ads1", "ads2", "ads3",
            "rampa_state", "rampa_flags", "rampa_faults", "rampa_vbat", "t_rem", "bw1", "bw2", "umb", "rbf",
            "umb_mv", "lora", "flash", "main_alt", "log_head", "log_dropped", "log_erasing", "reset"]
N_FIELDS = ["alt", "vz", "px", "py", "vx", "vy", "acc_axial", "acc_mag", "mach", "baro_alt", "sig_alt", "tilt",
            "qw", "qx", "qy", "qz", "max_alt", "flags", "mode"]
P_FIELDS = ["lat", "lon", "galt", "sats", "fix", "hacc", "pressure", "temp", "roll", "pitch", "yaw", "gb0", "gb1",
            "gb2", "peak_g", "imu_hz", "sensors", "nose", "imu_lost", "fw", "reset", "pad_lat", "pad_lon"]
R_FIELDS = ["ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "hgw"]
F_FIELDS = ["why_d", "why_m", "why_p3", "fired", "max_alt", "max_vz", "max_mach", "max_g", "pyro_events"]

LOG_DATA = struct.Struct("<BBBBI7fiiHhHhhHBBBBhH")
LOG_EVENT = struct.Struct("<BBHI54sH")
DIV = 47.0 / 147.0
AXES = ["auto", "+X", "-X", "+Y", "-Y", "+Z", "-Z"]


def crc16(b: bytes) -> int:
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


# =====================================================================
#  Demo Master
# =====================================================================
class DemoMaster(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    PARAMS = [("main_alt_m", 300, 100, 1000, "Vyska vypusteni hlavniho padaku"), ("drogue_fail_vz", 35, 10, 150, "Selhani drogue"),
              ("main_fail_vz", 15, 4, 60, "Selhani main"), ("drogue_check_s", 4, 1, 20, "kontrola drogue"),
              ("main_check_s", 4, 1, 20, "kontrola main"), ("apo_lockout_s", 6, 1, 60, "lockout"),
              ("apo_mach_max", 0.5, 0.2, 0.9, "mach"), ("apo_backup_s", 25, 3, 180, "backup"),
              ("apo_max_s", 35, 3, 300, "max"), ("main_backup_s", 100, 3, 900, "main backup"),
              ("p3_enable", 1, 0, 1, "p3"), ("p3_blind_s", 8, 0, 900, "p3 blind"), ("pyro_pulse_ms", 300, 20, 2000, "pulse"),
              ("liftoff_g", 2.5, 1.5, 10, "liftoff"), ("burn_max_s", 6, 0.5, 30, "burn"), ("cd_force_bat_s", 10, 3, 120, "bat"),
              ("cd_arm_s", 5, 1, 60, "arm"), ("ign_wait_s", 5, 1, 30, "ign"), ("rampa_ch", 1, 1, 3, "ch"),
              ("rampa_fire_ms", 1000, 100, 5000, "fire"), ("vbat_min_v", 7.2, 6, 8.4, "vbat"), ("req_cont3", 0, 0, 1, "c3"),
              ("max_tilt_deg", 20, 2, 60, "tilt"), ("nose_axis", 0, 0, 6, "nose"), ("tlm_hz", 5, 1, 10, "tlm"),
              ("locate_after_s", 30, 0, 900, "locate"), ("cam_mode", 1, 0, 1, "cam"), ("ext_ign", 0, 0, 1, "ext"),
              ("bw_mode", 1, 0, 2, "bw"), ("decl_deg", 5, -30, 30, "decl"), ("landed_still_s", 5, 2, 60, "landed"),
              ("cont_min_mv", 300, 50, 3000, "cont"), ("main_min_delay_s", 1.5, 0.5, 10, "delay"),
              ("ready_timeout_min", 30, 1, 240, "ready")]

    def __init__(self):
        super().__init__()
        self.t0 = time.time()
        self.state, self.raw = 1, False
        self.vals = {p[0]: p[1] for p in self.PARAMS}
        self.armed = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.n = 0

    def stop(self):
        self.timer.stop()

    def send(self, text: str):
        p = text.split()
        if not p:
            return
        c = p[0].upper()
        if c == "HELLO":
            self.line.emit("$ID,Zenith Master,1.00,34 (DEMO)")
        elif c == "PARAM" and len(p) > 1 and p[1].upper() == "LIST":
            for i, (n, d, lo, hi, desc) in enumerate(self.PARAMS):
                self.line.emit(f"$PRM,{i},{n},{self.vals[n]},{lo},{hi},{d},{desc}, demo")
            self.line.emit("#OK PARAM LIST")
        elif c == "PARAM" and len(p) > 3 and p[1].upper() == "SET":
            self.vals[p[2]] = float(p[3])
            self.line.emit(f"#OK {p[2]}={p[3]}")
        elif c == "PARAM":
            self.line.emit("#OK PARAM " + " ".join(p[1:]))
        elif c == "DIAG":
            self.state = 10 if p[1] == "1" else 1
            self.line.emit("#OK DIAG")
        elif c == "RAW":
            self.raw = p[1] == "1"
            self.line.emit("#OK RAW")
        elif c == "ARM":
            self.armed = p[1] == "1"
            self.line.emit("#OK ARM")
        elif c == "LOG" and len(p) > 1 and p[1].upper() == "DUMP":
            self.line.emit("$LOGBEGIN,3")
            ev = LOG_EVENT.pack(0xA8, 1, 0, 1234, b"BOOT demo", 0)
            ev = ev[:62] + struct.pack("<H", crc16(ev[:62]))
            self.line.emit("$LOG," + ev.hex().upper())
            for k in range(2):
                d = LOG_DATA.pack(0xA7, 4, 0, 0x47, 2000 + k * 10, 10.0 * k, 50.0, 9.0, 12.0, 12.1, 0.2, 3.0,
                                  497985411, 166912772, 8050, 900, 0x1FFF, 0, 0, 10, 0, 1, 12, 2, 0, 0)
                d = d[:62] + struct.pack("<H", crc16(d[:62]))
                self.line.emit("$LOG," + d.hex().upper())
            self.line.emit("$LOGEND,3")
        elif c == "LOG":
            self.line.emit("$LOGINFO,3,262016,0,0,15669272")
        elif c in ("PREPARE", "SAFE", "ABORT", "COUNTDOWN", "PADREF", "REBOOT", "LOCATE", "CAM"):
            self.state = {"PREPARE": 2, "SAFE": 1, "ABORT": 9, "COUNTDOWN": 3}.get(c, self.state)
            self.line.emit(f"#RC {c} 1")
        else:
            self.line.emit(f"#OK {text}")

    def tick(self):
        self.n += 1
        t = time.time() - self.t0
        roll = 4 * math.sin(t * 0.7)
        if self.n % 2 == 0:
            q = (math.cos(math.radians(45 + roll / 2)), 0.0, -math.sin(math.radians(45 + roll / 2)), 0.0)
            self.line.emit("$N," + ",".join(f"{v:.4f}" for v in (
                0.1 * math.sin(t), 0.02 * math.cos(t), 0.2, -0.1, 0, 0, 0.998, 1.001, 0.0, 0.12 * math.sin(t * 2), 0.6,
                abs(roll) + 3.0, *q, 0.3)) + ",63511,1")
        if self.n % 2 == 1:
            self.line.emit(f"$S,{self.state},0,{0xD6 | (1 if self.armed else 0)},63,{int(t * 1000)},8040,"
                           f"{int(180 + 20 * math.sin(t))},{0x07 | (0x40 if self.armed else 0)},2480,2470,2465,{2600 if self.armed else 2400},"
                           f"1,135,0,7950,-1,1,1,1,1,2510,1,1,{int(self.vals['main_alt_m'])},1234,0,0,0")
        if self.n % 4 == 0:
            self.line.emit(f"$P,49.7985411,16.6912772,{412.3 + random.random():.1f},13,3,1.8,98134.5,23.4,"
                           f"{roll:.1f},{-86.0 + roll / 3:.1f},123.0,0.120,-0.310,0.050,1.2,960,63,1,0,1.00,0,49.7985400,16.6912700")
        if self.raw:
            self.line.emit(f"$R,{0.998 + random.gauss(0, 0.002):.4f},{0.01 + random.gauss(0, 0.002):.4f},"
                           f"{-0.02 + random.gauss(0, 0.002):.4f},0.12,-0.31,0.05,21.0,-3.5,-44.2,0.00")
        if self.n % 20 == 0:
            self.line.emit("$F,0,0,0,0,0.0,0.0,0.00,0.0,0")


# =====================================================================
#  Main window
# =====================================================================
class DiagWindow(QtWidgets.QMainWindow):
    def __init__(self, port: str | None, demo: bool):
        super().__init__()
        self.setWindowTitle("Zenith – diagnostika letového počítače")
        self.resize(1450, 920)
        self.reader = None
        self.s: dict = {}
        self.nav: dict = {}
        self.pos: dict = {}
        self.raw: dict = {}
        self.fl: dict = {}
        self.params: dict = {}
        self.t0 = time.time()
        self.s_rx = 0.0
        self.log_lines: list[str] = []
        self.dumping = False
        self._build()
        QtCore.QTimer(self, timeout=self.refresh, interval=150).start()
        if demo:
            self.start_demo()
        elif port:
            self.connect_port(port)

    # ------------------------------------------------------------------ ui
    def _build(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        v = QtWidgets.QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        hdr, bar = vs.header("ZENITH V2.4 · MASTER · DIAGNOSTIKA", "Zenith", " Diagnostika")
        self.port_cb = QtWidgets.QComboBox()
        self.port_cb.setMinimumWidth(190)
        self.port_cb.setToolTip("Sériový port Masteru (USB)")
        self.port_cb.addItems(list_ports())
        b_ref = vs.button("Obnovit", tip="Znovu vyhledat porty")
        b_ref.clicked.connect(lambda: (self.port_cb.clear(), self.port_cb.addItems(list_ports())))
        b_con = vs.button("Připojit", "primary")
        b_con.clicked.connect(lambda: self.connect_port(self.port_cb.currentText()))
        b_demo = vs.button("Demo", tip="Simulovaný Master bez hardwaru")
        b_demo.clicked.connect(self.start_demo)
        self.conn_lb = vs.label("nepřipojeno", "subtle")
        self.state_chip = Chip("—")
        self.state_chip.setMinimumWidth(140)
        self.go_chip = Chip("GO / NO-GO")
        self.go_chip.setMinimumWidth(300)
        for w in (self.state_chip, self.go_chip, self.port_cb, b_ref, b_con, b_demo, self.conn_lb):
            bar.addWidget(w)
        v.addWidget(hdr)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        v.addWidget(self.tabs, 1)
        self._page = lambda w: (w.layout().setContentsMargins(20, 16, 20, 16), w)[1]
        self.tabs.addTab(self._page(self._tab_overview()), "Přehled")
        self.tabs.addTab(self._page(self._tab_nav()), "Navigace")
        self.tabs.addTab(self._page(self._tab_sensors()), "Senzory a GPS")
        self.tabs.addTab(self._page(self._tab_params()), "Parametry")
        self.tabs.addTab(self._page(self._tab_tests()), "Testy (DIAG)")
        self.tabs.addTab(self._page(self._tab_log()), "Letový log")
        self.tabs.addTab(self._page(self._tab_console()), "Konzole")

    def _btn(self, text, fn, obj=None):
        kind = {"abort": "danger", "go": "primary"}.get(obj or "", "")
        b = vs.button(text, kind, big=(obj == "abort"))
        b.clicked.connect(fn)
        return b

    def _tab_overview(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        gb = Card("Go / No-Go", "Podmínky startu")
        gl = QtWidgets.QGridLayout(gb.body)
        self.checks = {}
        items = [("pos", "Position MCU komunikuje"), ("nav", "Navigace platná"), ("hw", "INA226 / ADS1115 / PCA9685"),
                 ("bat", "Baterie ≥ vbat_min_v"), ("c1", "PYRO1 drogue – kontinuita"), ("c2", "PYRO2 main – kontinuita"),
                 ("c3", "PYRO3 záloha – kontinuita"), ("rbf", "RBF vytažen (lišta lze zapnout)"), ("tilt", "Raketa svisle"),
                 ("rampa", "Rampa online"), ("rampa_ready", "Rampa připravena (klíč, rozbuška)"), ("flash", "Log flash"),
                 ("lora", "LoRa rádio"), ("gps", "GPS fix")]
        for i, (k, n) in enumerate(items):
            c = Dot()
            self.checks[k] = c
            gl.addWidget(c, i, 0)
            gl.addWidget(label(n), i, 1)
        self.err_lb = label("", 13, True, PALETTE["bad"])
        self.err_lb.setWordWrap(True)
        gl.addWidget(self.err_lb, len(items), 0, 1, 2)
        g.addWidget(gb, 0, 0, 2, 1)

        pw = Card("Napájení a pyro", "Baterie, lišta, kontinuita")
        pl = QtWidgets.QVBoxLayout(pw.body)
        self.pow_lb = label("", 13)
        self.pow_lb.setWordWrap(True)
        pl.addWidget(self.pow_lb)
        self.pyro_lb = label("", 13)
        self.pyro_lb.setWordWrap(True)
        pl.addWidget(self.pyro_lb)
        g.addWidget(pw, 0, 1)

        rw = Card("Rampa · pogo UART", "Stav rampy")
        rl = QtWidgets.QVBoxLayout(rw.body)
        self.rampa_lb = label("", 13)
        self.rampa_lb.setWordWrap(True)
        rl.addWidget(self.rampa_lb)
        g.addWidget(rw, 1, 1)

        mw = Card("Mise", "Stejné příkazy jako ze země")
        ml = QtWidgets.QGridLayout(mw.body)
        ml.addWidget(self._btn("ABORT", lambda: self.send("ABORT"), "abort"), 0, 0, 1, 4)
        ml.addWidget(self._btn("PŘIPRAVIT (READY)", lambda: self.send("PREPARE"), "go"), 1, 0, 1, 2)
        self.cd_spin = QtWidgets.QSpinBox()
        self.cd_spin.setRange(10, 3600)
        self.cd_spin.setValue(30)
        self.cd_spin.setSuffix(" s")
        ml.addWidget(self.cd_spin, 1, 2)
        ml.addWidget(self._btn("ODPOČET", self.countdown, "go"), 1, 3)
        ml.addWidget(self._btn("SAFE / IDLE", lambda: self.send("SAFE")), 2, 0)
        ml.addWidget(self._btn("Pad reference", lambda: self.send("PADREF")), 2, 1)
        ml.addWidget(self._btn("Kalibrace gyra", lambda: self.send("GYROCAL")), 2, 2)
        ml.addWidget(self._btn("Restart Position MCU", lambda: self.send("POSREBOOT")), 2, 3)
        self.flight_lb = label("", 12, color=PALETTE["muted"])
        self.flight_lb.setWordWrap(True)
        ml.addWidget(self.flight_lb, 3, 0, 1, 4)
        g.addWidget(mw, 2, 0, 1, 2)
        g.setRowStretch(3, 1)
        return w

    def _tab_nav(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        nb = Card("Position MCU · 100 Hz", "Navigační řešení")
        nl = QtWidgets.QGridLayout(nb.body)
        self.nav_vals = {}
        for i, (k, n) in enumerate([("alt", "Výška KF [m]"), ("baro_alt", "Baro výška [m]"), ("vz", "vz [m/s]"),
                                    ("sig_alt", "σ výšky [m]"), ("mach", "Mach"), ("tilt", "Náklon [°]"),
                                    ("acc_axial", "Osové zrychlení [g]"), ("acc_mag", "|a| [g]"), ("px", "E [m]"),
                                    ("py", "N [m]"), ("max_alt", "Max. výška [m]"), ("mode", "Režim")]):
            nl.addWidget(label(n, 11, color=PALETTE["muted"]), (i // 4) * 2, i % 4)
            val = label("—", 22, True)
            self.nav_vals[k] = val
            nl.addWidget(val, (i // 4) * 2 + 1, i % 4)
        g.addWidget(nb, 0, 0)
        fb = Card("Navigace", "Příznaky")
        fl = QtWidgets.QGridLayout(fb.body)
        self.nav_chips = {}
        for i, n in enumerate(NAV_FLAGS):
            c = Chip(n)
            self.nav_chips[n] = c
            fl.addWidget(c, i // 4, i % 4)
        g.addWidget(fb, 1, 0)
        self.att = AttitudeView()
        ab = Card("Orientace", "Náklon a azimut špičky")
        al = QtWidgets.QVBoxLayout(ab.body)
        al.addWidget(self.att)
        self.euler_lb = label("", 12)
        al.addWidget(self.euler_lb)
        g.addWidget(ab, 0, 1, 2, 1)
        self.ch_nav_alt = StripChart("Výška", [("KF", PALETTE["info"]), ("baro", PALETTE["ok"])], 30, "m")
        self.ch_nav_acc = StripChart("Zrychlení", [("osové", PALETTE["accent"]), ("|a|", PALETTE["muted"])], 30, "g")
        g.addWidget(self.ch_nav_alt, 2, 0)
        g.addWidget(self.ch_nav_acc, 2, 1)
        g.setRowStretch(2, 1)
        return w

    def _tab_sensors(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        sb = Card("Position MCU", "Senzory a GPS")
        sl = QtWidgets.QVBoxLayout(sb.body)
        self.sens_chips = {}
        row = QtWidgets.QHBoxLayout()
        for n in SENS_FLAGS:
            c = Chip(n)
            self.sens_chips[n] = c
            row.addWidget(c)
        sl.addLayout(row)
        self.pos_lb = label("", 13)
        self.pos_lb.setWordWrap(True)
        sl.addWidget(self.pos_lb)
        g.addWidget(sb, 0, 0)
        rb = Card("Raw stream · osy desky", "Surová data")
        rl = QtWidgets.QVBoxLayout(rb.body)
        hl = QtWidgets.QHBoxLayout()
        hl.addWidget(self._btn("RAW zap", lambda: self.send("RAW 1")))
        hl.addWidget(self._btn("RAW vyp", lambda: self.send("RAW 0")))
        hl.addWidget(self._btn("GPS překonfigurovat", lambda: self.send("GPSRECFG")))
        rl.addLayout(hl)
        self.raw_lb = label("RAW vypnuto", 13)
        self.raw_lb.setWordWrap(True)
        rl.addWidget(self.raw_lb)
        rl.addWidget(label("Tip: postav raketu špičkou nahoru – osa s +1 g je osa špičky (param nose_axis, 0 = auto).",
                           11, color=PALETTE["muted"]))
        g.addWidget(rb, 0, 1)
        self.ch_raw = StripChart("Akcelerometr", [("ax", "#f85149"), ("ay", "#2ea043"), ("az", "#388bfd")], 20, "g")
        self.ch_gyro = StripChart("Gyroskop", [("gx", "#f85149"), ("gy", "#2ea043"), ("gz", "#388bfd")], 20, "°/s")
        g.addWidget(self.ch_raw, 1, 0)
        g.addWidget(self.ch_gyro, 1, 1)
        g.setRowStretch(1, 1)
        return w

    def _tab_params(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Výška hlavního padáku:", 14, True))
        self.main_alt_spin = QtWidgets.QSpinBox()
        self.main_alt_spin.setRange(100, 1000)
        self.main_alt_spin.setSingleStep(25)
        self.main_alt_spin.setSuffix(" m")
        top.addWidget(self.main_alt_spin)
        top.addWidget(self._btn("Nastavit a uložit", self.set_main_alt, "go"))
        top.addStretch(1)
        top.addWidget(self._btn("Načíst z rakety", lambda: self.send("PARAM LIST")))
        top.addWidget(self._btn("Zapsat změny", self.write_params))
        top.addWidget(self._btn("Uložit do flash", lambda: self.send("PARAM SAVE")))
        top.addWidget(self._btn("Výchozí hodnoty", self.defaults))
        v.addLayout(top)
        self.param_table = QtWidgets.QTableWidget(0, 6)
        self.param_table.setHorizontalHeaderLabels(["Parametr", "Hodnota", "Min", "Max", "Výchozí", "Popis"])
        self.param_table.horizontalHeader().setStretchLastSection(True)
        self.param_table.verticalHeader().setVisible(False)
        self.param_table.setColumnWidth(0, 170)
        v.addWidget(self.param_table, 1)
        return w

    def _tab_tests(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        info = label("Testy jsou povoleny jen v režimu DIAG (z IDLE). Pyro test: vyjmi pyro náplně, připoj testovací "
                     "žárovku/odpor. Kanál se odpálí jen s vytaženým RBF.", 12, color=PALETTE["warn"])
        info.setWordWrap(True)
        g.addWidget(info, 0, 0, 1, 4)
        g.addWidget(self._btn("DIAG ZAP", lambda: self.send("DIAG 1"), "go"), 1, 0)
        g.addWidget(self._btn("DIAG VYP", lambda: self.send("DIAG 0")), 1, 1)
        g.addWidget(self._btn("Pyro lišta ZAP", lambda: self.send("ARM 1")), 2, 0)
        g.addWidget(self._btn("Pyro lišta VYP", lambda: self.send("ARM 0")), 2, 1)
        for ch, n in ((1, "PYRO1 drogue"), (2, "PYRO2 main"), (3, "PYRO3 záloha")):
            g.addWidget(self._btn(f"ODPÁLIT {n}", lambda _=False, ch=ch: self.fire(ch), "abort"), 3, ch - 1)
        g.addWidget(self._btn("Bzučák 1 s", lambda: self.send("BUZZ 1000")), 4, 0)
        g.addWidget(self._btn("Kamera nahrávat", lambda: self.send("CAM 1")), 4, 1)
        g.addWidget(self._btn("Kamera stop", lambda: self.send("CAM 0")), 4, 2)
        g.addWidget(self._btn("FORCE_BAT zap (baterie)", lambda: self.send("FORCEBAT 1")), 5, 0)
        g.addWidget(self._btn("FORCE_BAT vyp", lambda: self.send("FORCEBAT 0")), 5, 1)
        g.addWidget(self._btn("Napájení serv zap", lambda: self.send("SERVOPWR 1")), 6, 0)
        g.addWidget(self._btn("Napájení serv vyp", lambda: self.send("SERVOPWR 0")), 6, 1)
        g.addWidget(self._btn("Ping rampy", lambda: self.send("RAMPA PING")), 6, 2)
        g.addWidget(self._btn("Lokátor bzučák zap", lambda: self.send("LOCATE 1")), 7, 0)
        g.addWidget(self._btn("Lokátor bzučák vyp", lambda: self.send("LOCATE 0")), 7, 1)
        self.test_lb = label("", 13)
        self.test_lb.setWordWrap(True)
        g.addWidget(self.test_lb, 8, 0, 1, 4)
        g.setRowStretch(9, 1)
        return w

    def _tab_log(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        h.addWidget(self._btn("Info", lambda: self.send("LOG INFO")))
        h.addWidget(self._btn("Stáhnout log → CSV", self.dump_log, "go"))
        h.addWidget(self._btn("Smazat log", self.erase_log))
        h.addStretch(1)
        self.log_info = label("", 12)
        h.addWidget(self.log_info)
        v.addLayout(h)
        self.events = QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        v.addWidget(label("Události (z logu i živě):", 12, color=PALETTE["muted"]))
        v.addWidget(self.events, 1)
        return w

    def _tab_console(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(3000)
        v.addWidget(self.console, 1)
        h = QtWidgets.QHBoxLayout()
        self.show_stream = QtWidgets.QCheckBox("zobrazit i proud $S/$N/$P")
        self.cmd_le = QtWidgets.QLineEdit()
        self.cmd_le.setPlaceholderText("příkaz, např. PARAM SET main_alt_m 350   |   HELLO   |   FLIGHT")
        self.cmd_le.returnPressed.connect(lambda: (self.send(self.cmd_le.text()), self.cmd_le.clear()))
        h.addWidget(self.cmd_le, 1)
        h.addWidget(self.show_stream)
        v.addLayout(h)
        return w

    # ------------------------------------------------------------------ actions
    def connect_port(self, port: str):
        if not port:
            return
        self.stop_reader()
        self.reader = SerialReader(port)
        self.reader.line.connect(self.on_line)
        self.reader.status.connect(lambda s: self.conn_lb.setText(s))
        self.reader.start()
        QtCore.QTimer.singleShot(800, lambda: (self.send("HELLO"), self.send("PARAM LIST")))

    def start_demo(self):
        self.stop_reader()
        self.reader = DemoMaster()
        self.reader.line.connect(self.on_line)
        self.conn_lb.setText("DEMO")
        QtCore.QTimer.singleShot(200, lambda: (self.send("HELLO"), self.send("PARAM LIST")))

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None

    def send(self, text: str):
        if not text:
            return
        self.console.appendPlainText("> " + text)
        if self.reader is None:
            self.console.appendPlainText("! nepřipojeno")
            return
        self.reader.send(text)

    def countdown(self):
        if QtWidgets.QMessageBox.question(self, "Odpočet", f"Spustit odpočet {self.cd_spin.value()} s z USB?") == \
                QtWidgets.QMessageBox.StandardButton.Yes:
            self.send(f"COUNTDOWN {self.cd_spin.value()}")

    def fire(self, ch: int):
        txt, ok = QtWidgets.QInputDialog.getText(self, "Pyro test", f"Odpálit kanál {ch}? Napiš FIRE pro potvrzení:")
        if ok and txt.strip().upper() == "FIRE":
            self.send(f"FIRE {ch} CONFIRM")

    def set_main_alt(self):
        self.send(f"PARAM SET main_alt_m {self.main_alt_spin.value()}")
        self.send("PARAM SAVE")

    def write_params(self):
        n = 0
        for r in range(self.param_table.rowCount()):
            name = self.param_table.item(r, 0).text()
            val = self.param_table.item(r, 1).text().strip()
            if name in self.params and val != str(self.params[name]):
                self.send(f"PARAM SET {name} {val}")
                n += 1
        self.console.appendPlainText(f"# zapsáno {n} změn – nezapomeň 'Uložit do flash'")

    def defaults(self):
        if QtWidgets.QMessageBox.question(self, "Výchozí", "Načíst výchozí hodnoty všech parametrů?") == \
                QtWidgets.QMessageBox.StandardButton.Yes:
            self.send("PARAM DEFAULTS")
            self.send("PARAM LIST")

    def dump_log(self):
        self.log_lines = []
        self.dumping = True
        self.send("LOG DUMP")

    def erase_log(self):
        if QtWidgets.QMessageBox.question(self, "Smazat log", "Opravdu smazat celý letový log z flash?") == \
                QtWidgets.QMessageBox.StandardButton.Yes:
            self.send("LOG ERASE CONFIRM")

    def save_log(self):
        rows, events = [], []
        for hx in self.log_lines:
            try:
                b = bytes.fromhex(hx)
            except ValueError:
                continue
            if len(b) != 64 or crc16(b[:62]) != struct.unpack("<H", b[62:])[0]:
                continue
            if b[0] == 0xA7:
                rows.append(LOG_DATA.unpack(b))
            elif b[0] == 0xA8:
                e = LOG_EVENT.unpack(b)
                events.append((e[3], state_name(e[1]), e[4].split(b"\0")[0].decode(errors="replace")))
        path = os.environ.get("ZENITH_LOG_OUT") or QtWidgets.QFileDialog.getSaveFileName(
            self, "Uložit log", time.strftime("zenith_log_%Y%m%d_%H%M%S.csv"), "CSV (*.csv)")[0]
        if not path:
            return
        with open(path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t_ms", "state", "flags", "pyro", "alt", "vz", "baro_alt", "acc_axial", "acc_mag", "mach",
                         "tilt", "lat", "lon", "vbat_mv", "curr_ma", "nav_flags", "px", "py", "max_alt", "rampa_state",
                         "err", "sats", "nav_mode", "vh_dms"])
            for r in rows:
                wr.writerow([r[4], state_name(r[1]), r[2], r[3], *[f"{x:.3f}" for x in r[5:12]], r[12] * 1e-7,
                             r[13] * 1e-7, *r[14:25]])
        with open(os.path.splitext(path)[0] + "_events.txt", "w") as f:
            for t, st, txt in events:
                f.write(f"{t / 1000:10.3f}  {st:10s}  {txt}\n")
        for t, st, txt in events:
            self.events.appendPlainText(f"[log] {t / 1000:9.3f} s  {st:9s} {txt}")
        self.log_info.setText(f"uloženo {len(rows)} záznamů a {len(events)} událostí → {path}")

    # ------------------------------------------------------------------ parsing
    def on_line(self, s: str):
        p = s.split(",")
        tag = p[0]
        if tag == "$S" and len(p) > len(S_FIELDS):
            self.s = dict(zip(S_FIELDS, p[1:]))
            self.s_rx = time.time()
        elif tag == "$N" and len(p) > len(N_FIELDS):
            self.nav = dict(zip(N_FIELDS, p[1:]))
            t = time.time() - self.t0
            self.ch_nav_alt.add(t, KF=to_float(self.nav["alt"]), baro=to_float(self.nav["baro_alt"]))
            self.ch_nav_acc.add(t, **{"osové": to_float(self.nav["acc_axial"]), "|a|": to_float(self.nav["acc_mag"])})
        elif tag == "$P" and len(p) > len(P_FIELDS) - 2:
            self.pos = dict(zip(P_FIELDS, p[1:]))
        elif tag == "$R" and len(p) > len(R_FIELDS):
            self.raw = dict(zip(R_FIELDS, p[1:]))
            t = time.time() - self.t0
            self.ch_raw.add(t, **{k: to_float(self.raw[k]) for k in ("ax", "ay", "az")})
            self.ch_gyro.add(t, **{k: to_float(self.raw[k]) for k in ("gx", "gy", "gz")})
        elif tag == "$F" and len(p) > len(F_FIELDS):
            self.fl = dict(zip(F_FIELDS, p[1:]))
        elif tag == "$PRM" and len(p) >= 8:
            self.on_param(p)
        elif tag == "$EV":
            self.events.appendPlainText(f"{to_int(p[1]) / 1000:9.3f} s  {','.join(p[2:])}")
        elif tag == "$LOGBEGIN":
            self.log_lines = []
            self.log_info.setText(f"stahuji {p[1]} záznamů…")
        elif tag == "$LOG" and len(p) == 2:
            self.log_lines.append(p[1])
            if len(self.log_lines) % 2000 == 0:
                self.log_info.setText(f"staženo {len(self.log_lines)}…")
        elif tag == "$LOGEND":
            self.dumping = False
            self.save_log()
        elif tag == "$LOGINFO" and len(p) >= 6:
            self.log_info.setText(f"záznamů {p[1]} / {p[2]} · zahozeno {p[3]} · mazání {p[4]} · JEDEC {to_int(p[5]):06X}")
        if tag in ("$S", "$N", "$P", "$R", "$F", "$LOG"):
            if self.show_stream.isChecked() and tag != "$LOG":
                self.console.appendPlainText(s)
        else:
            self.console.appendPlainText(s)
            if s.startswith(("#OK", "#ERR", "#RC")):
                self.test_lb.setText(s)

    def on_param(self, p):
        idx, name, val, lo, hi, df = to_int(p[1]), p[2], p[3], p[4], p[5], p[6]
        desc = ",".join(p[7:])
        self.params[name] = val
        if self.param_table.rowCount() <= idx:
            self.param_table.setRowCount(idx + 1)
        for col, txt in enumerate([name, val, lo, hi, df, desc]):
            it = QtWidgets.QTableWidgetItem(txt)
            if col != 1:
                it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.param_table.setItem(idx, col, it)
        if name == "main_alt_m" and not self.main_alt_spin.hasFocus():
            self.main_alt_spin.setValue(int(float(val)))

    # ------------------------------------------------------------------ display
    def refresh(self):
        if not self.s:
            return
        s = self.s
        age = time.time() - self.s_rx
        st = state_name(to_int(s["state"]))
        self.state_chip.set(("hot" if st == "ABORT" else "warn") if age < 1 else False,
                            f"{st}" if age < 1 else "SPOJENÍ ZTRACENO")
        err = to_int(s["err"])
        self.go_chip.set(err == 0, "GO" if err == 0 else "NO-GO: " + err_name(err))
        flags, health, pyro = to_int(s["flags"]), to_int(s["health"]), to_int(s["pyro"])
        vb = to_int(s["vbat_mv"]) / 1000
        vmin = to_float(self.params.get("vbat_min_v", "7.2"), 7.2)
        tilt = to_float(self.nav.get("tilt", "nan"))
        maxt = to_float(self.params.get("max_tilt_deg", "20"), 20)
        rs, rf = to_int(s["rampa_state"]), to_int(s["rampa_flags"])
        c = self.checks
        c["pos"].set(bool(health & 1))
        c["nav"].set(bool(health & 2))
        c["hw"].set(bool(health & 32))
        c["bat"].set(vb >= vmin)
        for i in range(3):
            c[f"c{i + 1}"].set(bool(pyro & (1 << i)) if i < 2 or to_float(self.params.get("req_cont3", "0")) > 0.5
                               else (True if pyro & 4 else "warn"))
        c["rbf"].set(to_int(s["rbf"]) == 1)
        c["tilt"].set(tilt <= maxt if math.isfinite(tilt) else None)
        c["rampa"].set(rs != 0)
        c["rampa_ready"].set(bool(rf & 0x80))
        c["flash"].set(to_int(s["flash"]) == 1)
        c["lora"].set(to_int(s["lora"]) == 1)
        c["gps"].set(True if health & 8 else "warn")
        self.err_lb.setText("" if err == 0 else ("Přerušeno: " if st == "ABORT" else "NO-GO: ") + err_name(err))
        cont = [to_int(s[f"ads{i}"]) / DIV / 1000 for i in range(3)]
        rail = to_int(s["ads3"]) / DIV / 1000
        self.pow_lb.setText(
            f"<b>Baterie</b> {vb:.2f} V, {to_int(s['curr_ma'])} mA · pogo 5 V {'ANO' if to_int(s['umb']) else 'ne'} "
            f"({to_int(s['umb_mv']) * 2 / 1000:.2f} V) · FORCE_BAT {'ZAP' if flags & 0x20 else 'vyp'}<br>"
            f"Breakwire 1 {'celý' if to_int(s['bw1']) else 'PŘERUŠEN'} · Breakwire 2 {'celý' if to_int(s['bw2']) else 'PŘERUŠEN'} · "
            f"RBF {'vytažen (armable)' if to_int(s['rbf']) else 'ZASUNUT (safe)'}")
        self.pyro_lb.setText(
            "<b>Pyro</b> lišta {:s} ({:.2f} V)<br>".format("ZAPNUTA" if pyro & 0x40 else "vypnuta", rail) +
            "<br>".join(f"PYRO{i + 1} {n}: {cont[i]:.2f} V – {'OK' if pyro & (1 << i) else 'BEZ KONTINUITY'}"
                        f"{' · VYPÁLEN' if pyro & (8 << i) else ''}"
                        for i, n in enumerate(("drogue", "main", "záloha (oba padáky)"))))
        self.rampa_lb.setText(
            f"Stav <b>{RAMPA_STATES[rs] if rs < len(RAMPA_STATES) else rs}</b> · baterie {to_int(s['rampa_vbat']) / 1000:.2f} V · "
            f"klíč {'ZAP' if rf & 1 else 'vyp'} · rozbušky {''.join(str(i + 1) if rf & (2 << i) else '-' for i in range(3))} · "
            f"LoRa rampy {'vyp' if rf & 0x20 else 'ZAP'} · {'PŘIPRAVENA' if rf & 0x80 else 'nepřipravena'}"
            + ("<br>chyby: " + ", ".join(bits(to_int(s['rampa_faults']), RAMPA_FAULTS)) if to_int(s['rampa_faults']) else ""))
        trem = to_int(s["t_rem"], -1)
        f = self.fl
        self.flight_lb.setText(
            (f"T-{trem / 1000:.1f} s · " if trem >= 0 else "") +
            (f"poslední let: max {f.get('max_alt', '0')} m, vz {f.get('max_vz', '0')} m/s, Mach {f.get('max_mach', '0')}, "
             f"{f.get('max_g', '0')} g · drogue {WHY[to_int(f.get('why_d'))]} · main {WHY[to_int(f.get('why_m'))]} · "
             f"PYRO3 {WHY[to_int(f.get('why_p3'))]}" if f else "") +
            f" · log {s['log_head']} záznamů ({s['log_dropped']} zahozeno)")
        # navigation
        n = self.nav
        if n:
            for k, lb in self.nav_vals.items():
                if k == "mode":
                    lb.setText(NAV_MODES[to_int(n["mode"])] if to_int(n["mode"]) < 4 else n["mode"])
                else:
                    lb.setText(f"{to_float(n[k]):.2f}")
            nf = to_int(n["flags"])
            for i, nm in enumerate(NAV_FLAGS):
                on = bool(nf & (1 << i))
                bad_if_off = nm in ("VALID", "BARO_OK", "ATT_OK", "PAD_REF", "IMU_OK")
                self.nav_chips[nm].set(True if on else (False if bad_if_off else None))
            self.att.set(tuple(to_float(n[k]) for k in ("qw", "qx", "qy", "qz")), to_int(self.pos.get("nose", "1"), 1))
        p = self.pos
        if p:
            self.euler_lb.setText(f"roll {to_float(p['roll']):.1f}° · pitch {to_float(p['pitch']):.1f}° · "
                                  f"heading osy X {to_float(p['yaw']):.0f}° · osa špičky {AXES[min(6, to_int(p['nose']))]}")
            sf = to_int(p["sensors"])
            for i, nm in enumerate(SENS_FLAGS):
                self.sens_chips[nm].set(bool(sf & (1 << i)))
            self.pos_lb.setText(
                f"FW {p['fw']} · IMU {p['imu_hz']} Hz · ztracené vzorky {p['imu_lost']} · špička {p['peak_g']} g · "
                f"restart {'watchdog' if to_int(p['reset']) else 'power-on'}<br>"
                f"Tlak {to_float(p['pressure']):.1f} Pa · teplota {to_float(p['temp']):.1f} °C · bias gyra "
                f"{p['gb0']}/{p['gb1']}/{p['gb2']} °/s<br>"
                f"GPS: fix {p['fix']} · {p['sats']} sat · hAcc {p['hacc']} m · {to_float(p['lat']):.7f}, {to_float(p['lon']):.7f} · "
                f"výška MSL {p['galt']} m<br>rampa (GPS počátek): {p.get('pad_lat', '?')}, {p.get('pad_lon', '?')}")
        r = self.raw
        if r:
            self.raw_lb.setText(f"a = {r['ax']}, {r['ay']}, {r['az']} g<br>ω = {r['gx']}, {r['gy']}, {r['gz']} °/s<br>"
                                f"m = {r['mx']}, {r['my']}, {r['mz']} µT · podíl high-g {r['hgw']}")

    def closeEvent(self, ev):
        self.stop_reader()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Zenith Diagnostika")
    vs.setup_app(app, "VoltinoLabs.Zenith.Diagnostika")
    w = DiagWindow(a.port, a.demo)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
