#!/usr/bin/env python3
"""
Zenith — DIAGNOSTIC CONSOLE (flight computer on a USB cable)

Plug the Zenith's MASTER MCU (Raspberry Pi Pico 2, U15) into the laptop and
this window shows the state of the whole vehicle, laid out like the Daidalos
diagnostic console (Daidalos_Landing_Rocket_Code, Ariadna_V2/Diagnostics):

  * header       port (or "Simulace"), connect, board status chips, ABORT
  * second bar   state / firmware / parameters / errors chips and the mission
                 buttons (DIAG mode, pad reference, gyro calibration,
                 PREPARE, countdown, SAFE, record every line to CSV)
  * Přehled      Master / Position / rampa cards, attitude (3-D rocket, tilt
                 bull's-eye, numbers), flight and parachutes, errors and
                 warnings (the Go/No-Go list)
  * Poloha a senzory, Grafy, Surová data IMU
  * Aktuátory a pyro   pyro rail and channels (hold to fire, DIAG mode),
                 power switches, camera, buzzers, rampa ping, console
  * Parametry a časování   grouped, searchable table, edit, send, save
  * Letový záznam      download the flash log to CSV, events, plots

    pip install PyQt6 pyserial
    python zenith_diag.py                # pick the port in the window
    python zenith_diag.py --port COM5
    python zenith_diag.py --demo         # no hardware: a simulated Master

The protocol is described in docs/USB_PROTOCOL.md.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import struct
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import voltino_style as vs  # noqa: E402
from voltino_style import C, Chip, button, label  # noqa: E402
import zenith_widgets as zw  # noqa: E402
from zenith_widgets import RocketView, TiltBubble, StripChart, PadMap, attitude_summary  # noqa: E402
from zenith_common import (RAMPA_STATES, RAMPA_FAULTS, NAV_FLAGS, NAV_MODES, SENS_FLAGS, WHY,  # noqa: E402
                           SerialReader, haversine_m, list_ports, state_name, err_name, bits, to_float, to_int)

APP_NAME = "Zenith · Diagnostika"

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
DIV = 47.0 / 147.0                      # pyro sense divider in front of the ADS1115
AXES = ["auto", "+X", "-X", "+Y", "-Y", "+Z", "-Z"]

STATE_CZ = {"BOOT": "start", "IDLE": "na rampě, bezpečno", "READY": "PŘIPRAVENO ke startu", "COUNTDOWN": "ODPOČET",
            "BOOST": "motor hoří", "COAST": "setrvačný let", "DROGUE": "brzdicí padák", "MAIN": "hlavní padák",
            "LANDED": "přistáno", "ABORT": "PŘERUŠENO", "DIAG": "DIAG režim (testy)"}
WHY_CZ = {"-": "—", "APO_KF": "apogeum (Kalmanův filtr)", "APO_BARO": "apogeum (barometr)",
          "APO_VZ": "apogeum (svislá rychlost)", "APO_TIMER_NAVLOST": "časovač – navigace ztracena",
          "APO_TIMER_MAX": "pojistka apo_max_s", "MAIN_ALT": "výška main_alt_m", "MAIN_TIMER": "časovač main_backup_s",
          "MAIN_LOW_APOGEE": "nízké apogeum (hned)", "P3_DROGUE_FAIL": "selhal drogue", "P3_MAIN_FAIL": "selhal main",
          "P3_BLIND": "bez navigace (p3_blind_s)"}
EVENT_BITS = [(0, "Start"), (1, "Vyhoření"), (2, "Apogeum"), (3, "Drogue"), (4, "Main"), (5, "PYRO3"),
              (6, "Přistáno")]
NAV_BITS = [(i, n) for i, n in enumerate(NAV_FLAGS)]
SENS_BITS = [(i, n) for i, n in enumerate(SENS_FLAGS)]
TF_ARMED, TF_RBF, TF_BW1, TF_BW2, TF_UMB, TF_ON_BATT, TF_LOGGING, TF_GO = (1 << i for i in range(8))
HL_POS, HL_NAV, HL_BARO, HL_GPS, HL_FLASH, HL_HW, HL_CAM, HL_LOCATE = (1 << i for i in range(8))

# ---------------------------------------------------------------- parameters
# What every parameter does (Czech, for the editor). Keys from firmware/common/zenith_params.h;
# anything not listed shows the firmware's own description.
PARAM_HELP = {
    "main_alt_m": "Výška nad rampou, ve které se při sestupu odpálí hlavní padák (PYRO2).",
    "drogue_fail_vz": "Selhání brzdicího padáku: klesá-li raketa po drogue_check_s rychleji, odpálí se PYRO3.",
    "main_fail_vz": "Selhání hlavního padáku: klesá-li raketa po main_check_s rychleji, odpálí se PYRO3.",
    "drogue_check_s": "Za kolik sekund po PYRO1 se začne hlídat rychlost klesání.",
    "main_check_s": "Za kolik sekund po PYRO2 se začne hlídat rychlost klesání.",
    "apo_lockout_s": "Nejdřív tolik sekund po startu smí být uznáno apogeum.",
    "apo_mach_max": "Nad tímto Machem (odhad navigace) se apogeum neuzná.",
    "apo_backup_s": "Když je navigace neplatná: drogue časovačem v T+ tolik sekund.",
    "apo_max_s": "Absolutní pojistka: drogue nejpozději v T+ tolik sekund.",
    "main_backup_s": "Když je navigace neplatná: hlavní padák tolik sekund po drogue.",
    "main_min_delay_s": "Nejmenší odstup PYRO2 po PYRO1.",
    "p3_enable": "1 = záložní nálož PYRO3 (drogue i main zároveň) je povolená.",
    "p3_blind_s": "Bez navigace: PYRO3 tolik sekund po PYRO2 (0 = nikdy).",
    "pyro_pulse_ms": "Délka proudového pulzu pyro kanálu. Pozor na 2W odpory 2× 4,7 Ω.",
    "cont_min_mv": "Práh kontinuity pyro kanálu na ADS1115 (napětí na děliči).",
    "req_cont3": "1 = start vyžaduje i kontinuitu PYRO3.",
    "liftoff_g": "Detekce startu: osové zrychlení nad touto hodnotou po dobu 60 ms.",
    "burn_max_s": "Nejdelší možné hoření motoru; potom se přejde do COAST.",
    "cd_force_bat_s": "V T− tolik sekund se napájení přepne na baterii rakety (FORCE_BAT).",
    "cd_arm_s": "V T− tolik sekund se zapne pyro lišta (PYRO_SW_ARM).",
    "ign_wait_s": "Po T-0 se tolik sekund čeká na start, pak HANGFIRE.",
    "rampa_ch": "Kanál rozbušky motoru na rampě (1–3).",
    "rampa_fire_ms": "Délka zážehového pulzu rampy.",
    "vbat_min_v": "Nejnižší napětí baterie rakety (2S), při kterém lze startovat.",
    "max_tilt_deg": "Největší náklon rakety na rampě, při kterém lze startovat.",
    "ext_ign": "1 = start bez rampy (externí zážeh, detekce akcelerometrem).",
    "bw_mode": "Breakwire: 0 nepoužit, 1 stačí kterýkoli, 2 oba.",
    "ready_timeout_min": "READY bez odpočtu se po tolika minutách vrátí do IDLE.",
    "nose_axis": "Osa špičky na IMU: 0 automaticky (osa s +1 g na rampě), 1 +X, 2 −X, 3 +Y, 4 −Y, 5 +Z, 6 −Z.",
    "decl_deg": "Magnetická deklinace místa startu.",
    "landed_still_s": "Přistání: raketa v klidu po tuto dobu.",
    "locate_after_s": "Bzučák pro dohledání se zapne tolik sekund po přistání (0 = hned).",
    "tlm_hz": "Frekvence LoRa telemetrie.",
    "cam_mode": "Kamera: 1 = zapnout a nahrávat od READY.",
}
PARAM_GROUPS: list[tuple[str, str, tuple[str, ...]]] = [
    ("Padáky a apogeum", "kdy drogue, kdy main, záloha PYRO3, pojistky časovačem",
     ("main_", "apo_", "drogue_", "p3_")),
    ("Pyro", "pulz, práh kontinuity",
     ("pyro_", "cont_min_mv", "req_cont3")),
    ("Start a odpočet", "rampa, detekce startu, podmínky Go/No-Go",
     ("liftoff_g", "burn_max_s", "cd_", "ign_wait_s", "rampa_", "vbat_min_v", "max_tilt_deg", "ext_ign", "bw_mode",
      "ready_timeout_min")),
    ("Senzory a navigace", "osa špičky, deklinace",
     ("nose_axis", "decl_deg")),
    ("Po přistání, telemetrie, kamera", "dohledání, LoRa, kamera",
     ("landed_", "locate_", "tlm_", "cam_")),
]
PARAM_OTHER = ("Ostatní", "parametry bez skupiny", ())


def param_group(key: str) -> int:
    for i, (_n, _s, rules) in enumerate(PARAM_GROUPS):
        for r in rules:
            if key == r or (r.endswith(("_", ".")) and key.startswith(r)):
                return i
    return len(PARAM_GROUPS)


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
    """A simulated Master: same lines and replies as the firmware, no hardware."""
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    PARAMS = [("main_alt_m", 300, 100, 1000, "Vyska vypusteni hlavniho padaku (PYRO2) nad rampou [m]"),
              ("drogue_fail_vz", 35, 10, 150, "Selhani drogue [m/s]"), ("main_fail_vz", 15, 4, 60, "Selhani main [m/s]"),
              ("drogue_check_s", 4, 1, 20, "kontrola drogue [s]"), ("main_check_s", 4, 1, 20, "kontrola main [s]"),
              ("apo_lockout_s", 6, 1, 60, "lockout [s]"), ("apo_mach_max", 0.5, 0.2, 0.9, "mach"),
              ("apo_backup_s", 25, 3, 180, "backup [s]"), ("apo_max_s", 35, 3, 300, "max [s]"),
              ("main_backup_s", 100, 3, 900, "main backup [s]"), ("p3_enable", 1, 0, 1, "p3"),
              ("p3_blind_s", 8, 0, 900, "p3 blind [s]"), ("pyro_pulse_ms", 300, 20, 2000, "pulse [ms]"),
              ("liftoff_g", 2.5, 1.5, 10, "liftoff [g]"), ("burn_max_s", 6, 0.5, 30, "burn [s]"),
              ("cd_force_bat_s", 10, 3, 120, "bat [s]"), ("cd_arm_s", 5, 1, 60, "arm [s]"),
              ("ign_wait_s", 5, 1, 30, "ign [s]"), ("rampa_ch", 1, 1, 3, "ch"),
              ("rampa_fire_ms", 1000, 100, 5000, "fire [ms]"), ("vbat_min_v", 7.2, 6, 8.4, "vbat [V]"),
              ("req_cont3", 0, 0, 1, "c3"), ("max_tilt_deg", 20, 2, 60, "tilt [deg]"), ("nose_axis", 0, 0, 6, "nose"),
              ("tlm_hz", 5, 1, 10, "tlm [Hz]"), ("locate_after_s", 30, 0, 900, "locate [s]"),
              ("cam_mode", 1, 0, 1, "cam"), ("ext_ign", 0, 0, 1, "ext"), ("bw_mode", 1, 0, 2, "bw"),
              ("decl_deg", 5, -30, 30, "decl [deg]"), ("landed_still_s", 5, 2, 60, "landed [s]"),
              ("cont_min_mv", 300, 50, 3000, "cont [mV]"), ("main_min_delay_s", 1.5, 0.5, 10, "delay [s]"),
              ("ready_timeout_min", 30, 1, 240, "ready [min]")]

    def __init__(self):
        super().__init__()
        self.t0 = time.time()
        self.state, self.raw = 1, False
        self.vals = {p[0]: p[1] for p in self.PARAMS}
        self.armed = self.force_bat = self.cam = self.locate = False
        self.fired = 0
        self.cd_end = 0.0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.n = 0

    def stop(self):
        self.timer.stop()

    def _prm(self, i: int):
        n, d, lo, hi, desc = self.PARAMS[i]
        self.line.emit(f"$PRM,{i},{n},{self.vals[n]:g},{lo:g},{hi:g},{d:g},{desc}")

    def send(self, text: str):
        p = text.split()
        if not p:
            return
        c = p[0].upper()
        diag = self.state == 10
        if c == "HELLO":
            self.line.emit(f"$ID,Zenith Master,1.00,{len(self.PARAMS)}")
        elif c == "PARAM" and len(p) > 1 and p[1].upper() == "LIST":
            for i in range(len(self.PARAMS)):
                self._prm(i)
            self.line.emit("#OK PARAM LIST")
        elif c == "PARAM" and len(p) > 3 and p[1].upper() == "SET":
            idx = next((i for i, q in enumerate(self.PARAMS) if q[0] == p[2]), -1)
            if idx < 0:
                self.line.emit("#ERR unknown parameter")
                return
            self.vals[p[2]] = float(p[3])
            self.line.emit(f"#OK {p[2]}={float(p[3]):g}")
            self._prm(idx)
        elif c == "PARAM" and len(p) > 1 and p[1].upper() == "DEFAULTS":
            self.vals = {q[0]: q[1] for q in self.PARAMS}
            self.line.emit("#OK defaults loaded (PARAM SAVE to keep)")
        elif c == "PARAM":
            self.line.emit("#OK PARAM " + " ".join(p[1:]).upper())
        elif c == "DIAG":
            on = len(p) > 1 and p[1] == "1"
            if on and self.state in (1, 9):
                self.state = 10
                self.line.emit("#OK DIAG ON")
            elif not on and self.state == 10:
                self.state, self.armed, self.force_bat = 1, False, False
                self.line.emit("#OK DIAG OFF")
            else:
                self.line.emit("#ERR DIAG only from IDLE/ABORT")
        elif c == "RAW":
            self.raw = len(p) > 1 and p[1] == "1"
            self.line.emit("#OK RAW")
        elif c in ("ARM", "FORCEBAT", "SERVOPWR"):
            if not diag:
                self.line.emit("#ERR DIAG mode required")
                return
            on = len(p) > 1 and p[1] == "1"
            if c == "ARM":
                self.armed = on
            elif c == "FORCEBAT":
                self.force_bat = on
            self.line.emit(f"#OK {c}")
        elif c == "FIRE":
            if not diag:
                self.line.emit("#ERR DIAG mode required")
            elif len(p) < 3 or p[2] != "CONFIRM":
                self.line.emit("#ERR FIRE <1..3> CONFIRM")
            else:
                ch = to_int(p[1])
                self.fired |= 1 << (ch - 1)
                self.line.emit(f"$EV,{int((time.time() - self.t0) * 1000)},DIAG FIRE CH{ch}")
                self.line.emit("#OK FIRE")
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
        elif c == "CAM":
            self.cam = len(p) > 1 and p[1] == "1"
            self.line.emit("#RC CAM 1")
        elif c == "LOCATE":
            self.locate = len(p) > 1 and p[1] == "1"
            self.line.emit("#RC LOCATE 1")
        elif c == "COUNTDOWN":
            if self.state != 2:
                self.line.emit("#RC COUNTDOWN 3")
            else:
                self.state, self.cd_end = 3, time.time() + to_float(p[1] if len(p) > 1 else "30", 30)
                self.line.emit("#RC COUNTDOWN 1")
        elif c in ("PREPARE", "SAFE", "ABORT", "PADREF", "REBOOT"):
            self.state = {"PREPARE": 2, "SAFE": 1, "ABORT": 9}.get(c, self.state)
            self.line.emit(f"#RC {c} 1")
        else:
            self.line.emit(f"#OK {text}")

    def tick(self):
        self.n += 1
        t = time.time() - self.t0
        roll = 4 * math.sin(t * 0.7)
        if self.state == 3 and time.time() >= self.cd_end:
            self.state = 9                        # the demo does not fly: rampa "no ignition"
            self.line.emit(f"$EV,{int(t * 1000)},ABORT demo: zadna rampa")
        if self.n % 2 == 0:
            a = math.radians(45 + roll / 2)
            q = (math.cos(a), 0.05 * math.sin(t * 0.3), -math.sin(a), 0.0)
            px, py = 0.6 * math.sin(t * 0.21), 0.4 * math.cos(t * 0.17)
            self.line.emit("$N," + ",".join(f"{v:.4f}" for v in (
                0.1 * math.sin(t), 0.02 * math.cos(t), px, py, 0, 0, 0.998, 1.001, 0.0, 0.12 * math.sin(t * 2), 0.6,
                abs(roll) + 3.0, *q, 0.3)) + ",63511,1")
        if self.n % 2 == 1:
            flags = 0xD6 | (1 if self.armed else 0) | (0x20 if self.force_bat else 0)
            health = 0x3F | (0x40 if self.cam else 0) | (0x80 if self.locate else 0)
            pyro = 0x07 | (self.fired << 3) | (0x40 if self.armed else 0)
            trem = int((self.cd_end - time.time()) * 1000) if self.state == 3 else -1
            self.line.emit(f"$S,{self.state},0,{flags},{health},{int(t * 1000)},{int(8040 - 20 * math.sin(t / 9))},"
                           f"{int(180 + 20 * math.sin(t)) + (250 if self.cam else 0)},{pyro},2480,2470,2465,"
                           f"{2600 if self.armed else 2400},{2 if self.state in (2, 3) else 1},"
                           f"{0xCF if self.state in (2, 3) else 0x6F},0,7950,{trem},1,1,1,1,2510,1,1,"
                           f"{int(self.vals['main_alt_m'])},1234,0,0,0")
        if self.n % 4 == 0:
            self.line.emit(f"$P,49.7985411,16.6912772,{412.3 + random.random():.1f},13,3,1.8,98134.5,23.4,"
                           f"{roll:.1f},{-86.0 + roll / 3:.1f},123.0,0.120,-0.310,0.050,1.2,960,63,1,0,1.00,0,"
                           f"49.7985400,16.6912700")
        if self.raw:
            self.line.emit(f"$R,{0.998 + random.gauss(0, 0.002):.4f},{0.01 + random.gauss(0, 0.002):.4f},"
                           f"{-0.02 + random.gauss(0, 0.002):.4f},{0.12 + random.gauss(0, 0.05):.3f},"
                           f"{-0.31 + random.gauss(0, 0.05):.3f},{0.05 + random.gauss(0, 0.05):.3f},"
                           f"{21.0 + random.gauss(0, 0.3):.2f},{-3.5 + random.gauss(0, 0.3):.2f},"
                           f"{-44.2 + random.gauss(0, 0.3):.2f},0.00")
        if self.n % 20 == 0:
            self.line.emit(f"$F,0,0,0,{self.fired},0.0,0.0,0.00,0.0,{bin(self.fired).count('1')}")


# =====================================================================
#  small view helpers (as in the Daidalos console)
# =====================================================================
GOOD, WARN, BAD, DIM = C["positive"], C["accent"], C["danger"], C["subtle"]


class KV(QtWidgets.QWidget):
    """A two-column grid of name : value labels."""

    def __init__(self, names: list[str]):
        super().__init__()
        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(14)
        g.setVerticalSpacing(3)
        self.v: dict[str, QtWidgets.QLabel] = {}
        for i, n in enumerate(names):
            g.addWidget(label(n, "muted"), i, 0)
            lab = label("—", "value")
            lab.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            g.addWidget(lab, i, 1)
            self.v[n] = lab
        g.setColumnStretch(1, 1)

    def set(self, name: str, text: str, color: str | None = None) -> None:
        lab = self.v[name]
        if lab.text() != text:
            lab.setText(text)
        style = f"color: {color};" if color else ""
        if lab.styleSheet() != style:
            lab.setStyleSheet(style)

    def clear(self) -> None:
        for n in self.v:
            self.set(n, "—", DIM)


class Flags(QtWidgets.QWidget):
    """A row of chips, one per bit."""

    def __init__(self, bits_: list[tuple[int, str]]):
        super().__init__()
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(5)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Fixed)
        self.bits = bits_
        self.chips = [Chip(n) for _, n in bits_]
        for c_ in self.chips:
            lay.addWidget(c_)
        lay.addStretch(1)

    def set(self, mask: int | None, bad_when_off: tuple[int, ...] = ()) -> None:
        for (bit, name), c_ in zip(self.bits, self.chips):
            if mask is None:
                c_.set("", name)
            elif mask & (1 << bit):
                c_.set("ok", name)
            else:
                c_.set("bad" if bit in bad_when_off else "", name)


class HoldButton(QtWidgets.QPushButton):
    """Fires only after being held down for `hold_ms`; shows the progress."""
    held = QtCore.pyqtSignal()

    def __init__(self, text: str, hold_ms: int = 1200):
        super().__init__(text)
        self.hold_ms = hold_ms
        self.setProperty("kind", "danger")
        self._t0 = 0.0
        self._prog = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Držte {hold_ms / 1000:.1f} s".replace(".", ","))

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self._t0 = time.monotonic()
            self._timer.start()
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self._cancel()
        super().mouseReleaseEvent(e)

    def leaveEvent(self, e):
        self._cancel()
        super().leaveEvent(e)

    def changeEvent(self, e):
        if e.type() == QtCore.QEvent.Type.EnabledChange and not self.isEnabled():
            self._cancel()
        super().changeEvent(e)

    def _cancel(self):
        self._timer.stop()
        self._prog = 0.0
        self.update()

    def _tick(self):
        self._prog = (time.monotonic() - self._t0) * 1000 / self.hold_ms
        if self._prog >= 1.0:
            self._cancel()
            self.held.emit()
        self.update()

    def paintEvent(self, e):
        super().paintEvent(e)
        if self._prog > 0:
            p = QtGui.QPainter(self)
            col = QtGui.QColor(C["danger"])
            col.setAlpha(70)
            r = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
            r.setWidth(r.width() * min(1.0, self._prog))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(col)
            p.drawRoundedRect(r, 7, 7)


def card(kicker: str, title: str) -> vs.Card:
    return vs.Card(kicker, title, body=False)


def fmt(v, nd=1, unit="", sign=False):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    try:
        return vs.cz(float(v), nd, sign=sign) + (f" {unit}" if unit else "")
    except (TypeError, ValueError):
        return str(v)


def user_dir() -> str:
    d = os.environ.get("ZENITH_DATA") or os.path.join(os.path.expanduser("~"), "Documents", "Zenith")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        d = os.path.expanduser("~")
    return d


# =====================================================================
#  Main window
# =====================================================================
class DiagWindow(QtWidgets.QMainWindow):
    PC_KEY, PC_VAL, PC_UNIT, PC_DEF, PC_RANGE, PC_HELP = range(6)

    def __init__(self, port: str | None, demo: bool):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1500, 980)
        self.reader = None
        self.port_name = ""
        self.s: dict = {}
        self.nav: dict = {}
        self.pos: dict = {}
        self.raw: dict = {}
        self.fl: dict = {}
        self.d_time = {"S": 0.0, "N": 0.0, "P": 0.0, "R": 0.0, "F": 0.0}
        self.fw_info = ""
        self.params: list[dict] = []          # {"id", "key", "val", "lo", "hi", "def", "unit", "desc"}
        self.param_rows: dict[str, int] = {}
        self.param_edits: dict[str, float] = {}
        self.param_items: dict[str, QtWidgets.QTreeWidgetItem] = {}
        self.param_group_items: list[QtWidgets.QTreeWidgetItem] = []
        self.t0 = time.monotonic()
        self.log_lines: list[str] = []
        self.dumping = False
        self.rec_file = None
        self.rec_writer = None

        root = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.setCentralWidget(root)
        outer.addWidget(self._build_header())
        outer.addWidget(self._build_diag_bar())
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._scroll(self._build_overview()), "Přehled")
        self.tabs.addTab(self._scroll(self._build_nav_tab()), "Poloha a senzory")
        self.tabs.addTab(self._build_graphs(), "Grafy")
        self.tabs.addTab(self._scroll(self._build_raw_tab()), "Surová data IMU")
        self.tabs.addTab(self._scroll(self._build_act_tab()), "Aktuátory a pyro")
        self.tabs.addTab(self._build_params(), "Parametry a časování")
        self.tabs.addTab(self._scroll(self._build_log_tab()), "Letový záznam")
        outer.addWidget(self.tabs, 1)

        QtGui.QShortcut(QtGui.QKeySequence(Qt.Key.Key_Escape), self, activated=self.disarm)
        self._fill_timer = QtCore.QTimer(self, singleShot=True, interval=100, timeout=self._param_fill)
        self.diag_timer = QtCore.QTimer(self, interval=200, timeout=self._refresh_diag)
        self.diag_timer.start()
        self.refresh_ports()
        self._refresh_diag()
        if demo:
            self.port_box.setCurrentIndex(self.port_box.findData("DEMO"))
            self.connect_toggle()
        elif port:
            idx = self.port_box.findData(port)
            if idx < 0:
                self.port_box.addItem(port, port)
                idx = self.port_box.count() - 1
            self.port_box.setCurrentIndex(idx)
            self.connect_toggle()

    @staticmethod
    def _scroll(w: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        s = QtWidgets.QScrollArea()
        s.setWidgetResizable(True)
        s.setWidget(w)
        return s

    # ------------------------------------------------------------ header
    def _build_header(self) -> QtWidgets.QWidget:
        h = QtWidgets.QFrame()
        h.setObjectName("header")
        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(0)
        brand.addWidget(label("ZENITH V2.4 · MASTER · DIAGNOSTIKA", "kicker"))
        word = QtWidgets.QHBoxLayout()
        word.setSpacing(0)
        word.addWidget(label("Zenith", "title"))
        word.addWidget(label(" Console", "wordAccent"))
        word.addStretch(1)
        brand.addLayout(word)
        brand.addStretch(1)

        lay = QtWidgets.QHBoxLayout()
        lay.setSpacing(10)
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(190)
        self.port_box.setToolTip("Sériový port Masteru (USB)")
        lay.addWidget(self.port_box)
        b = button("Obnovit", tip="Znovu vyhledat porty")
        b.clicked.connect(self.refresh_ports)
        lay.addWidget(b)
        self.conn_btn = button("Připojit", "primary")
        self.conn_btn.clicked.connect(self.connect_toggle)
        lay.addWidget(self.conn_btn)
        self.conn_lb = label("", "subtle")
        lay.addWidget(self.conn_lb)
        lay.addStretch(1)
        b = button("Bzučák 1 s", tip="Krátké pípnutí: ověří, že mluvíte se správnou raketou (BUZZ 1000)")
        b.clicked.connect(lambda: self._cmd("BUZZ 1000"))
        lay.addWidget(b)
        self.abort_btn = button("ABORT", "danger", "Přerušit odpočet / start (stejně jako ze země)")
        self.abort_btn.clicked.connect(lambda: self._cmd("ABORT"))
        lay.addWidget(self.abort_btn)

        # second line: board status
        self.chip_board = Chip("Deska")
        self.chip_pos = Chip("Position")
        self.chip_rampa = Chip("Rampa")
        self.chip_rbf = Chip("RBF")
        self.chip_bat = Chip("Baterie")
        self.chip_amp = Chip("Proud")
        self.chip_rail = Chip("Pyro lišta")
        status = QtWidgets.QHBoxLayout()
        status.setSpacing(8)
        status.addStretch(1)
        for c_ in (self.chip_board, self.chip_pos, self.chip_rampa, self.chip_rbf, self.chip_bat, self.chip_amp,
                   self.chip_rail):
            status.addWidget(c_)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8)
        right.addLayout(lay)
        right.addLayout(status)
        outer = QtWidgets.QHBoxLayout(h)
        outer.setContentsMargins(20, 12, 20, 12)
        outer.setSpacing(18)
        outer.addLayout(brand)
        outer.addLayout(right, 1)
        return h

    def _build_diag_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(20, 8, 20, 8)
        lay.setSpacing(8)
        self.chip_state = Chip("Stav: —")
        self.chip_fw = Chip("Firmware: —")
        self.chip_sync = Chip("Parametry: —")
        self.chip_err = Chip("Chyby: —")
        self.chip_go = Chip("")
        self.chip_go.setVisible(False)
        for c_ in (self.chip_state, self.chip_fw, self.chip_sync, self.chip_err, self.chip_go):
            lay.addWidget(c_)
        lay.addStretch(1)
        self.diag_btn = button("DIAG režim", "primary",
                               "Zapne/vypne DIAG režim Masteru (jen z IDLE nebo ABORT). Jen v něm fungují "
                               "testy pyro, FORCE_BAT a napájení serv (záložka Aktuátory a pyro).")
        self.diag_btn.setCheckable(True)
        self.diag_btn.clicked.connect(self.toggle_diag)
        lay.addWidget(self.diag_btn)
        b = button("Pad reference", tip="Teď zachytit referenční tlak a GPS počátek (raketa na rampě).")
        b.clicked.connect(lambda: self._cmd("PADREF"))
        lay.addWidget(b)
        b = button("Kalibrace gyra", tip="Kalibrace gyra na Position MCU. Raketou 1 s nehýbat!")
        b.clicked.connect(lambda: self._cmd("GYROCAL"))
        lay.addWidget(b)
        self.prep_btn = button("Připravit", "danger",
                               "PREPARE: reference na rampě, kamera, rampa zkontroluje klíč, rozbušku a baterii → READY.")
        self.prep_btn.clicked.connect(self.prepare)
        lay.addWidget(self.prep_btn)
        self.cd_btn = button("Odpočet…", "danger", "COUNTDOWN: přijme se jen v READY a když je vše GO.")
        self.cd_btn.clicked.connect(self.countdown)
        lay.addWidget(self.cd_btn)
        b = button("SAFE", tip="Zpět do IDLE (pyro lišta vypnuta).")
        b.clicked.connect(lambda: self._cmd("SAFE"))
        lay.addWidget(b)
        self.rec_btn = button("Nahrávat", tip="Ukládá každou řádku z desky do CSV (čas, typ, obsah).")
        self.rec_btn.setCheckable(True)
        self.rec_btn.clicked.connect(self.toggle_record)
        lay.addWidget(self.rec_btn)
        return bar

    # ------------------------------------------------------------ overview
    def _build_overview(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)

        self.board_kv = {}
        names = {
            "M": ("Master MCU", "Pico 2 · stav mise, pyro, log, LoRa",
                  ["Online", "Firmware", "Stav", "Chyba", "Baterie", "Proud", "Napájení", "RBF", "Breakwire",
                   "Pyro lišta", "Periferie I²C", "Flash log", "LoRa", "Spoj Position", "Kamera", "Lokátor",
                   "Restart"]),
            "P": ("Position MCU", "XIAO RP2350 · senzory, navigace",
                  ["Online", "Firmware", "Senzory", "Navigace", "Orientace", "Osa špičky", "IMU Hz",
                   "IMU ztracené", "Špička g", "Bias gyra", "GPS", "Restart"]),
            "R": ("Rampa", "Pico 2 · pogo UART 20 Hz",
                  ["Spojení", "Stav", "Klíč", "Rozbušky", "Pyro rampy", "Baterie rampy", "LoRa rampy",
                   "Napájení rakety", "Připravena", "Chyby rampy", "Odpočet"]),
        }
        for col, key in enumerate(("M", "P", "R")):
            title, kick, rows = names[key]
            c_ = card(kick, title)
            kv = KV(rows)
            c_.lay.addWidget(kv)
            c_.lay.addStretch(1)
            self.board_kv[key] = kv
            if key == "P":
                b = button("Restart", tip="Restart Position MCU (POSREBOOT), jen na zemi")
                b.clicked.connect(lambda: self._cmd("POSREBOOT"))
                c_.head.addWidget(b)
            if key == "R":
                b = button("Ping", tip="RAMPA PING")
                b.clicked.connect(lambda: self._cmd("RAMPA PING"))
                c_.head.addWidget(b)
            g.addWidget(c_, 0, col)

        att = card("Orientace", "Jak je raketa nakloněná")
        row = QtWidgets.QHBoxLayout()
        self.rocket = RocketView()
        self.rocket.setMinimumSize(300, 300)
        row.addWidget(self.rocket, 3)
        col = QtWidgets.QVBoxLayout()
        self.bubble = TiltBubble()
        self.bubble.setMinimumSize(180, 180)
        col.addWidget(self.bubble, 1)
        self.att_kv = KV(["Náklon", "Směr náklonu", "Roll", "Pitch / heading", "Euler (Position)", "Osa špičky"])
        col.addWidget(self.att_kv)
        row.addLayout(col, 2)
        att.lay.addLayout(row)
        g.addWidget(att, 1, 0, 1, 2)

        fl = card("Let", "Mise a padáky")
        self.flight_kv = KV(["Stav", "Do startu", "Výška / max", "Svislá rychlost", "Mach", "Zrychlení osové",
                             "Hlavní padák v", "Drogue (PYRO1)", "Main (PYRO2)", "Záloha (PYRO3)",
                             "Poslední let", "Pyro událostí", "Letový log"])
        fl.lay.addWidget(self.flight_kv)
        fl.lay.addWidget(label("Události", "subtle"))
        self.ev_flags = Flags(EVENT_BITS)
        fl.lay.addWidget(self.ev_flags)
        fl.lay.addStretch(1)
        g.addWidget(fl, 1, 2)

        pr = card("Kontrola", "Chyby a varování (Go / No-Go)")
        self.problems = QtWidgets.QListWidget()
        self.problems.setMinimumHeight(170)
        pr.lay.addWidget(self.problems)
        g.addWidget(pr, 2, 0, 1, 3)
        for i in range(3):
            g.setColumnStretch(i, 1)
        return w

    # ------------------------------------------------------------ nav tab
    def _build_nav_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)
        mp = card("Poloha", "Vůči rampě (sever nahoru)")
        self.padmap = PadMap()
        self.padmap.setMinimumSize(340, 340)
        mp.lay.addWidget(self.padmap, 1)
        b = button("Vymazat stopu")
        b.clicked.connect(self.padmap.clear)
        mp.head.addWidget(b)
        g.addWidget(mp, 0, 0, 2, 1)

        st = card("Stav", "Navigační řešení")
        self.nav_kv = KV(["Poloha E / N / výška", "Rychlost E / N / svislá", "Výška KF / baro", "σ výšky",
                          "Zrychlení osové / |a|", "Mach", "Max. výška", "Režim"])
        st.lay.addWidget(self.nav_kv)
        st.lay.addWidget(label("Příznaky", "subtle"))
        self.nav_flags = Flags(NAV_BITS[:8])
        self.nav_flags2 = Flags(NAV_BITS[8:])
        st.lay.addWidget(self.nav_flags)
        st.lay.addWidget(self.nav_flags2)
        st.lay.addStretch(1)
        g.addWidget(st, 0, 1)

        se = card("Senzory", "GPS · barometr · IMU · magnetometr")
        self.sens_kv = KV(["GPS", "Poloha (lat, lon)", "GPS výška", "Rampa (GPS počátek)", "Vzdálenost od rampy",
                           "Barometr", "Teplota", "Bias gyra", "IMU", "Restart Position"])
        se.lay.addWidget(self.sens_kv)
        self.sens_flags = Flags(SENS_BITS)
        se.lay.addWidget(self.sens_flags)
        se.lay.addStretch(1)
        b = button("GPS překonfigurovat", tip="GPSRECFG: znovu poslat konfiguraci přijímači")
        b.clicked.connect(lambda: self._cmd("GPSRECFG"))
        se.head.addWidget(b)
        g.addWidget(se, 1, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        return w

    # ------------------------------------------------------------ graphs
    def _build_graphs(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(12)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Okno grafů", "muted"))
        self.span_box = QtWidgets.QComboBox()
        for s in (10, 30, 60, 120, 300):
            self.span_box.addItem(f"{s} s", s)
        self.span_box.setCurrentIndex(1)
        self.span_box.currentIndexChanged.connect(self._span_changed)
        top.addWidget(self.span_box)
        b = button("Vymazat")
        b.clicked.connect(lambda: [c_.clear() for c_ in self.charts])
        top.addWidget(b)
        self.pause_btn = button("Pozastavit")
        self.pause_btn.setCheckable(True)
        top.addWidget(self.pause_btn)
        top.addStretch(1)
        g.addLayout(top, 0, 0, 1, 2)
        self.ch_alt = StripChart("Výška nad rampou", ["KF", "baro"], "m")
        self.ch_vel = StripChart("Svislá rychlost", ["vz"], "m/s")
        self.ch_acc = StripChart("Zrychlení", ["osové", "|a|"], "g")
        self.ch_tilt = StripChart("Náklon", ["náklon", "limit"], "°")
        self.ch_pos = StripChart("Poloha vůči rampě", ["E", "N"], "m")
        self.ch_mach = StripChart("Mach a σ výšky", ["Mach", "σ výšky"], "")
        self.ch_pwr = StripChart("Napájení", ["baterie", "proud"], "V / A")
        self.ch_cont = StripChart("Kontinuita pyro", ["P1", "P2", "P3", "lišta"], "V")
        self.charts = [self.ch_alt, self.ch_vel, self.ch_acc, self.ch_tilt, self.ch_pos, self.ch_mach,
                       self.ch_pwr, self.ch_cont]
        for i, c_ in enumerate(self.charts):
            g.addWidget(c_, 1 + i // 2, i % 2)
        for r in range(1, 5):
            g.setRowStretch(r, 1)
        return w

    def _span_changed(self) -> None:
        s = float(self.span_box.currentData())
        for c_ in self.charts:
            c_.span = s
            c_.update()

    # ------------------------------------------------------------ raw IMU
    def _build_raw_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(12)
        top = QtWidgets.QHBoxLayout()
        self.raw_btn = button("Zapnout surová data", "primary")
        self.raw_btn.setCheckable(True)
        self.raw_btn.toggled.connect(self._raw_toggled)
        top.addWidget(self.raw_btn)
        top.addWidget(label("Position MCU posílá ~25×/s. Osy desky, jak jsou natištěné. Postav raketu špičkou "
                            "nahoru: osa s +1 g je osa špičky (parametr nose_axis, 0 = automaticky).", "muted"))
        top.addStretch(1)
        g.addLayout(top, 0, 0, 1, 2)
        kv = card("Hodnoty", "Poslední vzorek")
        self.raw_kv = KV(["Akcelerometr X / Y / Z", "|a|", "Osa s +1 g", "Gyroskop X / Y / Z", "|ω|",
                          "Magnetometr X / Y / Z", "|B|", "Podíl high-g IMU"])
        kv.lay.addWidget(self.raw_kv)
        g.addWidget(kv, 1, 0, 3, 1, Qt.AlignmentFlag.AlignTop)
        self.ch_raw_acc = StripChart("Akcelerometr", ["X", "Y", "Z"], "g", span_s=10.0)
        self.ch_raw_gyr = StripChart("Gyroskop", ["X", "Y", "Z"], "°/s", span_s=10.0)
        self.ch_raw_mag = StripChart("Magnetometr", ["X", "Y", "Z"], "µT", span_s=10.0)
        for i, c_ in enumerate((self.ch_raw_acc, self.ch_raw_gyr, self.ch_raw_mag)):
            c_.setMinimumHeight(180)
            g.addWidget(c_, 1 + i, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 2)
        for r in (1, 2, 3):
            g.setRowStretch(r, 1)
        return w

    def _raw_toggled(self, on: bool) -> None:
        self._cmd(f"RAW {1 if on else 0}")
        self.raw_btn.setText("Vypnout surová data" if on else "Zapnout surová data")

    def _refresh_raw(self) -> None:
        d = self.raw
        a = [to_float(d.get(k)) for k in ("ax", "ay", "az")]
        gy = [to_float(d.get(k)) for k in ("gx", "gy", "gz")]
        m = [to_float(d.get(k)) for k in ("mx", "my", "mz")]

        def j(v, nd):
            return " / ".join(f"{x:+.{nd}f}" for x in v)
        am, gm, mm = (math.sqrt(sum(x * x for x in v)) for v in (a, gy, m))
        kv = self.raw_kv
        kv.set("Akcelerometr X / Y / Z", j(a, 3) + " g")
        kv.set("|a|", f"{am:.3f} g", GOOD if abs(am - 1.0) < 0.04 else None)
        big = max(range(3), key=lambda i: abs(a[i]))
        axis = ("+" if a[big] > 0 else "−") + "XYZ"[big]
        kv.set("Osa s +1 g", f"{axis}  ({a[big]:+.2f} g)  → nose_axis {1 + 2 * big + (0 if a[big] > 0 else 1)}",
               GOOD if abs(abs(a[big]) - 1) < 0.1 else None)
        kv.set("Gyroskop X / Y / Z", j(gy, 2) + " °/s")
        kv.set("|ω|", f"{gm:.2f} °/s", GOOD if gm < 1.0 else None)
        kv.set("Magnetometr X / Y / Z", j(m, 1) + " µT")
        kv.set("|B|", f"{mm:.1f} µT", GOOD if 25.0 < mm < 65.0 else WARN)
        kv.set("Podíl high-g IMU", fmt(to_float(d.get("hgw")), 2))

    # ------------------------------------------------------------ actuators and pyro
    def _build_act_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)

        pc = card("Master MCU · PYRO1–3", "Pyro kanály")
        self.arm_chip = Chip("LIŠTA VYPNUTA")
        pc.head.addWidget(self.arm_chip)
        self.arm_btn = button("Zapnout pyro lištu…", "primary", "ARM 1: zapne PYRO_SW_ARM (jen DIAG a RBF venku)")
        self.arm_btn.setProperty("size", "big")
        self.arm_btn.clicked.connect(self.arm_clicked)
        self.disarm_btn = button("Vypnout lištu  (Esc)", "", "ARM 0: okamžitě vypne pyro lištu")
        self.disarm_btn.setProperty("size", "big")
        self.disarm_btn.clicked.connect(self.disarm)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.arm_btn, 1)
        row.addWidget(self.disarm_btn, 1)
        pc.lay.addLayout(row)
        self.pyro_note = label("", "muted")
        self.pyro_note.setWordWrap(True)
        pc.lay.addWidget(self.pyro_note)
        self.pyro_rows = []
        for i, nm in enumerate(("PYRO1 · brzdicí padák (drogue)", "PYRO2 · hlavní padák (main)",
                                "PYRO3 · záloha (oba padáky)")):
            fr = QtWidgets.QFrame()
            fr.setStyleSheet(f"QFrame {{ border-top: 1px solid {C['line']}; }} QLabel, QPushButton {{ border-top: none; }}")
            gl = QtWidgets.QGridLayout(fr)
            gl.setContentsMargins(0, 10, 0, 2)
            gl.setHorizontalSpacing(8)
            gl.setVerticalSpacing(6)
            head = QtWidgets.QHBoxLayout()
            head.addWidget(label(nm, "big"))
            head.addStretch(1)
            cont = Chip("—")
            fired = Chip("")
            fired.setVisible(False)
            head.addWidget(fired)
            head.addWidget(cont)
            gl.addLayout(head, 0, 0, 1, 2)
            raw = label("ADC —", "subtle")
            gl.addWidget(raw, 1, 0)
            fire = HoldButton(f"Podržet: ODPÁLIT {i + 1}")
            fire.held.connect(lambda k=i: self.fire(k + 1))
            gl.addWidget(fire, 1, 1)
            gl.setColumnStretch(1, 1)
            pc.lay.addWidget(fr)
            self.pyro_rows.append({"cont": cont, "raw": raw, "fired": fired, "fire": fire})
        pc.lay.addStretch(1)
        warn = label("Pyro test jen bez náplní: na svorky testovací žárovku nebo odpor. Délka pulzu je parametr "
                     "pyro_pulse_ms. Kanál se odpálí jen v DIAG režimu, s vytaženým RBF a zapnutou lištou.", "subtle")
        warn.setWordWrap(True)
        pc.lay.addWidget(warn)
        g.addWidget(pc, 0, 0, 2, 1)

        pw = card("Napájení a periferie", "Ruční ovládání")
        self.act_kv = KV(["Napájení", "Baterie", "FORCE_BAT", "Kamera", "Lokátor", "Rampa"])
        pw.lay.addWidget(self.act_kv)
        gl = QtWidgets.QGridLayout()
        gl.setSpacing(8)
        items = [("FORCE_BAT zap", "FORCEBAT 1", "Napájení z baterie rakety (jen DIAG)"),
                 ("FORCE_BAT vyp", "FORCEBAT 0", ""),
                 ("Napájení serv zap", "SERVOPWR 1", "PCA9685 výstup napájení serv (jen DIAG)"),
                 ("Napájení serv vyp", "SERVOPWR 0", ""),
                 ("Kamera nahrávat", "CAM 1", ""), ("Kamera stop", "CAM 0", ""),
                 ("Lokátor zap", "LOCATE 1", "Hlasitý bzučák pro dohledání"), ("Lokátor vyp", "LOCATE 0", ""),
                 ("Bzučák 1 s", "BUZZ 1000", ""), ("Ping rampy", "RAMPA PING", ""),
                 ("Restart Position MCU", "POSREBOOT", ""), ("Restart Masteru", "REBOOT", "")]
        self.act_btns = {}
        for k, (txt, cmd, tip) in enumerate(items):
            b = button(txt, tip=tip or cmd)
            b.clicked.connect(lambda _=False, c_=cmd: self._cmd(c_))
            self.act_btns[cmd] = b
            gl.addWidget(b, k // 2, k % 2)
        pw.lay.addLayout(gl)
        pw.lay.addStretch(1)
        g.addWidget(pw, 0, 1)

        lc = card("Deska", "Záznam")
        b = button("Vymazat")
        lc.head.addWidget(b)
        self.show_stream = QtWidgets.QCheckBox("i proud $S/$N/$P")
        lc.head.addWidget(self.show_stream)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1500)
        self.log_view.setMinimumHeight(260)
        b.clicked.connect(self.log_view.clear)
        lc.lay.addWidget(self.log_view, 1)
        self.cmd_line = QtWidgets.QLineEdit()
        self.cmd_line.setPlaceholderText("Příkaz pro desku, např. HELLO, FLIGHT, PARAM SET main_alt_m 350  (Enter)")
        self.cmd_line.returnPressed.connect(self._send_raw)
        lc.lay.addWidget(self.cmd_line)
        g.addWidget(lc, 1, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        g.setRowStretch(1, 1)
        return w

    def _send_raw(self) -> None:
        txt = self.cmd_line.text().strip()
        if txt:
            self._cmd(txt)
            self.cmd_line.clear()

    # ------------------------------------------------------------ parameters
    def _build_params(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(10)
        note = label(
            "Tabulka parametrů žije ve flash Masteru (sektory A/B). Měnit lze mimo odpočet a let. Postup: Načíst "
            "z desky → klepnout na hodnotu a upravit (žlutě) → Odeslat změny → Uložit do flash. Modře = jiná než "
            "výchozí. Výška hlavního padáku je main_alt_m (skupina Padáky a apogeum).", "muted")
        note.setWordWrap(True)
        lay.addWidget(note)
        row = QtWidgets.QHBoxLayout()
        for txt, kind, fn, tip in (
                ("Načíst z desky", "primary", lambda: self._cmd("PARAM LIST"), "PARAM LIST"),
                ("Odeslat změny", "primary", self.param_send, "PARAM SET pro každou žlutou hodnotu"),
                ("Uložit do flash", "", lambda: self._cmd("PARAM SAVE"), "PARAM SAVE: přežije restart"),
                ("Výchozí hodnoty", "danger", self.param_defaults, "PARAM DEFAULTS (neuloží)"),
                ("Export profilu…", "", self.param_export, "Uloží tabulku do JSON na počítači"),
                ("Import profilu…", "", self.param_import, "Načte JSON a označí rozdíly k odeslání")):
            b = button(txt, kind, tip)
            b.clicked.connect(fn)
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.filter = QtWidgets.QLineEdit()
        self.filter.setPlaceholderText("hledat: název nebo popis (např. padák, main, rampa, pyro)")
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._param_filter)
        row.addWidget(self.filter, 1)
        self.param_changed_only = QtWidgets.QCheckBox("Jen změněné")
        self.param_changed_only.setToolTip("Jen hodnoty jiné než výchozí a neodeslané úpravy")
        self.param_changed_only.toggled.connect(self._param_filter)
        row.addWidget(self.param_changed_only)
        for txt, fn in (("Rozbalit vše", lambda: self.ptree.expandAll()),
                        ("Sbalit vše", lambda: self.ptree.collapseAll())):
            b = button(txt)
            b.clicked.connect(fn)
            row.addWidget(b)
        lay.addLayout(row)

        t = QtWidgets.QTreeWidget()
        t.setColumnCount(6)
        t.setHeaderLabels(["parametr", "hodnota", "jednotka", "výchozí", "rozsah", "co dělá"])
        t.setRootIsDecorated(True)
        t.setUniformRowHeights(True)
        t.setWordWrap(False)
        t.setIndentation(14)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)  # value column only, below
        t.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        hdr = t.header()
        hdr.setStretchLastSection(True)
        hdr.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        t.itemChanged.connect(self._param_item_changed)
        t.itemClicked.connect(self._param_item_clicked)
        t.itemDoubleClicked.connect(self._param_item_clicked)
        self.ptree = t
        lay.addWidget(t, 1)
        self._param_fill()
        return w

    @staticmethod
    def _num(v: float) -> str:
        return f"{v:.6g}"

    @staticmethod
    def _help(p: dict) -> str:
        return PARAM_HELP.get(p["key"]) or p["desc"]

    def _param_fill(self) -> None:
        t = self.ptree
        expanded = {g.text(0) for g in self.param_group_items if g.isExpanded()} if self.param_group_items else None
        t.blockSignals(True)
        t.clear()
        self.param_items.clear()
        self.param_rows = {p["key"]: i for i, p in enumerate(self.params)}
        heads: list[QtWidgets.QTreeWidgetItem] = []
        bold = QtGui.QFont(t.font())
        bold.setBold(True)
        for name, sub, _ in PARAM_GROUPS + [PARAM_OTHER]:
            g = QtWidgets.QTreeWidgetItem([name, "", "", "", "", sub])
            g.setData(self.PC_HELP, Qt.ItemDataRole.UserRole, sub)
            g.setFlags(Qt.ItemFlag.ItemIsEnabled)
            g.setFont(0, bold)
            g.setForeground(0, QtGui.QColor(C["accent"]))
            g.setForeground(self.PC_HELP, QtGui.QColor(C["subtle"]))
            heads.append(g)
        for p in sorted(self.params, key=lambda q: q["id"]):
            help_ = self._help(p)
            it = QtWidgets.QTreeWidgetItem([
                p["key"], self._num(self.param_edits.get(p["key"], p["val"])), p["unit"], self._num(p["def"]),
                f"{self._num(p['lo'])} … {self._num(p['hi'])}", help_])
            it.setData(0, Qt.ItemDataRole.UserRole, p["key"])
            it.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEditable)
            tip = f"{p['key']} (#{p['id']})\n{help_}" + (f"\nfirmware: {p['desc']}" if p["desc"] != help_ else "")
            for c_ in range(6):
                it.setToolTip(c_, tip)
            for c_ in (self.PC_UNIT, self.PC_DEF, self.PC_RANGE, self.PC_HELP):
                it.setForeground(c_, QtGui.QColor(C["muted"] if c_ == self.PC_HELP else C["subtle"]))
            it.setTextAlignment(self.PC_VAL, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            heads[param_group(p["key"])].addChild(it)
            self.param_items[p["key"]] = it
        self.param_group_items = [g for g in heads if g.childCount()]
        for g in self.param_group_items:
            t.addTopLevelItem(g)
            g.setExpanded(expanded is None or g.text(0) in expanded)
        for k in self.param_items:
            self._param_color(k)
        t.blockSignals(False)
        for c_ in range(self.PC_HELP):
            t.resizeColumnToContents(c_)
        t.setColumnWidth(self.PC_VAL, max(t.columnWidth(self.PC_VAL), 90))
        self._param_filter()

    def _param_show(self, key: str, v: float) -> None:
        it = self.param_items.get(key)
        if it is None:
            return
        self.ptree.blockSignals(True)
        it.setText(self.PC_VAL, self._num(v))
        self.ptree.blockSignals(False)
        self._param_color(key)

    def _param_changed(self, p: dict) -> bool:
        return p["key"] in self.param_edits or abs(p["val"] - p["def"]) > 1e-6 * (1 + abs(p["def"]))

    def _param_color(self, key: str) -> None:
        it = self.param_items.get(key)
        r = self.param_rows.get(key)
        if it is None or r is None:
            return
        p = self.params[r]
        self.ptree.blockSignals(True)
        if key in self.param_edits:
            it.setBackground(self.PC_VAL, QtGui.QColor(245, 165, 28, 70))
            it.setForeground(self.PC_VAL, QtGui.QColor(C["accent_soft"]))
        elif self._param_changed(p):
            it.setBackground(self.PC_VAL, QtGui.QColor(74, 163, 255, 45))      # differs from default
            it.setForeground(self.PC_VAL, QtGui.QColor(C["text"]))
        else:
            it.setBackground(self.PC_VAL, QtGui.QBrush())
            it.setForeground(self.PC_VAL, QtGui.QColor(C["text"]))
        self.ptree.blockSignals(False)
        self._param_group_counts()

    def _param_group_counts(self) -> None:
        """'10 param. · 2 jiné než výchozí · 1 k odeslání' in each group header."""
        self.ptree.blockSignals(True)
        for g in self.param_group_items:
            n = g.childCount()
            keys = [g.child(i).data(0, Qt.ItemDataRole.UserRole) for i in range(n)]
            ch = sum(1 for k in keys if self._param_changed(self.params[self.param_rows[k]]))
            ed = sum(1 for k in keys if k in self.param_edits)
            txt = f"{n} param." + (f" · {ch} jiné než výchozí" if ch else "") + (f" · {ed} k odeslání" if ed else "")
            g.setText(self.PC_HELP, txt + "  —  " + (g.data(self.PC_HELP, Qt.ItemDataRole.UserRole) or ""))
            g.setForeground(self.PC_HELP, QtGui.QColor(C["accent_soft"] if ed else C["subtle"]))
        self.ptree.blockSignals(False)

    def _param_item_clicked(self, it: QtWidgets.QTreeWidgetItem, col: int) -> None:
        if it.data(0, Qt.ItemDataRole.UserRole) is not None and col == self.PC_VAL:
            self.ptree.editItem(it, self.PC_VAL)

    def _param_item_changed(self, it: QtWidgets.QTreeWidgetItem, col: int) -> None:
        key = it.data(0, Qt.ItemDataRole.UserRole)
        if col != self.PC_VAL or key is None or key not in self.param_rows:
            return
        p = self.params[self.param_rows[key]]
        try:
            v = float(it.text(self.PC_VAL).replace(",", ".").strip())
        except ValueError:
            self._param_show(key, self.param_edits.get(key, p["val"]))
            return
        v = max(p["lo"], min(p["hi"], v))
        if abs(v - p["val"]) <= 1e-9 * (1 + abs(v)):
            self.param_edits.pop(key, None)                 # typed back to the board's value
        else:
            self.param_edits[key] = v
        self._param_show(key, v)

    def _param_filter(self, *_args) -> None:
        if not hasattr(self, "ptree"):
            return
        txt = self.filter.text().strip().lower()
        only = self.param_changed_only.isChecked()
        for g in self.param_group_items:
            shown = 0
            for i in range(g.childCount()):
                it = g.child(i)
                p = self.params[self.param_rows[it.data(0, Qt.ItemDataRole.UserRole)]]
                hit = not txt or txt in p["key"].lower() or txt in self._help(p).lower()
                hide = not hit or (only and not self._param_changed(p))
                it.setHidden(hide)
                shown += not hide
            g.setHidden(shown == 0)
            if (txt or only) and shown:
                g.setExpanded(True)

    def param_set(self, key: str, value: float) -> None:
        """Mark one parameter for sending (as if typed into the table)."""
        r = self.param_rows.get(key)
        if r is None:
            return
        p = self.params[r]
        v = max(p["lo"], min(p["hi"], float(value)))
        if abs(v - p["val"]) <= 1e-9 * (1 + abs(v)):
            self.param_edits.pop(key, None)
        else:
            self.param_edits[key] = v
        self._param_show(key, v)

    def param_send(self) -> None:
        if self.reader is None or not self.param_edits:
            return
        edits = sorted(self.param_edits.items())
        self.log(f"> PARAM SET × {len(edits)}  (nezapomeňte Uložit do flash)")
        for key, v in edits:
            self.reader.send(f"PARAM SET {key} {v:.6g}")

    def param_defaults(self) -> None:
        r = QtWidgets.QMessageBox.question(self, APP_NAME, "Vrátit všechny parametry na výchozí? "
                                           "(do flash se zapíše až tlačítkem Uložit)")
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("PARAM DEFAULTS")
            QtCore.QTimer.singleShot(300, lambda: self._cmd("PARAM LIST"))

    def param_export(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export", os.path.join(user_dir(), "zenith_params.json"),
                                                        "JSON (*.json)")
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump({p["key"]: self.param_edits.get(p["key"], p["val"]) for p in self.params}, fh, indent=2)

    def param_import(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import", user_dir(), "JSON (*.json)")
        if not path:
            return
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        for p in self.params:
            if p["key"] in d and abs(float(d[p["key"]]) - p["val"]) > 1e-9:
                self.param_edits[p["key"]] = max(p["lo"], min(p["hi"], float(d[p["key"]])))
        self._param_fill()
        self.log(f"# import: {len(self.param_edits)} rozdílů, odešlete tlačítkem „Odeslat změny“")

    def _pval(self, key: str, default: float) -> float:
        r = self.param_rows.get(key)
        return self.params[r]["val"] if r is not None else default

    # ------------------------------------------------------------ flight log
    def _build_log_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QGridLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(14)
        bc = card("V raketě", "Flash Masteru (W25Q128)")
        self.log_info = label("Obsah neznámý — Zjistit obsah", "muted")
        self.log_info.setWordWrap(True)
        bc.lay.addWidget(self.log_info)
        r = QtWidgets.QHBoxLayout()
        for txt, fn, tip, kind in (("Zjistit obsah", lambda: self._cmd("LOG INFO"), "LOG INFO", ""),
                                   ("Stáhnout vše → CSV", self.dump_log, "LOG DUMP, uloží CSV a události", "primary")):
            b = button(txt, kind, tip)
            b.clicked.connect(fn)
            r.addWidget(b)
        r.addStretch(1)
        b = button("Smazat záznam v raketě…", tip="LOG ERASE CONFIRM, jen v IDLE/DIAG/LANDED")
        b.clicked.connect(self.erase_log)
        r.addWidget(b)
        bc.lay.addLayout(r)
        self.log_prog = QtWidgets.QProgressBar()
        self.log_prog.setFormat("%v / %m záznamů")
        self.log_prog.setValue(0)
        bc.lay.addWidget(self.log_prog)
        bc.lay.addStretch(1)
        lay.addWidget(bc, 0, 0)

        ec = card("Události", "Z logu i živě ($EV)")
        b = button("Vymazat")
        ec.head.addWidget(b)
        self.events = QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMinimumHeight(150)
        b.clicked.connect(self.events.clear)
        ec.lay.addWidget(self.events, 1)
        lay.addWidget(ec, 0, 1)

        pw = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(pw)
        g.setContentsMargins(0, 0, 0, 0)
        self.log_charts = {
            "alt": StripChart("Výška (stažený záznam)", ["KF", "baro"], "m"),
            "vz": StripChart("Svislá rychlost", ["vz"], "m/s"),
            "acc": StripChart("Zrychlení", ["osové", "|a|"], "g"),
            "pwr": StripChart("Baterie a stav", ["baterie V", "stav"], ""),
        }
        for i, c_ in enumerate(self.log_charts.values()):
            c_.setMinimumHeight(190)
            g.addWidget(c_, i // 2, i % 2)
        lay.addWidget(pw, 1, 0, 1, 2)
        lay.setColumnStretch(0, 1)
        lay.setColumnStretch(1, 1)
        lay.setRowStretch(1, 1)
        return w

    def dump_log(self) -> None:
        self.log_lines = []
        self.dumping = True
        self._cmd("LOG DUMP")

    def erase_log(self) -> None:
        if QtWidgets.QMessageBox.question(self, "Smazat log", "Opravdu smazat celý letový log z flash?") == \
                QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("LOG ERASE CONFIRM")

    def save_log(self) -> None:
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
        self._plot_log(rows)
        for t, st, txt in events:
            self.events.appendPlainText(f"[log] {t / 1000:9.3f} s  {st:9s} {txt}")
        path = os.environ.get("ZENITH_LOG_OUT") or QtWidgets.QFileDialog.getSaveFileName(
            self, "Uložit log", os.path.join(user_dir(), time.strftime("zenith_log_%Y%m%d_%H%M%S.csv")),
            "CSV (*.csv)")[0]
        if not path:
            self.log_info.setText(f"staženo {len(rows)} záznamů a {len(events)} událostí (neuloženo)")
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
        self.log_info.setText(f"uloženo {len(rows)} záznamů a {len(events)} událostí → {path}")

    def _plot_log(self, rows) -> None:
        for c_ in self.log_charts.values():
            c_.clear()
        if not rows:
            return
        t0, t1 = rows[0][4] / 1000.0, rows[-1][4] / 1000.0
        span = max(1.0, t1 - t0)
        step = max(1, len(rows) // 2500)
        for c_ in self.log_charts.values():
            c_.span = span
        for r in rows[::step]:
            t = r[4] / 1000.0
            self.log_charts["alt"].add(t, [r[5], r[7]])
            self.log_charts["vz"].add(t, [r[6]])
            self.log_charts["acc"].add(t, [r[8], r[9]])
            self.log_charts["pwr"].add(t, [r[14] / 1000.0, r[1]])

    # ------------------------------------------------------------ link
    def refresh_ports(self) -> None:
        cur = self.port_box.currentData()
        self.port_box.clear()
        try:
            import serial.tools.list_ports as lp
            for p in sorted(lp.comports(), key=lambda p: p.device):
                desc = p.description if p.description and p.description != "n/a" else ""
                self.port_box.addItem(f"{p.device}  {desc}".strip(), p.device)
        except Exception:  # noqa: BLE001  (no pyserial: demo only)
            for p in list_ports():
                self.port_box.addItem(p, p)
        self.port_box.addItem("Simulace (bez hardwaru)", "DEMO")
        idx = self.port_box.findData(cur)
        if idx >= 0:
            self.port_box.setCurrentIndex(idx)

    @property
    def is_open(self) -> bool:
        return self.reader is not None

    def connect_toggle(self) -> None:
        if self.reader is not None:
            self.stop_reader()
            self.on_link_state(False, "odpojeno")
            return
        port = self.port_box.currentData()
        if not port:
            return
        if port == "DEMO":
            self.reader = DemoMaster()
        else:
            self.reader = SerialReader(port)
            self.reader.status.connect(self._link_status)
            self.reader.start()
        self.reader.line.connect(self.on_line)
        self.port_name = port
        self.on_link_state(True, "Simulace" if port == "DEMO" else port)

    def _link_status(self, s: str) -> None:
        self.conn_lb.setText(s)
        self.log(f"# {s}")

    def stop_reader(self) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None

    def on_link_state(self, up: bool, msg: str) -> None:
        self.conn_btn.setText("Odpojit" if up else "Připojit")
        self.conn_btn.setProperty("kind", "" if up else "primary")
        vs.repolish(self.conn_btn)
        self.conn_lb.setText(msg if up else "")
        self.log(f"# {'připojeno: ' + msg if up else msg}")
        self.s, self.nav, self.pos, self.raw, self.fl = {}, {}, {}, {}, {}
        self.fw_info = ""
        self.raw_btn.blockSignals(True)
        self.raw_btn.setChecked(False)
        self.raw_btn.setText("Zapnout surová data")
        self.raw_btn.blockSignals(False)
        if up:
            QtCore.QTimer.singleShot(200 if self.port_name == "DEMO" else 800,
                                     lambda: (self._cmd("HELLO"), self._cmd("PARAM LIST")))

    def _cmd(self, text: str) -> None:
        if not text:
            return
        if self.reader is None:
            self.log(f"# nepřipojeno: {text}")
            return
        self.reader.send(text)
        self.log(f"> {text}")

    send = _cmd

    def log(self, text: str) -> None:
        self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    # ------------------------------------------------------------ actions
    def _state(self) -> str:
        return state_name(to_int(self.s.get("state"), -1)) if self.s else ""

    def toggle_diag(self) -> None:
        self._cmd(f"DIAG {1 if self.diag_btn.isChecked() else 0}")

    def prepare(self) -> None:
        r = QtWidgets.QMessageBox.question(
            self, APP_NAME, "PŘIPRAVIT raketu ke startu (PREPARE)?\n\nRaketa vezme referenci na rampě, zapne "
            "kameru a rampa zkontroluje klíč, rozbušku a baterii.")
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("PREPARE")

    def countdown(self) -> None:
        n, ok = QtWidgets.QInputDialog.getInt(self, "Odpočet", "Spustit odpočet z USB, sekund (10 … 3600):\n\n"
                                              "Všichni za bezpečnostní čáru. ABORT kdykoli tlačítkem nahoře.",
                                              30, 10, 3600)
        if ok:
            self._cmd(f"COUNTDOWN {n}")

    def arm_clicked(self) -> None:
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        box.setWindowTitle("Zapnout pyro lištu")
        box.setText("Zapnout pyro lištu (ARM 1)?")
        box.setInformativeText(
            "Po zapnutí lze podržením tlačítka odpálit kterýkoli kanál.\n\n"
            "• V raketě nejsou náplně, na svorkách je testovací zátěž.\n"
            "• RBF je vytažený jen pro tento test.\n"
            "• Esc nebo „Vypnout lištu“ ji okamžitě vypne.")
        yes = box.addButton("Zapnout", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        no = box.addButton("Zrušit", QtWidgets.QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(no)
        box.exec()
        if box.clickedButton() is yes:
            self._cmd("ARM 1")

    def disarm(self) -> None:
        if self.reader is not None and self._state() == "DIAG":
            self._cmd("ARM 0")

    def fire(self, ch: int) -> None:
        if self._state() != "DIAG":
            self.log("# odpálení zamítnuto: Master není v DIAG režimu")
            return
        if not to_int(self.s.get("pyro")) & 0x40:
            self.log("# odpálení zamítnuto: pyro lišta je vypnutá")
            return
        self._cmd(f"FIRE {ch} CONFIRM")

    def toggle_record(self) -> None:
        if self.rec_btn.isChecked():
            path = os.path.join(user_dir(), time.strftime("zenith_rec_%Y%m%d_%H%M%S.csv"))
            try:
                self.rec_file = open(path, "w", newline="", encoding="utf-8")
            except OSError as e:
                self.log(f"# nelze nahrávat: {e}")
                self.rec_btn.setChecked(False)
                return
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow(["t", "typ", "řádka"])
            self.log(f"# nahrávám do {path}")
        elif self.rec_file:
            self.rec_file.close()
            self.rec_file = self.rec_writer = None
            self.log("# nahrávání ukončeno")

    # ------------------------------------------------------------ parsing
    def on_line(self, s: str) -> None:
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}", s.split(",", 1)[0], s])
        p = s.split(",")
        tag = p[0]
        now = time.monotonic()
        t = now - self.t0
        stream = tag in ("$S", "$N", "$P", "$R", "$F")
        if tag == "$S" and len(p) > len(S_FIELDS):
            self.s = dict(zip(S_FIELDS, p[1:]))
            self.d_time["S"] = now
            if not self.pause_btn.isChecked():
                cont = [to_int(self.s[f"ads{i}"]) / DIV / 1000 for i in range(4)]
                self.ch_pwr.add(t, [to_int(self.s["vbat_mv"]) / 1000, to_int(self.s["curr_ma"]) / 1000])
                self.ch_cont.add(t, cont)
        elif tag == "$N" and len(p) > len(N_FIELDS):
            self.nav = dict(zip(N_FIELDS, p[1:]))
            self.d_time["N"] = now
            if not self.pause_btn.isChecked():
                n = self.nav
                self.ch_alt.add(t, [to_float(n["alt"]), to_float(n["baro_alt"])])
                self.ch_vel.add(t, [to_float(n["vz"])])
                self.ch_acc.add(t, [to_float(n["acc_axial"]), to_float(n["acc_mag"])])
                self.ch_tilt.add(t, [to_float(n["tilt"]), self._pval("max_tilt_deg", 20.0)])
                self.ch_pos.add(t, [to_float(n["px"]), to_float(n["py"])])
                self.ch_mach.add(t, [to_float(n["mach"]), to_float(n["sig_alt"])])
        elif tag == "$P" and len(p) > len(P_FIELDS) - 2:
            self.pos = dict(zip(P_FIELDS, p[1:]))
            self.d_time["P"] = now
        elif tag == "$R" and len(p) > len(R_FIELDS):
            self.raw = dict(zip(R_FIELDS, p[1:]))
            self.d_time["R"] = now
            if not self.pause_btn.isChecked():
                r = self.raw
                self.ch_raw_acc.add(t, [to_float(r[k]) for k in ("ax", "ay", "az")])
                self.ch_raw_gyr.add(t, [to_float(r[k]) for k in ("gx", "gy", "gz")])
                self.ch_raw_mag.add(t, [to_float(r[k]) for k in ("mx", "my", "mz")])
            self._refresh_raw()
        elif tag == "$F" and len(p) > len(F_FIELDS):
            self.fl = dict(zip(F_FIELDS, p[1:]))
            self.d_time["F"] = now
        elif tag == "$PRM" and len(p) >= 8:
            self._prm(p)
            return
        elif tag == "$ID":
            self.fw_info = ",".join(p[1:])
        elif tag == "$EV":
            self.events.appendPlainText(f"{to_int(p[1]) / 1000:9.3f} s  {','.join(p[2:])}")
        elif tag == "$LOGBEGIN":
            self.log_lines = []
            self.log_prog.setMaximum(max(1, to_int(p[1] if len(p) > 1 else "0")))
            self.log_prog.setValue(0)
            self.log_info.setText(f"stahuji {p[1] if len(p) > 1 else '?'} záznamů…")
        elif tag == "$LOG" and len(p) == 2:
            self.log_lines.append(p[1])
            if len(self.log_lines) % 200 == 0:
                self.log_prog.setValue(len(self.log_lines))
            return
        elif tag == "$LOGEND":
            self.dumping = False
            self.log_prog.setValue(len(self.log_lines))
            self.save_log()
        elif tag == "$LOGINFO" and len(p) >= 6:
            self.log_info.setText(f"záznamů {p[1]} z {p[2]} · zahozeno {p[3]} · "
                                  f"{'probíhá mazání · ' if to_int(p[4]) else ''}flash JEDEC {to_int(p[5]):06X}")
        elif s.startswith("#OK PARAM LIST"):
            self.param_edits.clear()
            self._param_fill()
        elif s.startswith("#OK DIAG") or s.startswith("#ERR DIAG"):
            pass
        if stream:
            if self.show_stream.isChecked():
                self.log(s)
        else:
            self.log(s)

    def _prm(self, p: list[str]) -> None:
        idx, key = to_int(p[1]), p[2]
        desc = ",".join(p[7:]).strip()
        m = re.search(r"\[([^\]]+)\]", desc)
        unit = m.group(1).replace("deg", "°") if m else ""
        entry = {"id": idx, "key": key, "val": to_float(p[3], 0.0), "lo": to_float(p[4], 0.0),
                 "hi": to_float(p[5], 0.0), "def": to_float(p[6], 0.0), "unit": unit, "desc": desc}
        if key in self.param_rows:
            self.params[self.param_rows[key]].update(entry)
            self.param_edits.pop(key, None)
            self._param_show(key, entry["val"])
        else:
            self.params.append(entry)
            self.param_rows[key] = len(self.params) - 1
            self._fill_timer.start()          # one rebuild after the whole PARAM LIST burst

    # ------------------------------------------------------------ refresh
    def _fresh(self, k: str) -> bool:
        limit = {"P": 1.5, "F": 3.0}.get(k, 1.0)
        return self.reader is not None and time.monotonic() - self.d_time[k] < limit

    def _refresh_diag(self) -> None:
        S = self.s if self._fresh("S") else {}
        N = self.nav if self._fresh("N") else {}
        P = self.pos if self._fresh("P") else {}
        F = self.fl if self._fresh("F") else {}
        problems: list[tuple[str, str]] = []
        st = state_name(to_int(S.get("state"))) if S else ""
        vmin = self._pval("vbat_min_v", 7.2)
        maxt = self._pval("max_tilt_deg", 20.0)

        # ---- header chips
        if not self.is_open:
            for c_, n in ((self.chip_board, "Deska"), (self.chip_pos, "Position"), (self.chip_rampa, "Rampa"),
                          (self.chip_rbf, "RBF"), (self.chip_bat, "Baterie"), (self.chip_amp, "Proud"),
                          (self.chip_rail, "Pyro lišta")):
                c_.set("", n)
        elif not S:
            self.chip_board.set("bad", "Deska: mlčí")
        if S:
            flags, health, pyro = to_int(S["flags"]), to_int(S["health"]), to_int(S["pyro"])
            vb = to_int(S["vbat_mv"]) / 1000
            rs, rf = to_int(S["rampa_state"]), to_int(S["rampa_flags"])
            self.chip_board.set("ok", "Deska: OK")
            self.chip_pos.set("ok" if health & HL_POS else "bad", "Position: OK" if health & HL_POS else "Position: MLČÍ")
            rname = RAMPA_STATES[rs] if 0 <= rs < len(RAMPA_STATES) else str(rs)
            self.chip_rampa.set("ok" if rf & 0x80 else ("warn" if rs else ""), f"Rampa: {rname}")
            rbf = to_int(S["rbf"])
            self.chip_rbf.set("warn" if rbf else "ok", "RBF: vytažen" if rbf else "RBF: zasunut")
            self.chip_bat.set("ok" if vb >= vmin else ("bad" if vb > 0.5 else ""), f"Baterie: {fmt(vb, 2, 'V')}")
            self.chip_amp.set("", f"Proud: {fmt(to_int(S['curr_ma']) / 1000, 2, 'A')}")
            self.chip_rail.set("hot" if pyro & 0x40 else "", "Pyro lišta: ZAPNUTA" if pyro & 0x40 else "Pyro lišta: vyp.")

        # ---- diag bar
        if not self.is_open:
            self.chip_state.set("", "Stav: odpojeno")
        elif not S:
            self.chip_state.set("warn", "Stav: —  (žádné $S řádky: jiný firmware?)")
        else:
            self.chip_state.set("hot" if st in ("READY", "COUNTDOWN") else
                                ("bad" if st == "ABORT" else ("warn" if st == "DIAG" else "ok")),
                                f"Stav: {st} · {STATE_CZ.get(st, '')}")
        fwm = self.fw_info.split(",")[1] if self.fw_info.count(",") >= 1 else ""
        fwp = P.get("fw", "") if P else ""
        if fwm or fwp:
            self.chip_fw.set("ok" if fwm and fwp and to_float(fwm) == to_float(fwp) else "warn",
                             f"Firmware M:{fwm or '?'} / P:{fwp or '?'}")
            if fwm and fwp and to_float(fwm) != to_float(fwp):
                problems.append((WARN, f"Master ({fwm}) a Position ({fwp}) mají různé verze firmwaru."))
        else:
            self.chip_fw.set("", "Firmware: —")
        if self.params:
            nch = sum(1 for q in self.params if self._param_changed(q) and q["key"] not in self.param_edits)
            ned = len(self.param_edits)
            self.chip_sync.set("warn" if ned else "ok", f"Parametry: {len(self.params)} načteno"
                               + (f" · {nch} jiné než výchozí" if nch else "") + (f" · {ned} k odeslání" if ned else ""))
            if ned:
                problems.append((WARN, f"{ned} upravených parametrů není odeslaných."))
        else:
            self.chip_sync.set("", "Parametry: —")
        self.diag_btn.setChecked(st == "DIAG")
        self.diag_btn.setEnabled(self.is_open and st in ("IDLE", "ABORT", "DIAG"))
        self.prep_btn.setEnabled(self.is_open and st == "IDLE")
        self.cd_btn.setEnabled(self.is_open and st == "READY")

        # ---- Master
        kv = self.board_kv["M"]
        if S:
            err = to_int(S["err"])
            kv.set("Online", "ano", GOOD)
            kv.set("Firmware", f"{fwm or '?'}  ({self.fw_info.split(',')[2] if self.fw_info.count(',') >= 2 else '?'} "
                               "parametrů)" if self.fw_info else "— (čeká na $ID)")
            kv.set("Stav", f"{st}  {STATE_CZ.get(st, '')}", BAD if st == "ABORT" else None)
            kv.set("Chyba", "žádná" if err == 0 else err_name(err), GOOD if err == 0 else BAD)
            kv.set("Baterie", fmt(vb, 2, "V") + f"  (min. {fmt(vmin, 1, 'V')})",
                   BAD if 0.5 < vb < vmin else (WARN if vb < vmin + 0.3 else GOOD))
            kv.set("Proud", fmt(to_int(S["curr_ma"]) / 1000, 2, "A"))
            umb = to_int(S["umb"])
            kv.set("Napájení", f"pogo 5 V {'ANO' if umb else 'ne'} ({fmt(to_int(S['umb_mv']) * 2 / 1000, 2, 'V')})"
                               f" · {'z baterie (FORCE_BAT)' if flags & TF_ON_BATT else 'z pogo/USB'}")
            kv.set("RBF", "vytažen (lištu lze zapnout)" if to_int(S["rbf"]) else "zasunut (bezpečno)",
                   WARN if to_int(S["rbf"]) else GOOD)
            bw1, bw2 = to_int(S["bw1"]), to_int(S["bw2"])
            kv.set("Breakwire", f"1 {'celý' if bw1 else 'PŘERUŠEN'} · 2 {'celý' if bw2 else 'PŘERUŠEN'}",
                   GOOD if bw1 and bw2 else WARN)
            rail = to_int(S["ads3"]) / DIV / 1000
            kv.set("Pyro lišta", f"{'ZAPNUTA' if pyro & 0x40 else 'vypnuta'} ({fmt(rail, 2, 'V')})",
                   BAD if pyro & 0x40 else GOOD)
            kv.set("Periferie I²C", "OK (INA226, ADS1115, PCA9685)" if health & HL_HW else "NEODPOVÍDAJÍ",
                   GOOD if health & HL_HW else BAD)
            kv.set("Flash log", (f"OK, {S['log_head']} záznamů, zahozeno {S['log_dropped']}"
                                 + (" · MAŽE SE" if to_int(S["log_erasing"]) else "")) if to_int(S["flash"]) else "CHYBÍ",
                   GOOD if to_int(S["flash"]) else BAD)
            kv.set("LoRa", "OK" if to_int(S["lora"]) else "NEODPOVÍDÁ", GOOD if to_int(S["lora"]) else BAD)
            kv.set("Spoj Position", "OK" if health & HL_POS else "MRTVÝ", GOOD if health & HL_POS else BAD)
            kv.set("Kamera", "napájena" if health & HL_CAM else "vypnuta")
            kv.set("Lokátor", "BZUČÍ" if health & HL_LOCATE else "vyp.", WARN if health & HL_LOCATE else None)
            kv.set("Restart", "watchdog" if to_int(S["reset"]) else "zapnutí", WARN if to_int(S["reset"]) else None)

            # Go / No-Go (the same checks as the firmware, worded for people)
            if err:
                problems.append((BAD, ("Přerušeno: " if st == "ABORT" else "NO-GO: ") + err_name(err)))
            if not health & HL_POS:
                problems.append((BAD, "Master neslyší Position MCU."))
            if not health & HL_NAV:
                problems.append((BAD, "Navigační řešení NENÍ platné – nelze startovat."))
            if not health & HL_HW:
                problems.append((BAD, "I²C periferie (INA226 / ADS1115 / PCA9685) neodpovídají."))
            if 0.5 < vb < vmin:
                problems.append((BAD, f"Baterie {vb:.2f} V je pod vbat_min_v {vmin:.1f} V – nabijte (2S, plná 8,4 V)."))
            elif 0.5 < vb < vmin + 0.3:
                problems.append((WARN, f"Baterie {vb:.2f} V – blízko limitu {vmin:.1f} V."))
            for i, nm in enumerate(("PYRO1 (drogue)", "PYRO2 (main)", "PYRO3 (záloha)")):
                if pyro & (8 << i):
                    continue
                if not pyro & (1 << i):
                    req = i < 2 or self._pval("req_cont3", 0) > 0.5
                    problems.append((BAD if req else WARN, f"Bez kontinuity: {nm}."))
            if not to_int(S["rbf"]):
                problems.append((WARN, "RBF zasunut – pyro lištu nelze zapnout (před startem vytáhnout)."))
            if not to_int(S["flash"]):
                problems.append((BAD, "Flash log nefunguje – let nebude zaznamenán."))
            if not to_int(S["lora"]):
                problems.append((BAD, "LoRa nefunguje – žádná telemetrie ani příkazy ze země."))
            if not health & HL_GPS:
                problems.append((WARN, "GPS bez 3D fixu."))
            if self._pval("bw_mode", 1) > 0 and not (bw1 or bw2):
                problems.append((WARN, "Oba breakwiry přerušené."))
            if to_int(S["reset"]):
                problems.append((WARN, "Master se naposledy restartoval watchdogem."))
            if pyro & 0x40 and st == "DIAG":
                problems.append((WARN, "Pyro lišta je ZAPNUTÁ (DIAG test)."))
        else:
            kv.clear()
            if self.is_open:
                problems.append((WARN, "Master neposílá stavové řádky ($S)."))

        # ---- Position
        kv = self.board_kv["P"]
        nose = to_int(P.get("nose"), 1) if P else 1
        if P and S and to_int(S["health"]) & HL_POS:
            sf = to_int(P["sensors"])
            kv.set("Online", "ano", GOOD)
            kv.set("Firmware", P.get("fw", "?"))
            kv.set("Senzory", " ".join(n for b, n in SENS_BITS if sf & (1 << b)) or "—",
                   GOOD if sf & 0x0F == 0x0F else WARN)
            nf = to_int(N.get("flags")) if N else 0
            md = to_int(N.get("mode")) if N else -1
            kv.set("Navigace", (NAV_MODES[md] if 0 <= md < len(NAV_MODES) else "—")
                   + (" · platná" if nf & 1 else " · NEPLATNÁ"), GOOD if nf & 1 else BAD)
            kv.set("Orientace", "z gravitace + magnetometru" if nf & (1 << 4) else "neinicializována",
                   GOOD if nf & (1 << 4) else WARN)
            pna = int(self._pval("nose_axis", 0))
            kv.set("Osa špičky", f"{AXES[min(6, max(0, nose))]}  (nose_axis = {AXES[min(6, max(0, pna))]})")
            hz = to_int(P["imu_hz"])
            kv.set("IMU Hz", f"{hz} Hz", GOOD if hz > 800 else WARN)
            kv.set("IMU ztracené", P["imu_lost"], WARN if to_int(P["imu_lost"]) else GOOD)
            kv.set("Špička g", fmt(to_float(P["peak_g"]), 1, "g"))
            kv.set("Bias gyra", f"{P['gb0']} / {P['gb1']} / {P['gb2']} °/s")
            fix = to_int(P["fix"])
            kv.set("GPS", f"fix {fix}D, {P['sats']} družic, hAcc {P['hacc']} m", GOOD if fix >= 3 else WARN)
            kv.set("Restart", "watchdog" if to_int(P["reset"]) else "zapnutí", WARN if to_int(P["reset"]) else None)
            for bit, name, sev in ((0, "IMU LSM6DSV32X", BAD), (1, "high-g IMU LSM6DSV320X", WARN),
                                   (2, "barometr BMP580", BAD), (3, "magnetometr MMC5983MA", WARN),
                                   (4, "GPS", WARN)):
                if not sf & (1 << bit):
                    problems.append((sev, f"Position MCU: {name} nefunguje."))
            if to_int(P["imu_lost"]):
                problems.append((WARN, f"Position MCU ztratil {P['imu_lost']} vzorků IMU."))
        else:
            kv.clear()
        zw.set_nose_axis(nose)

        # ---- rampa
        kv = self.board_kv["R"]
        if S:
            rs, rf, rflt = to_int(S["rampa_state"]), to_int(S["rampa_flags"]), to_int(S["rampa_faults"])
            rname = RAMPA_STATES[rs] if 0 <= rs < len(RAMPA_STATES) else str(rs)
            kv.set("Spojení", "OK (pogo UART)" if rs else "NEKOMUNIKUJE", GOOD if rs else WARN)
            kv.set("Stav", rname, BAD if rname in ("ABORT", "FAULT") else None)
            if rs:
                kv.set("Klíč", "ZAPNUT" if rf & 1 else "vypnut", WARN if rf & 1 else None)
                kv.set("Rozbušky", "  ".join(f"K{i + 1}:{'OK' if rf & (2 << i) else '—'}" for i in range(3)),
                       GOOD if rf & (2 << (int(self._pval("rampa_ch", 1)) - 1)) else WARN)
                kv.set("Pyro rampy", "ZAPNUTO" if rf & 0x10 else "vypnuto", BAD if rf & 0x10 else None)
                kv.set("Baterie rampy", fmt(to_int(S["rampa_vbat"]) / 1000, 2, "V"))
                kv.set("LoRa rampy", "vypnuta (mluví raketa)" if rf & 0x20 else "ZAPNUTA")
                kv.set("Napájení rakety", "zapnuto" if rf & 0x40 else "vypnuto")
                kv.set("Připravena", "ANO" if rf & 0x80 else "ne", GOOD if rf & 0x80 else WARN)
                kv.set("Chyby rampy", ", ".join(bits(rflt, RAMPA_FAULTS)) or "žádné", BAD if rflt else GOOD)
            else:
                for n in ("Klíč", "Rozbušky", "Pyro rampy", "Baterie rampy", "LoRa rampy", "Napájení rakety",
                          "Připravena", "Chyby rampy"):
                    kv.set(n, "—", DIM)
            trem = to_int(S["t_rem"], -1)
            kv.set("Odpočet", f"T−{trem / 1000:.1f} s" if trem >= 0 else "—", BAD if trem >= 0 else None)
            if not rs and self._pval("ext_ign", 0) < 0.5:
                problems.append((WARN, "Rampa nekomunikuje (pogo) – start z rampy nepůjde."))
            elif rs and not rf & 0x80 and st in ("READY", "COUNTDOWN"):
                problems.append((BAD, "Rampa není připravena (klíč, rozbuška, baterie)."))
            if rflt:
                problems.append((BAD, "Rampa hlásí: " + ", ".join(bits(rflt, RAMPA_FAULTS)) + "."))
        else:
            kv.clear()

        # ---- attitude
        tilt_ok, tilt_warn = maxt / 2, maxt
        self.rocket.tilt_ok, self.rocket.tilt_warn = tilt_ok, tilt_warn
        self.bubble.tilt_ok, self.bubble.tilt_warn, self.bubble.gate_deg = tilt_ok, tilt_warn, maxt
        if N:
            q = tuple(to_float(N[k]) for k in ("qw", "qx", "qy", "qz"))
            if all(math.isfinite(x) for x in q) and any(q):
                a = attitude_summary(*q)
                self.rocket.set_quaternion(*q)
                self.rocket.set_actuators(burning=st == "BOOST")
                self.bubble.set_tilt(a["tilt"], a["tilt_az"], True)
                self.att_kv.set("Náklon", f"{a['tilt']:.2f}°  ({'špička nahoru' if a['nose_up'] else 'špička dolů'}"
                                f", limit {maxt:.0f}°)", GOOD if a["tilt"] < tilt_ok else
                                (WARN if a["tilt"] < tilt_warn else BAD))
                self.att_kv.set("Směr náklonu", f"{a['tilt_az']:.0f}° (azimut, kam se naklání vršek)")
                self.att_kv.set("Roll", f"{a['roll']:.0f}°  (kam míří referenční osa)")
                self.att_kv.set("Pitch / heading", f"{a['pitch']:.1f}° / {a['heading']:.0f}°")
                if a["tilt"] > maxt and st in ("IDLE", "READY", "COUNTDOWN"):
                    problems.append((BAD, f"Raketa je nakloněná {a['tilt']:.1f}° (limit max_tilt_deg {maxt:.0f}°)."))
            else:
                self.rocket.set_quaternion(1, 0, 0, 0, valid=False)
                self.bubble.set_tilt(0, 0, False)
        else:
            self.rocket.set_quaternion(1, 0, 0, 0, valid=False)
            self.bubble.set_tilt(0, 0, False)
            for n in ("Náklon", "Směr náklonu", "Roll", "Pitch / heading"):
                self.att_kv.set(n, "—", DIM)
        if P:
            self.att_kv.set("Euler (Position)", f"roll {to_float(P['roll']):.1f}° · pitch {to_float(P['pitch']):.1f}° · "
                                                f"yaw {to_float(P['yaw']):.0f}°")
            self.att_kv.set("Osa špičky", f"{AXES[min(6, max(0, nose))]} (osy desky Position)")
        else:
            self.att_kv.set("Euler (Position)", "—", DIM)
            self.att_kv.set("Osa špičky", "—", DIM)

        # ---- flight
        fk = self.flight_kv
        if S:
            pyro = to_int(S["pyro"])
            fk.set("Stav", f"{st}  {STATE_CZ.get(st, '')}")
            trem = to_int(S["t_rem"], -1)
            fk.set("Do startu", f"T−{trem / 1000:.1f} s" if trem >= 0 and st == "COUNTDOWN" else "—")
            fk.set("Hlavní padák v", f"{S['main_alt']} m nad rampou  (parametr main_alt_m)")
            fk.set("Letový log", f"{S['log_head']} záznamů, zahozeno {S['log_dropped']}")
            ev = 0
            nf = to_int(N.get("flags")) if N else 0
            if nf & (1 << 5) or st in ("BOOST", "COAST", "DROGUE", "MAIN", "LANDED"):
                ev |= 1
            if nf & (1 << 6) or st in ("COAST", "DROGUE", "MAIN", "LANDED"):
                ev |= 2
            if nf & (1 << 7) or st in ("DROGUE", "MAIN", "LANDED"):
                ev |= 4
            ev |= ((pyro >> 3) & 7) << 3
            if st == "LANDED":
                ev |= 64
            self.ev_flags.set(ev)
        else:
            for n in ("Stav", "Do startu", "Hlavní padák v", "Letový log"):
                fk.set(n, "—", DIM)
            self.ev_flags.set(None)
        if N:
            fk.set("Výška / max", f"{fmt(to_float(N['alt']), 1, 'm')} / {fmt(to_float(N['max_alt']), 1, 'm')}")
            fk.set("Svislá rychlost", fmt(to_float(N["vz"]), 1, "m/s", sign=True))
            fk.set("Mach", fmt(to_float(N["mach"]), 2) + ("  TRANSONICKÉ PÁSMO" if to_int(N["flags"]) & (1 << 9) else ""))
            fk.set("Zrychlení osové", fmt(to_float(N["acc_axial"]), 2, "g"))
        else:
            for n in ("Výška / max", "Svislá rychlost", "Mach", "Zrychlení osové"):
                fk.set(n, "—", DIM)
        if F:
            def why(k):
                w = WHY[to_int(F.get(k))] if 0 <= to_int(F.get(k)) < len(WHY) else F.get(k)
                return WHY_CZ.get(w, w)
            fired = to_int(F["fired"])
            for n, k, b in (("Drogue (PYRO1)", "why_d", 1), ("Main (PYRO2)", "why_m", 2), ("Záloha (PYRO3)", "why_p3", 4)):
                fk.set(n, ("ODPÁLENO · " if fired & b else "") + why(k), WARN if fired & b else None)
            fk.set("Poslední let", f"max {F['max_alt']} m · vz {F['max_vz']} m/s · Mach {F['max_mach']} · {F['max_g']} g")
            fk.set("Pyro událostí", F["pyro_events"])
        else:
            for n in ("Drogue (PYRO1)", "Main (PYRO2)", "Záloha (PYRO3)", "Poslední let", "Pyro událostí"):
                fk.set(n, "—", DIM)

        # ---- nav tab
        if N:
            n = N
            self.nav_kv.set("Poloha E / N / výška", f"{to_float(n['px']):+.2f} / {to_float(n['py']):+.2f} / "
                                                    f"{to_float(n['alt']):.2f} m")
            self.nav_kv.set("Rychlost E / N / svislá", f"{to_float(n['vx']):+.2f} / {to_float(n['vy']):+.2f} / "
                                                       f"{to_float(n['vz']):+.2f} m/s")
            self.nav_kv.set("Výška KF / baro", f"{to_float(n['alt']):.2f} / {to_float(n['baro_alt']):.2f} m")
            self.nav_kv.set("σ výšky", fmt(to_float(n["sig_alt"]), 2, "m"), GOOD if to_float(n["sig_alt"]) < 2 else WARN)
            self.nav_kv.set("Zrychlení osové / |a|", f"{to_float(n['acc_axial']):.3f} / {to_float(n['acc_mag']):.3f} g")
            self.nav_kv.set("Mach", fmt(to_float(n["mach"]), 2))
            self.nav_kv.set("Max. výška", fmt(to_float(n["max_alt"]), 1, "m"))
            md = to_int(n["mode"])
            self.nav_kv.set("Režim", NAV_MODES[md] if 0 <= md < len(NAV_MODES) else n["mode"])
            nf = to_int(n["flags"])
            self.nav_flags.set(nf, bad_when_off=(0, 1, 4))
            self.nav_flags2.set(nf, bad_when_off=(11, 12))
            self.padmap.set_position(to_float(n["px"]), to_float(n["py"]), 0.0)
            if not nf & (1 << 11) and st in ("IDLE", "READY", "COUNTDOWN"):
                problems.append((WARN, "Chybí reference rampy (Pad reference)."))
        else:
            self.nav_kv.clear()
            self.nav_flags.set(None)
            self.nav_flags2.set(None)
        if P:
            fix = to_int(P["fix"])
            self.sens_kv.set("GPS", f"fix {fix}D, {P['sats']} družic, hAcc {P['hacc']} m", GOOD if fix >= 3 else WARN)
            lat, lon = to_float(P["lat"]), to_float(P["lon"])
            self.sens_kv.set("Poloha (lat, lon)", f"{lat:.7f}, {lon:.7f}")
            self.sens_kv.set("GPS výška", fmt(to_float(P["galt"]), 1, "m n. m."))
            plat, plon = to_float(P.get("pad_lat")), to_float(P.get("pad_lon"))
            self.sens_kv.set("Rampa (GPS počátek)", f"{plat:.7f}, {plon:.7f}" if math.isfinite(plat) else "—")
            if math.isfinite(plat) and plat and lat:
                self.sens_kv.set("Vzdálenost od rampy", fmt(haversine_m(plat, plon, lat, lon), 1, "m"))
            else:
                self.sens_kv.set("Vzdálenost od rampy", "—", DIM)
            self.sens_kv.set("Barometr", fmt(to_float(P["pressure"]), 1, "Pa"))
            self.sens_kv.set("Teplota", fmt(to_float(P["temp"]), 1, "°C"))
            self.sens_kv.set("Bias gyra", f"{P['gb0']} / {P['gb1']} / {P['gb2']} °/s")
            self.sens_kv.set("IMU", f"{P['imu_hz']} Hz · ztraceno {P['imu_lost']} · špička {P['peak_g']} g")
            self.sens_kv.set("Restart Position", "watchdog" if to_int(P["reset"]) else "zapnutí")
            self.sens_flags.set(to_int(P["sensors"]), bad_when_off=(0, 2))
        else:
            self.sens_kv.clear()
            self.sens_flags.set(None)

        # ---- actuators tab
        if S:
            pyro, flags, health = to_int(S["pyro"]), to_int(S["flags"]), to_int(S["health"])
            diag = st == "DIAG"
            rail = bool(pyro & 0x40)
            self.arm_chip.set("hot" if rail else "", "LIŠTA ZAPNUTA" if rail else "LIŠTA VYPNUTA")
            self.arm_btn.setEnabled(diag and not rail and bool(to_int(S["rbf"])))
            self.disarm_btn.setEnabled(diag and rail)
            if not diag:
                self.pyro_note.setText("Testy pyro, FORCE_BAT a napájení serv jen v DIAG režimu (tlačítko „DIAG režim“ "
                                       "nahoře, z IDLE). Mimo DIAG ukazují ukazatele jen kontinuitu.")
            elif not to_int(S["rbf"]):
                self.pyro_note.setText("RBF je zasunutý: lištu nelze zapnout.")
            else:
                self.pyro_note.setText("DIAG režim: lištu lze zapnout a kanál odpálit podržením tlačítka.")
            thr = self._pval("cont_min_mv", 300) / 1000
            for i, row in enumerate(self.pyro_rows):
                mv = to_int(S[f"ads{i}"])
                ok, fired = bool(pyro & (1 << i)), bool(pyro & (8 << i))
                row["cont"].set("ok" if ok else "bad", "kontinuita OK" if ok else "BEZ KONTINUITY")
                row["fired"].setVisible(fired)
                row["fired"].set("warn", "VYPÁLEN")
                row["raw"].setText(f"ADC {mv} mV · {fmt(mv / DIV / 1000, 2, 'V')} (práh {fmt(thr, 2, 'V')} na pinu)")
                row["fire"].setEnabled(diag and rail)
            self.act_kv.set("Napájení", "pogo 5 V" if to_int(S["umb"]) else "bez pogo")
            self.act_kv.set("Baterie", f"{fmt(to_int(S['vbat_mv']) / 1000, 2, 'V')} · {fmt(to_int(S['curr_ma']) / 1000, 2, 'A')}")
            self.act_kv.set("FORCE_BAT", "ZAP (z baterie)" if flags & TF_ON_BATT else "vyp.",
                            WARN if flags & TF_ON_BATT else None)
            self.act_kv.set("Kamera", "napájena" if health & HL_CAM else "vypnuta")
            self.act_kv.set("Lokátor", "BZUČÍ" if health & HL_LOCATE else "vyp.")
            rs = to_int(S["rampa_state"])
            self.act_kv.set("Rampa", RAMPA_STATES[rs] if 0 <= rs < len(RAMPA_STATES) else str(rs))
            for cmd in ("FORCEBAT 1", "FORCEBAT 0", "SERVOPWR 1", "SERVOPWR 0"):
                self.act_btns[cmd].setEnabled(diag)
        else:
            self.arm_chip.set("", "LIŠTA —")
            self.arm_btn.setEnabled(False)
            self.disarm_btn.setEnabled(False)
            for row in self.pyro_rows:
                row["cont"].set("", "—")
                row["fired"].setVisible(False)
                row["raw"].setText("ADC —")
                row["fire"].setEnabled(False)
            self.act_kv.clear()
            self.pyro_note.setText("")

        # ---- problems list
        self.problems.clear()
        if not self.is_open:
            problems = [(DIM, "Připojte Master USB kabelem a zvolte port (nebo Simulace).")]
        elif not S and not self.fw_info:
            problems.insert(0, (WARN, "Deska neodpověděla jako letový firmware Zenithu ($ID). Je to správný port?"))
        go = bool(S) and bool(to_int(S["flags"]) & TF_GO)
        if self.is_open and S:
            self.chip_go.setVisible(True)
            self.chip_go.set("ok" if go else "bad", "GO" if go else "NO-GO")
        else:
            self.chip_go.setVisible(False)
        if not problems:
            problems = [(GOOD, "Vše v pořádku – GO.")]
        order = {BAD: 0, WARN: 1, GOOD: 2, DIM: 3}
        problems.sort(key=lambda x: order.get(x[0], 9))
        nbad = sum(1 for c_, _ in problems if c_ == BAD)
        nwarn = sum(1 for c_, _ in problems if c_ == WARN)
        for col, txt in problems:
            it = QtWidgets.QListWidgetItem(("✖  " if col == BAD else "▲  " if col == WARN else "✔  ") + txt)
            it.setForeground(QtGui.QColor(col))
            self.problems.addItem(it)
        self.chip_err.set("bad" if nbad else ("warn" if nwarn else ("ok" if self.is_open else "")),
                          f"Chyby: {nbad} · varování: {nwarn}")

    def closeEvent(self, ev):
        if self.rec_file:
            self.rec_file.close()
        if self.reader is not None and self._state() == "DIAG":
            self.reader.send("ARM 0")
        self.stop_reader()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=APP_NAME)
    ap.add_argument("--port", help="sériový port Masteru, např. COM5 nebo /dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="simulovaný Master, bez hardwaru")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    ap.add_argument("--tab", type=int, default=0, help=argparse.SUPPRESS)
    a = ap.parse_args()
    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    vs.setup_app(app, "VoltinoLabs.Zenith.Diagnostika")
    w = DiagWindow(a.port, a.demo)
    w.show()
    if a.screenshot:
        w.tabs.setCurrentIndex(a.tab)
        QtCore.QTimer.singleShot(3000, lambda: (w.grab().save(a.screenshot), app.quit()))
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
