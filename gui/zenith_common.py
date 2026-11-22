"""Shared pieces of the Zenith GUIs: protocol constants (mirror of
firmware/common/zenith_link.h), a serial line reader thread, widgets and theme."""
from __future__ import annotations

import math
import time
from collections import deque

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # the GUIs still start (demo mode) without pyserial
    serial = None

# ---------------------------------------------------------------- protocol
STATES = ["BOOT", "IDLE", "READY", "COUNTDOWN", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED", "ABORT", "DIAG"]
STATES_CZ = {
    "BOOT": "Start", "IDLE": "Na rampě – bezpečno", "READY": "Připraveno ke startu", "COUNTDOWN": "Odpočet",
    "BOOST": "Motor hoří", "COAST": "Setrvačný let", "DROGUE": "Brzdicí padák", "MAIN": "Hlavní padák",
    "LANDED": "Přistáno", "ABORT": "PŘERUŠENO", "DIAG": "Diagnostika",
}
FLIGHT_STATES = {"BOOST", "COAST", "DROGUE", "MAIN"}

ERRORS = [
    "OK", "Nekomunikuje Position MCU", "Navigace neplatná", "Rampa nekomunikuje", "Rampa není připravena",
    "Nízké napětí baterie", "PYRO1 (drogue) bez kontinuity", "PYRO2 (main) bez kontinuity",
    "PYRO3 (záloha) bez kontinuity", "RBF zasunut (SAFE)", "Chybí flash", "I2C periferie (INA/ADS/PCA)",
    "Přepnutí na baterii selhalo", "Pyro lišta se nezapnula", "Rampa nepotvrdila COMMIT", "HANGFIRE – motor nezažehl",
    "Přerušeno ze země", "Přerušeno rampou", "Raketa příliš nakloněná",
]
RAMPA_STATES = ["OFFLINE", "IDLE", "READY", "COUNTDOWN", "COMMITTED", "FIRED", "ABORT", "FAULT"]
RAMPA_FAULTS = ["nízká baterie", "klíč vypnut", "rozbuška bez kontinuity", "timeout rakety", "raketa NO-GO",
                "HW rampy", "tlačítko ABORT", "chyba časování"]
UL_RC = ["–", "OK", "odmítnuto: v letu", "odmítnuto: špatný stav", "odmítnuto: rampa", "odmítnuto: token/argument",
         "PŘERUŠENO", "odmítnuto: NO-GO"]
UL_CMDS = ["NOP", "PREPARE", "COUNTDOWN", "ABORT", "SAFE", "PADREF", "MAINALT", "LOCATE", "CAMERA", "REBOOT", "ROCKETPWR"]

NAV_FLAGS = ["VALID", "BARO_OK", "BARO_TRUST", "GPS_FUSED", "ATT_OK", "LIFTOFF", "BURNOUT", "APO_KF", "APO_BARO",
             "TRANSONIC", "HIGHG", "PAD_REF", "IMU_OK", "MAG_OK", "GPS_FIX", "STILL"]
NAV_MODES = ["INIT", "PAD", "FLIGHT", "BARO_ONLY"]
SENS_FLAGS = ["LSM6DSV32X", "LSM6DSV320X", "BMP580", "MMC5983MA", "GPS", "GPS_CFG"]
TF = ["ARMED", "RBF_OUT", "BW1", "BW2", "UMB", "ON_BATT", "LOGGING", "GO"]
HL = ["POS_LINK", "NAV_VALID", "BARO_TRUST", "GPS_FIX", "FLASH", "HW", "CAM", "LOCATE"]
WHY = ["-", "APO_KF", "APO_BARO", "APO_VZ", "APO_TIMER_NAVLOST", "APO_TIMER_MAX", "MAIN_ALT", "MAIN_TIMER",
       "MAIN_LOW_APOGEE", "P3_DROGUE_FAIL", "P3_MAIN_FAIL", "P3_BLIND"]


def bits(value: int, names: list[str]) -> list[str]:
    return [n for i, n in enumerate(names) if value & (1 << i)]


def state_name(i: int) -> str:
    return STATES[i] if 0 <= i < len(STATES) else f"?{i}"


def err_name(i: int) -> str:
    return ERRORS[i] if 0 <= i < len(ERRORS) else f"chyba {i}"


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


# ---------------------------------------------------------------- theme
PALETTE = {
    "bg": "#12161c", "panel": "#1b2129", "panel2": "#222a34", "text": "#e6edf3", "muted": "#8b98a5",
    "ok": "#2ea043", "warn": "#d29922", "bad": "#f85149", "info": "#388bfd", "accent": "#a371f7",
    "grid": "#2b333d", "dim": "#3a434d",
}

STYLE = f"""
QWidget {{ background: {PALETTE['bg']}; color: {PALETTE['text']}; font-size: 13px; }}
QGroupBox {{ background: {PALETTE['panel']}; border: 1px solid #2d3640; border-radius: 8px; margin-top: 14px; padding: 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {PALETTE['muted']}; font-weight: bold; }}
QPushButton {{ background: {PALETTE['panel2']}; border: 1px solid #36414d; border-radius: 6px; padding: 6px 12px; }}
QPushButton:hover {{ border-color: {PALETTE['info']}; }}
QPushButton:disabled {{ color: #555; }}
QPushButton#abort {{ background: #8b1a1a; border: 2px solid {PALETTE['bad']}; font-size: 20px; font-weight: bold; padding: 14px; }}
QPushButton#abort:hover {{ background: #b22222; }}
QPushButton#go {{ background: #1f4d2b; border: 1px solid {PALETTE['ok']}; font-weight: bold; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTableWidget {{
    background: {PALETTE['panel2']}; border: 1px solid #36414d; border-radius: 4px; padding: 3px; }}
QHeaderView::section {{ background: {PALETTE['panel']}; color: {PALETTE['muted']}; border: none; padding: 4px; }}
QTabBar::tab {{ background: {PALETTE['panel']}; padding: 8px 14px; border-top-left-radius: 6px; border-top-right-radius: 6px; }}
QTabBar::tab:selected {{ background: {PALETTE['panel2']}; color: white; }}
QTabWidget::pane {{ border: 1px solid #2d3640; }}
"""


def label(text: str = "", size: int = 13, bold: bool = False, color: str | None = None) -> QtWidgets.QLabel:
    lb = QtWidgets.QLabel(text)
    lb.setProperty("zsize", size)
    lb.setProperty("zbold", bold)
    set_label_style(lb, color)
    return lb


LABEL_SCALE = 1.3   # px per "size" unit; the VoltinoLabs theme sets 1.0


def set_label_style(lb: QtWidgets.QLabel, color: str | None = None):
    """Font size/weight live in the style sheet (the global sheet would override a QFont)."""
    size = lb.property("zsize") or 13
    bold = lb.property("zbold") or False
    css = f"font-size: {int(size * LABEL_SCALE)}px; background: transparent;"
    if bold:
        css += " font-weight: bold;"
    if color:
        css += f" color: {color};"
    lb.setStyleSheet(css)


class Chip(QtWidgets.QLabel):
    """Small coloured status indicator."""

    def __init__(self, text: str):
        super().__init__(text)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(70)
        self.set(None)

    def set(self, state: bool | None | str, text: str | None = None):
        if text is not None:
            self.setText(text)
        col = {True: PALETTE["ok"], False: PALETTE["bad"], None: "#3a434d", "warn": PALETTE["warn"],
               "info": PALETTE["info"]}.get(state, "#3a434d")
        self.setStyleSheet(f"background: {col}; color: white; border-radius: 9px; padding: 3px 8px; font-weight: bold;")


class StripChart(QtWidgets.QWidget):
    """Rolling time chart with several series, grid and axis labels."""

    def __init__(self, title: str, series: list[tuple[str, str]], span_s: float = 60.0, unit: str = ""):
        super().__init__()
        self.title, self.series, self.span, self.unit = title, series, span_s, unit
        self.data = {name: deque(maxlen=6000) for name, _ in series}
        self.setMinimumHeight(170)

    def add(self, t: float, **values):
        for k, v in values.items():
            if k in self.data and v is not None and math.isfinite(v):
                self.data[k].append((t, v))

    def clear(self):
        for d in self.data.values():
            d.clear()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QtGui.QColor(PALETTE["panel"]))
        l, r, t, b = 52, 10, 22, 22
        pw, ph = w - l - r, h - t - b
        p.setPen(QtGui.QColor(PALETTE["muted"]))
        p.drawText(8, 15, f"{self.title} [{self.unit}]" if self.unit else self.title)
        now = max((d[-1][0] for d in self.data.values() if d), default=0.0)
        t0 = now - self.span
        vals = [v for d in self.data.values() for (tt, v) in d if tt >= t0]
        if not vals:
            p.drawText(l + 10, t + ph // 2, "žádná data")
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            hi, lo = hi + 1, lo - 1
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        grid = QtGui.QPen(QtGui.QColor(PALETTE["grid"]))
        for i in range(5):
            y = t + ph * i / 4
            p.setPen(grid)
            p.drawLine(l, int(y), l + pw, int(y))
            p.setPen(QtGui.QColor(PALETTE["muted"]))
            val = hi - (hi - lo) * i / 4
            p.drawText(2, int(y) + 4, f"{val:7.3f}" if hi - lo < 2 else f"{val:7.1f}" if hi - lo < 200 else f"{val:7.0f}")
        for i in range(7):
            x = l + pw * i / 6
            p.setPen(grid)
            p.drawLine(int(x), t, int(x), t + ph)
            p.setPen(QtGui.QColor(PALETTE["muted"]))
            p.drawText(int(x) - 12, h - 5, f"{-self.span + self.span * i / 6:.0f}s")
        for k, (name, col) in enumerate(self.series):
            pts = [(tt, v) for (tt, v) in self.data[name] if tt >= t0]
            if len(pts) < 2:
                continue
            path = QtGui.QPainterPath()
            for i, (tt, v) in enumerate(pts):
                x = l + (tt - t0) / self.span * pw
                y = t + (hi - v) / (hi - lo) * ph
                path.moveTo(x, y) if i == 0 else path.lineTo(x, y)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 2))
            p.drawPath(path)
            p.drawText(l + 10 + 110 * k, t + 14, f"■ {name}")


class AttitudeView(QtWidgets.QWidget):
    """Side + top view of the rocket from the body->ENU quaternion and the nose axis."""

    def __init__(self):
        super().__init__()
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.nose = 1
        self.setMinimumSize(260, 200)

    def set(self, q, nose: int):
        self.q, self.nose = q, nose
        self.update()

    def _nose_world(self):
        w, x, y, z = self.q
        v = [0.0, 0.0, 0.0]
        i = max(0, min(2, (self.nose - 1) // 2))
        v[i] = -1.0 if (self.nose - 1) % 2 else 1.0
        r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
             [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
             [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
        return [sum(r[a][b] * v[b] for b in range(3)) for a in range(3)]

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QtGui.QColor(PALETTE["panel"]))
        e, n, u = self._nose_world()
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, u))))
        az = (math.degrees(math.atan2(e, n)) + 360) % 360
        # side view (east-up plane projection along the azimuth)
        cx, cy, L = w * 0.28, h * 0.55, min(w, h) * 0.35
        horiz = math.hypot(e, n)
        p.setPen(QtGui.QPen(QtGui.QColor(PALETTE["dim"]), 1))
        p.drawLine(int(cx - L), int(cy + L * 0.9), int(cx + L), int(cy + L * 0.9))
        p.setPen(QtGui.QPen(QtGui.QColor(PALETTE["info"]), 7, cap=Qt.PenCapStyle.RoundCap))
        p.drawLine(int(cx - horiz * L * 0.8), int(cy + u * L * 0.8), int(cx + horiz * L * 0.8), int(cy - u * L * 0.8))
        p.setBrush(QtGui.QColor(PALETTE["bad"]))
        p.drawEllipse(QtCore.QPointF(cx + horiz * L * 0.8, cy - u * L * 0.8), 6, 6)
        p.setPen(QtGui.QColor(PALETTE["text"]))
        p.drawText(8, 16, f"náklon {tilt:5.1f}°   azimut {az:5.0f}°")
        # top view: tilt bull's-eye
        bx, by, R = w * 0.75, h * 0.55, min(w * 0.22, h * 0.38)
        p.setPen(QtGui.QPen(QtGui.QColor(PALETTE["dim"]), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        for k in (1, 2, 3):
            p.drawEllipse(QtCore.QPointF(bx, by), R * k / 3, R * k / 3)
        p.drawText(int(bx) - 4, int(by - R) - 4, "S")
        p.drawText(int(bx) - 4, int(by + R) + 14, "J")
        p.drawText(int(bx + R) + 4, int(by) + 4, "V")
        p.drawText(int(bx - R) - 14, int(by) + 4, "Z")
        sc = min(1.0, tilt / 30.0) * R / max(1e-6, horiz) if horiz > 1e-6 else 0
        p.setBrush(QtGui.QColor(PALETTE["ok"] if tilt < 10 else PALETTE["warn"] if tilt < 20 else PALETTE["bad"]))
        p.drawEllipse(QtCore.QPointF(bx + e * sc, by - n * sc), 7, 7)
        p.setPen(QtGui.QColor(PALETTE["muted"]))
        p.drawText(int(bx - R), h - 6, "kruhy po 10°")


# ---------------------------------------------------------------- serial
class SerialReader(QtCore.QThread):
    """Reads text lines from a serial port in a thread; emits each line."""
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port, self.baud = port, baud
        self._ser = None
        self._run = True
        self._tx: deque[str] = deque()

    def send(self, text: str):
        self._tx.append(text)

    def stop(self):
        self._run = False
        self.wait(1500)

    def run(self):
        if serial is None:
            self.status.emit("pyserial není nainstalován (pip install pyserial)")
            return
        while self._run:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
                self._ser.dtr = True
                self.status.emit(f"připojeno {self.port}")
                buf = b""
                while self._run:
                    while self._tx:
                        self._ser.write((self._tx.popleft() + "\n").encode())
                    chunk = self._ser.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        s = raw.decode(errors="replace").strip()
                        if s:
                            self.line.emit(s)
            except Exception as ex:  # unplugged, wrong port ...
                self.status.emit(f"odpojeno: {ex}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                for _ in range(20):
                    if not self._run:
                        break
                    time.sleep(0.05)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


def list_ports() -> list[str]:
    if serial is None:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


def to_float(s: str, default: float = float("nan")) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def to_int(s: str, default: int = 0) -> int:
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return default
