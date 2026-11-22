// =====================================================================
//  ZENITH — GROUND STATION (same PCB and pinout as the Daidalos ground station)
//
//  Target: Raspberry Pi Pico 2 W (rp2040:rp2040:rpipico2w), arduino-pico.
//  Needs : LoRa (sandeepmistry), Adafruit GFX, Adafruit SSD1306.
//
//  Pinout (Netlist_PCBGroundStation_2026-11-19):
//    LoRa Ra-02  SPI1: MISO GP12, CS GP11, SCK GP14, MOSI GP15, RST GP20, DIO0 GP21
//    OLED SSD1306 0x3C  I2C0: SDA GP4, SCL GP5 (400 kHz)
//    Buttons (active HIGH, 10k pull-downs):
//      BL GP3  short: next OLED page
//      BC GP6  hold 1.5 s: ABORT (works without the laptop)
//      BR GP10 hold 2 s: reboot the ground station
//
//  Radio: receives the rocket telemetry (zl_tlm_t) and, when no rocket is on
//  the pad, the rampa beacon (zl_rampa_beacon_t). Every downlink packet says
//  when its sender's next 300 ms RX window opens; the station locks to that
//  and transmits queued commands inside it (2 copies for ABORT/COUNTDOWN).
//  One command per 10 s window.
//
//  USB (115200, text lines) to ground_station_gui.py — docs/USB_PROTOCOL.md:
//    out  $T,...  telemetry      $B,...  rampa beacon     $U,...  uplink status
//         $G,...  station status #OK / #ERR
//    in   CMD <NAME> [arg]       CANCEL     TEST 0|1      HELLO
// =====================================================================
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "zenith_link.h"

static const int LORA_MISO = 12, LORA_CS = 11, LORA_SCK = 14, LORA_MOSI = 15, LORA_RST = 20, LORA_DIO0 = 21;
static const int OLED_SDA = 4, OLED_SCL = 5;
static const int BTN_L = 3, BTN_C = 6, BTN_R = 10;

static Adafruit_SSD1306 s_oled(128, 64, &Wire, -1);

static const char *ST_NAME[ZS_COUNT] = { "BOOT", "IDLE", "READY", "CDOWN", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED", "ABORT", "DIAG" };
static const char *RS_NAME[8] = { "OFF", "IDLE", "READY", "CDOWN", "COMMIT", "FIRED", "ABORT", "FAULT" };
static const char *CMD_NAME[ZL_UL_COUNT] = { "NOP", "PREPARE", "COUNTDOWN", "ABORT", "SAFE", "PADREF", "MAINALT", "LOCATE", "CAMERA", "REBOOT", "ROCKETPWR" };

// ---------------------------------------------------------------- state (core 0)
static zl_tlm_t          s_tlm = {};
static zl_rampa_beacon_t s_bcn = {};
static uint32_t s_tlm_ms = 0, s_bcn_ms = 0;
static int      s_rssi = 0; static float s_snr = 0;
static uint32_t s_rx_ok = 0, s_rx_bad = 0;
static bool     s_test = false;

// window lock (per sender)
static uint32_t s_win_open_ms = 0;
static bool     s_win_valid = false;
static uint8_t  s_win_dest = ZL_UL_DEST_ROCKET;

// uplink queue: one pending command
struct Pending {
    bool     active;
    uint8_t  seq, cmd, dest;
    int32_t  arg;
    uint8_t  sent;        // copies sent in the current window
    uint8_t  tries;       // windows used
    uint32_t win_ms;      // window the copies went into
};
static Pending  s_ul = {};
static uint8_t  s_seq = 0;

// snapshot for the display core
struct Disp { zl_tlm_t t; zl_rampa_beacon_t b; uint32_t t_age, b_age; int rssi; float snr; bool pend; uint8_t pcmd; uint32_t rx_ok; int32_t win_in; };
static critical_section_t s_cs;
static Disp s_disp = {};
static volatile bool s_ready = false;
static volatile bool s_abort_btn = false;

// ---------------------------------------------------------------- USB out
static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...)
{
    char b[300];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (Serial && Serial.availableForWrite() > (int)strlen(b) + 2) { Serial.print(b); Serial.print("\r\n"); }
}

static void emit_tlm(const zl_tlm_t &t)
{
    out("$T,%d,%.1f,%u,%u,%lu,%.7f,%.7f,%.2f,%ld,%.1f,%.1f,%.2f,%u,%.2f,%u,%u,%.1f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, t.state, t.flags, (unsigned long)t.t_ms, t.lat_1e7 * 1e-7, t.lon_1e7 * 1e-7,
        t.alt_cm * 0.01, (long)t.t_rem_ms, t.vz_dms * 0.1, t.vh_dms * 0.1, t.acc_cg * 0.01, t.max_alt_m,
        t.vbat_mv * 0.001, t.next_rx_in_ms, t.main_alt_m, t.tilt_dd * 0.1, t.mach_x100 * 0.01, t.sats, t.pyro,
        t.health, t.rampa_state, t.rampa_flags, t.last_ul_seq, t.last_ul_rc, t.err, t.win_final);
}
static void emit_bcn(const zl_rampa_beacon_t &b)
{
    out("$B,%d,%.1f,%u,%u,%lu,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, b.state, b.flags, (unsigned long)b.t_ms, b.vbat_mv * 0.001, b.umb_ma,
        b.cont_mv[0], b.cont_mv[1], b.cont_mv[2], b.next_rx_in_ms, b.faults, b.last_ul_seq, b.last_ul_rc, b.fw);
}
static void ul_status(const char *what, uint8_t rc = 0)
{
    out("$U,%u,%s,%u,%s,%u,%u", s_ul.seq, s_ul.cmd < ZL_UL_COUNT ? CMD_NAME[s_ul.cmd] : "?", s_ul.dest, what, rc, s_ul.tries);
}

// ---------------------------------------------------------------- uplink
static bool critical_cmd(uint8_t c) { return c == ZL_UL_ABORT || c == ZL_UL_COUNTDOWN; }

static void queue_cmd(uint8_t cmd, int32_t arg, uint8_t dest)
{
    s_seq = (uint8_t)(s_seq + 1); if (!s_seq) s_seq = 1;
    s_ul.active = true; s_ul.seq = s_seq; s_ul.cmd = cmd; s_ul.arg = arg; s_ul.dest = dest;
    s_ul.sent = 0; s_ul.tries = 0; s_ul.win_ms = 0;
    ul_status("QUEUED");
}

static void lock_window(uint32_t rx_end_ms, uint16_t pkt_len, uint16_t next_rx_in_ms, uint8_t dest)
{
    if (next_rx_in_ms == ZL_NEXT_RX_NONE) return;
    const uint32_t tx_start = rx_end_ms - zl_lora_airtime_us(pkt_len) / 1000u;
    s_win_open_ms = tx_start + next_rx_in_ms;
    s_win_valid = true;
    s_win_dest = dest;
}

static void send_uplink(uint32_t now, bool retry)
{
    zl_uplink_t u = {};
    u.seq = s_ul.seq; u.cmd = s_ul.cmd; u.arg = s_ul.arg; u.dest = s_ul.dest;
    u.token = zl_ul_token_for(s_ul.cmd);
    u.flags = (uint8_t)(retry ? ZL_ULF_RETRY : 0);
    zl_lora_seal(&u, sizeof u, ZL_UL_MAGIC);
    if (LoRa.beginPacket()) {
        LoRa.write((const uint8_t *)&u, sizeof u);
        LoRa.endPacket(false);
    }
    LoRa.receive();
    (void)now;
}

static void uplink_service(uint32_t now)
{
    if (!s_ul.active) return;
    // the sender we want must be the one we are locked to and recently heard
    const uint32_t heard = (s_ul.dest == ZL_UL_DEST_ROCKET) ? s_tlm_ms : s_bcn_ms;
    if (!s_win_valid || s_win_dest != s_ul.dest || !heard || (uint32_t)(now - heard) > 25000u) return;
    // predict the window forward on the 10 s grid if the lock is old
    while ((int32_t)(now - (s_win_open_ms + ZL_WIN_DUR_MS)) > 0) s_win_open_ms += ZL_WIN_PERIOD_MS;
    const int32_t dt = (int32_t)(now - s_win_open_ms);
    if (dt < 0) return;
    if (s_ul.win_ms != s_win_open_ms) { s_ul.win_ms = s_win_open_ms; s_ul.sent = 0; s_ul.tries++; }
    if (s_ul.tries > 3) { ul_status("FAILED"); s_ul.active = false; return; }
    if (s_ul.sent == 0 && dt >= (int32_t)ZL_UL_OFFSET1_MS && dt < (int32_t)(ZL_WIN_DUR_MS - 40)) {
        send_uplink(now, false); s_ul.sent = 1; ul_status("SENT");
    } else if (s_ul.sent == 1 && critical_cmd(s_ul.cmd) && dt >= (int32_t)ZL_UL_OFFSET2_MS && dt < (int32_t)(ZL_WIN_DUR_MS - 35)) {
        send_uplink(now, true); s_ul.sent = 2; ul_status("SENT2");
    }
}

static void check_ack(uint8_t seq, uint8_t rc, uint8_t dest)
{
    if (s_ul.active && s_ul.dest == dest && s_ul.sent && seq == s_ul.seq && rc != ZL_RC_NONE) {
        ul_status("ACK", rc);
        s_ul.active = false;
    }
}

// ---------------------------------------------------------------- serial commands
static int cmd_index(const char *n)
{
    for (int i = 0; i < ZL_UL_COUNT; i++) if (!strcasecmp(n, CMD_NAME[i])) return i;
    return -1;
}
static void serial_line(char *l)
{
    char *a0 = strtok(l, " "), *a1 = strtok(nullptr, " "), *a2 = strtok(nullptr, " "), *a3 = strtok(nullptr, " ");
    if (!a0) return;
    if (!strcasecmp(a0, "HELLO")) { out("$ID,Zenith Ground Station,%u.%02u", ZENITH_FW_VERSION / 100, ZENITH_FW_VERSION % 100); return; }
    if (!strcasecmp(a0, "TEST")) { s_test = a1 && atoi(a1); out("#OK TEST %d", s_test); return; }
    if (!strcasecmp(a0, "CANCEL")) { if (s_ul.active) ul_status("CANCELLED"); s_ul.active = false; out("#OK CANCEL"); return; }
    if (!strcasecmp(a0, "CMD") && a1) {
        const int c = cmd_index(a1);
        if (c < 0) { out("#ERR unknown command %s", a1); return; }
        int32_t arg = a2 ? (int32_t)atol(a2) : 0;
        if (c == ZL_UL_COUNTDOWN) {
            if (arg < (int32_t)(ZL_CD_MIN_MS / 1000) || arg > (int32_t)(ZL_CD_MAX_MS / 1000)) { out("#ERR countdown %d..%d s", (int)(ZL_CD_MIN_MS / 1000), (int)(ZL_CD_MAX_MS / 1000)); return; }
            arg *= 1000;   // ms from the moment the rocket hears it
        }
        uint8_t dest = (c == ZL_UL_ROCKET_PWR) ? ZL_UL_DEST_RAMPA : ZL_UL_DEST_ROCKET;
        if (a3 && !strcasecmp(a3, "RAMPA")) dest = ZL_UL_DEST_RAMPA;
        if (a2 && !strcasecmp(a2, "RAMPA")) { dest = ZL_UL_DEST_RAMPA; arg = 0; }
        if (s_ul.active && s_ul.cmd == ZL_UL_ABORT && c != ZL_UL_ABORT) { out("#ERR abort pending"); return; }
        queue_cmd((uint8_t)c, arg, dest);
        out("#OK queued %s seq %u", CMD_NAME[c], s_ul.seq);
        return;
    }
    out("#ERR HELLO | CMD <name> [arg] [RAMPA] | CANCEL | TEST 0|1");
}

// ---------------------------------------------------------------- test generator
static void test_packet(uint32_t now)
{
    static uint32_t next = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + 200;
    const float t = fmodf(now * 0.001f, 120.0f);
    zl_tlm_t p = {};
    float h = 0, vz = 0; uint8_t st = ZS_READY;
    if (t < 20) { st = ZS_COUNTDOWN; p.t_rem_ms = (int32_t)((20 - t) * 1000); }
    else if (t < 22.5f) { st = ZS_BOOST; vz = (t - 20) * 180; h = 90 * (t - 20) * (t - 20); }
    else if (t < 42) { st = ZS_COAST; const float c = t - 22.5f; vz = 450 - 23 * c; h = 560 + 450 * c - 11.5f * c * c; }
    else if (t < 110) { st = t < 100 ? ZS_DROGUE : ZS_MAIN; const float c = t - 42; vz = st == ZS_DROGUE ? -25 : -6; h = 4960 - 25 * c; if (h < 0) h = 0; }
    else st = ZS_LANDED;
    p.state = st; p.alt_cm = (int32_t)(h * 100); p.vz_dms = (int16_t)(vz * 10); p.max_alt_m = (uint16_t)(h > 0 ? h : 0);
    p.lat_1e7 = 497985411 + (int32_t)(t * 30); p.lon_1e7 = 166912772 + (int32_t)(t * 50);
    p.vbat_mv = 8100; p.sats = 12; p.pyro = 0x47; p.health = 0x3F; p.flags = 0xC3; p.main_alt_m = 300;
    p.mach_x100 = (uint8_t)(fabsf(vz) / 3.4f); p.acc_cg = (int16_t)(st == ZS_BOOST ? 2000 : -30);
    p.rampa_state = st <= ZS_COUNTDOWN ? ZR_ST_COUNTDOWN : ZR_ST_OFFLINE;
    p.next_rx_in_ms = (uint16_t)(10000 - (now % 10000));
    p.t_ms = now;
    s_tlm = p; s_tlm_ms = now; s_rssi = -70; s_snr = 9.5f;
    emit_tlm(p);
}

// ---------------------------------------------------------------- setup / loop (core 0)
void setup()
{
    Serial.begin(115200);
    critical_section_init(&s_cs);
    SPI1.setRX(LORA_MISO); SPI1.setTX(LORA_MOSI); SPI1.setSCK(LORA_SCK);
    SPI1.begin();
    LoRa.setSPI(SPI1);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    bool ok = LoRa.begin(ZL_LORA_FREQ_HZ);
    if (ok) {
        LoRa.setSpreadingFactor(ZL_LORA_SF);
        LoRa.setSignalBandwidth(ZL_LORA_BW_HZ);
        LoRa.setCodingRate4(ZL_LORA_CR4);
        LoRa.setPreambleLength(ZL_LORA_PREAMBLE);
        LoRa.setSyncWord(ZL_LORA_SYNCWORD);
        LoRa.setTxPower(ZL_LORA_TX_DBM);
        LoRa.enableCrc();
        LoRa.receive();
    }
    if (!ok) out("#ERR LoRa init failed");
    s_ready = true;
    watchdog_enable(2000, true);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    // ---- USB --------------------------------------------------------------------
    static char line[96]; static uint8_t n = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (n) { line[n] = 0; serial_line(line); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = c;
    }
    if (s_abort_btn) { s_abort_btn = false; queue_cmd(ZL_UL_ABORT, 0, ZL_UL_DEST_ROCKET); }

    // ---- receive ---------------------------------------------------------------------
    const int sz = LoRa.parsePacket();
    if (sz > 0) {
        uint8_t b[64]; int k = 0;
        while (LoRa.available() && k < (int)sizeof b) b[k++] = (uint8_t)LoRa.read();
        while (LoRa.available()) LoRa.read();
        const uint32_t rx = millis();
        s_rssi = LoRa.packetRssi(); s_snr = LoRa.packetSnr();
        if (zl_lora_check(b, (size_t)k, sizeof(zl_tlm_t), ZL_TLM_MAGIC)) {
            memcpy(&s_tlm, b, sizeof s_tlm);
            s_tlm_ms = rx; s_rx_ok++; s_test = false;
            lock_window(rx, sizeof(zl_tlm_t), s_tlm.next_rx_in_ms, ZL_UL_DEST_ROCKET);
            check_ack(s_tlm.last_ul_seq, s_tlm.last_ul_rc, ZL_UL_DEST_ROCKET);
            emit_tlm(s_tlm);
        } else if (zl_lora_check(b, (size_t)k, sizeof(zl_rampa_beacon_t), ZL_BEACON_MAGIC)) {
            memcpy(&s_bcn, b, sizeof s_bcn);
            s_bcn_ms = rx; s_rx_ok++;
            // a rampa beacon means no rocket is talking: lock to the rampa
            if (!s_tlm_ms || (uint32_t)(rx - s_tlm_ms) > 5000u)
                lock_window(rx, sizeof(zl_rampa_beacon_t), s_bcn.next_rx_in_ms, ZL_UL_DEST_RAMPA);
            check_ack(s_bcn.last_ul_seq, s_bcn.last_ul_rc, ZL_UL_DEST_RAMPA);
            emit_bcn(s_bcn);
        } else {
            s_rx_bad++;
        }
    }
    if (s_test) test_packet(now);
    uplink_service(now);

    // ---- station status 1 Hz ---------------------------------------------------------------
    static uint32_t t_g = 0;
    if ((int32_t)(now - t_g) >= 0) {
        t_g = now + 1000;
        const int32_t win_in = s_win_valid ? (int32_t)(s_win_open_ms - now) : -1;
        out("$G,%lu,%lu,%ld,%d,%ld,%u,%u,%ld", (unsigned long)s_rx_ok, (unsigned long)s_rx_bad,
            (long)(s_tlm_ms ? (int32_t)(now - s_tlm_ms) : -1), s_win_valid, (long)win_in, s_ul.active, s_win_dest,
            (long)(s_bcn_ms ? (int32_t)(now - s_bcn_ms) : -1));
    }

    // ---- display snapshot ------------------------------------------------------------------------
    Disp d;
    d.t = s_tlm; d.b = s_bcn;
    d.t_age = s_tlm_ms ? now - s_tlm_ms : 0xFFFFFFFF;
    d.b_age = s_bcn_ms ? now - s_bcn_ms : 0xFFFFFFFF;
    d.rssi = s_rssi; d.snr = s_snr; d.pend = s_ul.active; d.pcmd = s_ul.cmd; d.rx_ok = s_rx_ok;
    d.win_in = s_win_valid ? (int32_t)(s_win_open_ms - now) : -1;
    critical_section_enter_blocking(&s_cs); s_disp = d; critical_section_exit(&s_cs);
}

// ---------------------------------------------------------------- core 1: OLED + buttons
void setup1()
{
    while (!s_ready) delay(1);
    pinMode(BTN_L, INPUT); pinMode(BTN_C, INPUT); pinMode(BTN_R, INPUT);
    Wire.setSDA(OLED_SDA); Wire.setSCL(OLED_SCL);
    Wire.begin(); Wire.setClock(400000);
    if (s_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        s_oled.clearDisplay(); s_oled.setTextColor(SSD1306_WHITE); s_oled.setTextSize(1);
        s_oled.setCursor(10, 20); s_oled.print("ZENITH GROUND ST.");
        s_oled.setCursor(10, 40); s_oled.print("waiting for link");
        s_oled.display();
    }
}

void loop1()
{
    static uint8_t page = 0;
    static uint32_t c_down = 0, r_down = 0, next = 0;
    static bool l_prev = false;
    const uint32_t now = millis();

    const bool l = digitalRead(BTN_L) == HIGH, c = digitalRead(BTN_C) == HIGH, r = digitalRead(BTN_R) == HIGH;
    if (now > 3000) {
        if (l && !l_prev) page = (uint8_t)((page + 1) % 3);
        l_prev = l;
        if (c) { if (!c_down) c_down = now; if (now - c_down > 1500 && c_down != 1) { s_abort_btn = true; c_down = 1; } }
        else c_down = 0;
        if (r) { if (!r_down) r_down = now; if (now - r_down > 2000) { watchdog_enable(1, true); while (1) {} } }
        else r_down = 0;
    }
    if ((int32_t)(now - next) < 0) { delay(5); return; }
    next = now + 200;

    Disp d;
    critical_section_enter_blocking(&s_cs); d = s_disp; critical_section_exit(&s_cs);
    s_oled.clearDisplay();
    s_oled.setTextSize(1);
    s_oled.setCursor(0, 0);
    s_oled.printf("R%d S%.0f #%lu", d.rssi, (double)d.snr, (unsigned long)d.rx_ok);
    if (d.pend) { s_oled.setCursor(98, 0); s_oled.print("UL"); s_oled.print(d.pcmd); }

    const bool rocket = d.t_age < 3000;
    if (!rocket && d.b_age < 5000) {
        // rampa alone
        s_oled.setCursor(0, 14); s_oled.setTextSize(2);
        s_oled.print("RAMPA "); s_oled.print(d.b.state < 8 ? RS_NAME[d.b.state] : "?");
        s_oled.setTextSize(1);
        s_oled.setCursor(0, 34); s_oled.printf("Bat %.2fV  umb %umA", d.b.vbat_mv * 0.001, d.b.umb_ma);
        s_oled.setCursor(0, 44); s_oled.printf("cont %u %u %u mV", d.b.cont_mv[0], d.b.cont_mv[1], d.b.cont_mv[2]);
        s_oled.setCursor(0, 54); s_oled.printf("key%d flt %02X win %lds", (d.b.flags & ZL_RMP_KEY) ? 1 : 0, d.b.faults, (long)(d.win_in / 1000));
    } else if (d.t_age == 0xFFFFFFFF) {
        s_oled.setCursor(0, 24); s_oled.setTextSize(2); s_oled.print("NO LINK");
    } else {
        const zl_tlm_t &t = d.t;
        if (d.t_age > 3000) { s_oled.setCursor(60, 0); s_oled.printf("LOST %lus", (unsigned long)(d.t_age / 1000)); }
        s_oled.setTextSize(2);
        s_oled.setCursor(0, 12);
        if (t.state == ZS_COUNTDOWN && t.t_rem_ms >= 0) s_oled.printf("T-%.1f", (t.t_rem_ms - (int32_t)d.t_age) * 0.001);
        else s_oled.printf("%ldm", (long)(t.alt_cm / 100));
        s_oled.setTextSize(1);
        s_oled.setCursor(84, 12); s_oled.print(t.state < ZS_COUNT ? ST_NAME[t.state] : "?");
        s_oled.setCursor(84, 22); s_oled.print((t.flags & ZL_TF_GO) ? "GO" : "NOGO");
        if (page == 0) {
            s_oled.setCursor(0, 32); s_oled.printf("vz %.1f max %u", t.vz_dms * 0.1, t.max_alt_m);
            s_oled.setCursor(0, 42); s_oled.printf("M%.2f B%.2fV sat%u", t.mach_x100 * 0.01, t.vbat_mv * 0.001, t.sats);
            s_oled.setCursor(0, 54);
            for (int i = 0; i < 3; i++) s_oled.print((t.pyro & (1 << i)) ? (char)('1' + i) : '-');
            s_oled.print(" F");
            for (int i = 0; i < 3; i++) s_oled.print((t.pyro & (8 << i)) ? (char)('1' + i) : '-');
            s_oled.printf(" %s E%u", (t.flags & ZL_TF_ARMED) ? "ARM" : "saf", t.err);
        } else if (page == 1) {
            s_oled.setCursor(0, 32); s_oled.printf("%.6f", t.lat_1e7 * 1e-7);
            s_oled.setCursor(0, 42); s_oled.printf("%.6f", t.lon_1e7 * 1e-7);
            s_oled.setCursor(0, 54); s_oled.printf("sat %u main %um", t.sats, t.main_alt_m);
        } else {
            s_oled.setCursor(0, 32); s_oled.printf("rampa %s fl %02X", t.rampa_state < 8 ? RS_NAME[t.rampa_state] : "?", t.rampa_flags);
            s_oled.setCursor(0, 42); s_oled.printf("ul seq %u rc %u", t.last_ul_seq, t.last_ul_rc);
            s_oled.setCursor(0, 54); s_oled.printf("win in %lds", (long)(d.win_in / 1000));
        }
    }
    if (d.pend && d.pcmd == ZL_UL_ABORT) { s_oled.fillRect(96, 54, 32, 10, SSD1306_WHITE); s_oled.setTextColor(SSD1306_BLACK); s_oled.setCursor(98, 55); s_oled.print("ABRT"); s_oled.setTextColor(SSD1306_WHITE); }
    s_oled.display();
}
