// =====================================================================
//  ZENITH v2.4 — MASTER MCU (U15, Raspberry Pi Pico 2)
//
//  "What is the mission doing?" Owns the mission state machine, the
//  countdown with the launch pad (Rampa) over the pogo umbilical, the three
//  recovery pyro channels, the LoRa link to the ground station, the flash
//  log, the parameter table and the USB diagnostic console.
//
//  Board : Raspberry Pi Pico 2 (arduino-pico core, rp2040:rp2040:rpipico2)
//  Needs : LoRa (sandeepmistry), Adafruit NeoPixel
//
//  Pinout (zenith_netlist_v2_4.txt — full table in docs/PINOUT.md):
//    GP0/GP1   UART0 (Serial1)  link to the Position MCU, 921600 Bd
//    GP2..GP6  SPI0  LoRa Ra-02: SCK GP2, MOSI GP3, MISO GP4, NSS GP5, DIO0 GP6
//    GP7       RBF_NC_SENSE  (LOW = RBF out, pyro rail can be armed)
//    GP8/GP9   UART1 (Serial2)  umbilical TX/RX -> pogo J1.3 / J1.4, 115200 Bd
//    GP10..13  SPI1  W25Q128 flash: SCK GP10, MOSI GP11, MISO GP12, CS GP13
//    GP14/GP15 I2C1 (Wire1)  INA226 0x40, ADS1115 0x48, PCA9685 0x41
//    GP16/17/18 FIRE1/2/3 (low-side AO3400A)    GP19 PYRO_SW_ARM (rail Q3)
//    GP20/GP21 LAUNCH_BW1/2 (LOW = wire intact)  GP22 CAM_TX (PIO UART -> RunCam)
//    GP26      XIAO_SYNC -> Position GP17 (HIGH = in flight)
//    GP27      WS2812B RGB      GP28 UMB_PRESENT (ADC2, pogo 5 V / 2)
//    PCA9685:  LED6 CAM_EN, LED7 SERVO_PWR_EN, LED8 LORA_RST (low = reset),
//              LED9 BUZZER_EN, LED10 FORCE_BAT, LED0..5 servo outputs (unused)
//    ADS1115:  AIN0..2 CONT1..3, AIN3 PYRO_SENSE (all 100k/47k = 0.3197)
//
//  Cores:
//    core 0  links, I2C, state machine, pyros, USB console    — never blocks
//    core 1  LoRa TDMA (TX ~51 ms blocking), flash log writer — may block
//
//  Pyro channel use: PYRO1 = drogue (apogee), PYRO2 = main (main_alt_m),
//  PYRO3 = backup charges of BOTH chutes (fired on detected chute failure).
// =====================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/watchdog.h"
#include "pico/util/queue.h"
#include "pico/critical_section.h"

#include "zenith_link.h"
#include "zenith_params.h"
#include "zenith_flight.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// =====================================================================
//  1. PINS & CONSTANTS
// =====================================================================
static const uint8_t PIN_LINK_TX   = 0,  PIN_LINK_RX  = 1;
static const uint8_t PIN_LORA_SCK  = 2,  PIN_LORA_MOSI = 3, PIN_LORA_MISO = 4, PIN_LORA_CS = 5, PIN_LORA_DIO0 = 6;
static const uint8_t PIN_RBF       = 7;
static const uint8_t PIN_UMB_TX    = 8,  PIN_UMB_RX   = 9;
static const uint8_t PIN_FL_SCK    = 10, PIN_FL_MOSI  = 11, PIN_FL_MISO = 12, PIN_FL_CS = 13;
static const uint8_t PIN_I2C_SDA   = 14, PIN_I2C_SCL  = 15;
static const uint8_t PIN_FIRE[3]   = { 16, 17, 18 };
static const uint8_t PIN_ARM       = 19;
static const uint8_t PIN_BW1       = 20, PIN_BW2      = 21;
static const uint8_t PIN_CAM_TX    = 22;
static const uint8_t PIN_SYNC      = 26;
static const uint8_t PIN_RGB       = 27;
static const uint8_t PIN_UMB_SENSE = 28;

static const uint8_t ADDR_INA = 0x40, ADDR_ADS = 0x48, ADDR_PCA = 0x41;
static const uint8_t PCA_CAM = 6, PCA_SERVO_PWR = 7, PCA_LORA_RST = 8, PCA_BUZZER = 9, PCA_FORCE_BAT = 10;

static const float   DIV_RATIO    = 47.0f / 147.0f;   // CONTx / PYRO_SENSE dividers
static const uint32_t NAV_FRESH_MS = 200;
static const uint32_t WDT_MS       = 1000;

// =====================================================================
//  2. SHARED STATE (core 0 unless noted)
// =====================================================================
static float    P[ZP_COUNT];                 // parameter values
static uint8_t  s_state = ZS_BOOT;
static uint8_t  s_err = ZE_NONE;             // first active no-go reason
static uint8_t  s_abort_err = ZE_NONE;       // reason the last abort happened
static uint32_t s_state_ms = 0;

// sensors
static bool     s_ina_ok = false, s_ads_ok = false, s_pca_ok = false;
static uint16_t s_vbat_mv = 0;
static int16_t  s_curr_ma = 0;
static uint16_t s_ads_mv[4] = { 0, 0, 0, 0 };  // CONT1..3, PYRO_SENSE at the ADC pin
static bool     s_cont[3] = { false, false, false };
static bool     s_rbf_armable = false, s_bw[2] = { false, false }, s_umb = false;
static uint16_t s_umb_mv = 0;

// Position MCU
static zl_nav_t        s_nav = {};
static zl_pos_status_t s_pos = {};
static zl_pos_raw_t    s_raw = {};
static uint32_t s_nav_ms = 0, s_pos_ms = 0, s_raw_ms = 0;
static zl_rx_t  s_link_rx;
static uint8_t  s_pyro_events = 0;
static bool     s_raw_stream = false;

// Rampa
static zl_rmp_status_t s_rmp = {};
static uint32_t s_rmp_ms = 0;
static zl_rx_t  s_umb_rx;
static uint8_t  s_rmp_seq = 0;

// pyro outputs
static bool     s_armed = false;
static uint8_t  s_fired = 0;                 // channels fired (flight)
static uint32_t s_fire_end[3] = { 0, 0, 0 };

// countdown
static uint32_t s_t0_ms = 0;                 // T-0 in millis()
static bool     s_cd_active = false;
static bool     s_force_bat = false;
static bool     s_commit_sent = false;
static uint32_t s_bat_check_ms = 0;
static uint8_t  s_bw_at_start = 0;           // breakwires intact when the countdown started
static zf_lift_t s_lift = {};
static uint32_t s_ready_ms = 0;

// flight
static zf_t     s_fl = {};

// misc outputs
static bool     s_cam_on = false, s_cam_rec = false, s_locate = false;
static bool     s_diag_buzz = false;
static uint32_t s_buzz_until = 0;
static bool     s_flash_ok = false;
static uint32_t s_flash_jedec = 0;
static uint8_t  s_last_ul_seq = 0, s_last_ul_rc = ZL_RC_NONE;
static uint8_t  s_reset_reason = 0;
static bool     s_lora_ok = false;           // written by core 1
static volatile uint32_t s_lora_rx_count = 0, s_lora_tx_count = 0;
static volatile int16_t  s_lora_rssi = 0;

static Adafruit_NeoPixel s_rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static SerialPIO         s_cam(PIN_CAM_TX, NOPIN);

// core 0 <-> core 1
struct UplinkMsg { zl_uplink_t u; uint32_t rx_ms; int16_t rssi; };
static queue_t s_ul_q;                                    // core1 -> core0
struct LogRec { uint8_t b[64]; };
static queue_t s_log_q;                                   // core0 -> core1
static critical_section_t s_tlm_cs;
static zl_tlm_t s_tlm_shared = {};                        // core0 writes, core1 reads
static volatile int32_t  s_tlm_t0_rem_ref = -1;           // countdown T-0 (millis) for core 1 windows
static volatile uint32_t s_tlm_t0_ms = 0;
static volatile uint8_t  s_tlm_hz = ZL_TLM_HZ_DEFAULT;
static volatile bool     s_core0_ready = false;
// flash ownership: core 1 owns it; core 0 asks for a pause for param save / log dump
static volatile bool     s_flash_pause_req = false, s_flash_paused = false;
static volatile bool     s_log_erase_req = false, s_log_erasing = false;
static volatile uint32_t s_log_head = 0, s_log_dropped = 0;
static volatile bool     s_logging = false;
static volatile uint32_t s_core1_alive_ms = 0;

// =====================================================================
//  3. USB CONSOLE OUTPUT (never blocks in flight)
// =====================================================================
static uint32_t s_usb_drops = 0;
static void usb_line(const char *s)
{
    if (!Serial) return;
    const size_t n = strlen(s);
    if (Serial.availableForWrite() < (int)(n + 2)) { s_usb_drops++; return; }
    Serial.write((const uint8_t *)s, n);
    Serial.write("\r\n");
}
static void usb_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void usb_printf(const char *fmt, ...)
{
    char b[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    usb_line(b);
}

static void log_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// =====================================================================
//  4. I2C DRIVERS (Wire1: INA226, ADS1115, PCA9685)
// =====================================================================
static bool i2c_w8(uint8_t a, uint8_t r, uint8_t v)
{
    Wire1.beginTransmission(a); Wire1.write(r); Wire1.write(v);
    return Wire1.endTransmission() == 0;
}
static bool i2c_w16(uint8_t a, uint8_t r, uint16_t v)
{
    Wire1.beginTransmission(a); Wire1.write(r); Wire1.write((uint8_t)(v >> 8)); Wire1.write((uint8_t)v);
    return Wire1.endTransmission() == 0;
}
static bool i2c_r16(uint8_t a, uint8_t r, uint16_t *v)
{
    Wire1.beginTransmission(a); Wire1.write(r);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)a, 2) != 2) return false;
    const uint8_t h = (uint8_t)Wire1.read(), l = (uint8_t)Wire1.read();
    *v = (uint16_t)((h << 8) | l);
    return true;
}

// ---- PCA9685 (outputs, not servos: no active control on Zenith) ----------
static uint8_t s_pca_cache[16];   // 0 off, 1 on, 2 buzzer PWM
static bool pca_begin(void)
{
    bool ok = i2c_w8(ADDR_PCA, 0x00, 0x10);           // MODE1: sleep
    ok = ok && i2c_w8(ADDR_PCA, 0xFE, 3);             // PRE_SCALE 3 -> ~1.5 kHz (buzzer tone)
    ok = ok && i2c_w8(ADDR_PCA, 0x01, 0x04);          // MODE2: totem pole
    ok = ok && i2c_w8(ADDR_PCA, 0xFA, 0x00);          // ALL_LED_ON  = 0
    ok = ok && i2c_w8(ADDR_PCA, 0xFB, 0x00);
    ok = ok && i2c_w8(ADDR_PCA, 0xFC, 0x00);          // ALL_LED_OFF = full off
    ok = ok && i2c_w8(ADDR_PCA, 0xFD, 0x10);
    ok = ok && i2c_w8(ADDR_PCA, 0x00, 0x20);          // MODE1: auto-increment, awake
    delayMicroseconds(600);
    memset(s_pca_cache, 0, sizeof s_pca_cache);
    return ok;
}
static bool pca_set(uint8_t ch, uint8_t mode)
{
    if (ch > 15) return false;
    if (s_pca_cache[ch] == mode && s_pca_ok) return true;
    uint16_t on = 0, off = 0x1000;                    // full off
    if (mode == 1) { on = 0x1000; off = 0; }          // full on
    if (mode == 2) { on = 0; off = 2048; }            // 50 % at ~1.5 kHz
    Wire1.beginTransmission(ADDR_PCA);
    Wire1.write((uint8_t)(0x06 + 4 * ch));
    Wire1.write((uint8_t)on); Wire1.write((uint8_t)(on >> 8));
    Wire1.write((uint8_t)off); Wire1.write((uint8_t)(off >> 8));
    const bool ok = Wire1.endTransmission() == 0;
    if (ok) s_pca_cache[ch] = mode;
    s_pca_ok = ok;
    return ok;
}

// ---- INA226 (battery, 5 mOhm, 1 mA/LSB) ------------------------------------
static bool ina_begin(void)
{
    return i2c_w16(ADDR_INA, 0x00, 0x4727) &&         // avg 16, 1.1 ms, continuous
           i2c_w16(ADDR_INA, 0x05, 1024);             // CAL = 0.00512 / (1 mA * 5 mOhm)
}
static void ina_read(void)
{
    uint16_t v, c;
    s_ina_ok = i2c_r16(ADDR_INA, 0x02, &v) && i2c_r16(ADDR_INA, 0x04, &c);
    if (s_ina_ok) {
        s_vbat_mv = (uint16_t)(((uint32_t)v * 125u) / 100u);
        s_curr_ma = (int16_t)c;
    }
}

// ---- ADS1115 (single shot, round robin, +-4.096 V, 860 SPS) ----------------
static bool ads_start(uint8_t ch)
{
    const uint16_t cfg = (uint16_t)(0x8000u | ((uint16_t)(4u + (ch & 3u)) << 12) | (1u << 9) | (1u << 8) | (7u << 5) | 0x0003u);
    return i2c_w16(ADDR_ADS, 0x01, cfg);
}
static void ads_poll(void)
{
    static uint8_t ch = 0;
    static bool started = false;
    static uint32_t t_start = 0;
    if (!started) { s_ads_ok = ads_start(ch); started = s_ads_ok; t_start = millis(); return; }
    uint16_t cfg;
    if (!i2c_r16(ADDR_ADS, 0x01, &cfg)) { s_ads_ok = false; started = false; return; }
    if (!(cfg & 0x8000u)) { if (millis() - t_start > 20) started = false; return; }
    uint16_t raw;
    if (!i2c_r16(ADDR_ADS, 0x00, &raw)) { s_ads_ok = false; started = false; return; }
    int16_t sr = (int16_t)raw; if (sr < 0) sr = 0;
    s_ads_mv[ch] = (uint16_t)((uint32_t)sr >> 3);     // 0.125 mV/LSB
    s_ads_ok = true;
    ch = (uint8_t)((ch + 1u) & 3u);
    started = ads_start(ch);
    t_start = millis();
}

// =====================================================================
//  5. W25Q128 FLASH (SPI1) — log in sectors 0..4093, params in 4094/4095
// =====================================================================
static const uint32_t FL_SECTOR = 4096;
static const uint32_t FL_PARAM_A = 4094u * FL_SECTOR, FL_PARAM_B = 4095u * FL_SECTOR;
static const uint32_t LOG_REC = 64;
static const uint32_t LOG_END = 4094u * FL_SECTOR;      // bytes
static const uint32_t LOG_NREC = LOG_END / LOG_REC;
static SPISettings s_fl_spi(20000000, MSBFIRST, SPI_MODE0);

static inline void fl_sel(void)   { SPI1.beginTransaction(s_fl_spi); digitalWrite(PIN_FL_CS, LOW); }
static inline void fl_desel(void) { digitalWrite(PIN_FL_CS, HIGH); SPI1.endTransaction(); }
static uint8_t fl_status(void) { fl_sel(); SPI1.transfer(0x05); const uint8_t s = SPI1.transfer(0); fl_desel(); return s; }
static bool fl_busy(void) { return fl_status() & 0x01; }
static bool fl_wait(uint32_t ms) { const uint32_t t = millis(); while (fl_busy()) { if (millis() - t > ms) return false; } return true; }
static void fl_wren(void) { fl_sel(); SPI1.transfer(0x06); fl_desel(); }
static uint32_t fl_jedec(void)
{
    fl_sel(); SPI1.transfer(0x9F);
    uint32_t id = (uint32_t)SPI1.transfer(0) << 16; id |= (uint32_t)SPI1.transfer(0) << 8; id |= SPI1.transfer(0);
    fl_desel(); return id;
}
static void fl_read(uint32_t a, uint8_t *b, uint32_t n)
{
    fl_sel(); SPI1.transfer(0x03); SPI1.transfer((uint8_t)(a >> 16)); SPI1.transfer((uint8_t)(a >> 8)); SPI1.transfer((uint8_t)a);
    for (uint32_t i = 0; i < n; i++) b[i] = SPI1.transfer(0);
    fl_desel();
}
static void fl_program(uint32_t a, const uint8_t *b, uint32_t n)   // n <= 256, within one page
{
    fl_wren();
    fl_sel(); SPI1.transfer(0x02); SPI1.transfer((uint8_t)(a >> 16)); SPI1.transfer((uint8_t)(a >> 8)); SPI1.transfer((uint8_t)a);
    for (uint32_t i = 0; i < n; i++) SPI1.transfer(b[i]);
    fl_desel();
}
static void fl_erase_cmd(uint8_t cmd, uint32_t a)
{
    fl_wren();
    fl_sel(); SPI1.transfer(cmd); SPI1.transfer((uint8_t)(a >> 16)); SPI1.transfer((uint8_t)(a >> 8)); SPI1.transfer((uint8_t)a);
    fl_desel();
}

// ---- core 0 access to the flash (only on the ground) -----------------------
static bool flash_acquire(uint32_t timeout_ms)
{
    s_flash_pause_req = true;
    const uint32_t t = millis();
    while (!s_flash_paused) { if (millis() - t > timeout_ms) { s_flash_pause_req = false; return false; } watchdog_update(); }
    return true;
}
static void flash_release(void) { s_flash_pause_req = false; }

// ---- parameters ---------------------------------------------------------------
struct ParamImage { uint32_t magic; uint16_t count; uint16_t seq; float v[ZP_COUNT]; uint16_t crc; uint16_t _pad; };
static uint16_t s_param_seq = 0;
static void params_defaults(void) { for (int i = 0; i < ZP_COUNT; i++) P[i] = ZP_TABLE[i].def; }
static bool param_img_ok(const ParamImage &im)
{
    return im.magic == 0x5A50524Du && im.count == ZP_COUNT &&
           zl_crc16((const uint8_t *)&im, offsetof(ParamImage, crc)) == im.crc;
}
static void params_load_raw(void)     // flash already owned by the caller
{
    ParamImage a, b;
    fl_read(FL_PARAM_A, (uint8_t *)&a, sizeof a);
    fl_read(FL_PARAM_B, (uint8_t *)&b, sizeof b);
    const bool oa = param_img_ok(a), ob = param_img_ok(b);
    const ParamImage *use = nullptr;
    if (oa && ob) use = ((int16_t)(a.seq - b.seq) >= 0) ? &a : &b;
    else if (oa) use = &a; else if (ob) use = &b;
    params_defaults();
    if (use) {
        s_param_seq = use->seq;
        for (int i = 0; i < ZP_COUNT; i++) {
            const float v = use->v[i];
            if (v >= ZP_TABLE[i].min && v <= ZP_TABLE[i].max) P[i] = v;
        }
    }
}
static bool params_save_raw(void)
{
    ParamImage im = {};
    im.magic = 0x5A50524Du; im.count = ZP_COUNT; im.seq = (uint16_t)(s_param_seq + 1);
    for (int i = 0; i < ZP_COUNT; i++) im.v[i] = P[i];
    im.crc = zl_crc16((const uint8_t *)&im, offsetof(ParamImage, crc));
    const uint32_t addr = (im.seq & 1u) ? FL_PARAM_B : FL_PARAM_A;
    fl_erase_cmd(0x20, addr);
    if (!fl_wait(500)) return false;
    const uint8_t *p = (const uint8_t *)&im;
    for (uint32_t o = 0; o < sizeof im; o += 256) {
        const uint32_t n = (sizeof im - o) > 256 ? 256 : (sizeof im - o);
        fl_program(addr + o, p + o, n);
        if (!fl_wait(10)) return false;
    }
    ParamImage chk; fl_read(addr, (uint8_t *)&chk, sizeof chk);
    if (!param_img_ok(chk)) return false;
    s_param_seq = im.seq;
    return true;
}
static bool params_save(void)
{
    if (zl_state_in_flight(s_state) || s_state == ZS_COUNTDOWN) return false;
    if (!s_flash_ok || !flash_acquire(2000)) return false;
    const bool ok = params_save_raw();
    flash_release();
    return ok;
}

// ---- log records ---------------------------------------------------------------
#pragma pack(push, 1)
struct LogData {               // 64 bytes
    uint8_t  magic;            // 0xA7 data, 0xA8 event
    uint8_t  state, flags, pyro;
    uint32_t t_ms;
    float    alt, vz, baro_alt, acc_axial, acc_mag, mach, tilt;
    int32_t  lat, lon;
    uint16_t vbat_mv; int16_t curr_ma;
    uint16_t nav_flags;
    int16_t  px, py;
    uint16_t max_alt;
    uint8_t  rampa_state, err, sats, nav_mode;
    int16_t  vh_dms;           // horizontal speed
    uint16_t crc;
};
struct LogEvent {
    uint8_t  magic;            // 0xA8
    uint8_t  state;
    uint16_t _r;
    uint32_t t_ms;
    char     text[54];
    uint16_t crc;
};
#pragma pack(pop)
static_assert(sizeof(LogData) == 64, "log record");
static_assert(sizeof(LogEvent) == 64, "log event");

static void log_push(const uint8_t *b64)
{
    if (!s_logging) return;
    LogRec r; memcpy(r.b, b64, 64);
    if (!queue_try_add(&s_log_q, &r)) s_log_dropped++;
}

static void log_event(const char *fmt, ...)
{
    LogEvent e = {};
    e.magic = 0xA8; e.state = s_state; e.t_ms = millis();
    va_list ap; va_start(ap, fmt);
    vsnprintf(e.text, sizeof e.text, fmt, ap);
    va_end(ap);
    e.crc = zl_crc16((const uint8_t *)&e, 62);
    log_push((const uint8_t *)&e);
    usb_printf("$EV,%lu,%s", (unsigned long)e.t_ms, e.text);
}

// =====================================================================
//  6. LINKS: Position MCU and Rampa
// =====================================================================
static void link_send(HardwareSerial &port, uint8_t type, const void *p, uint8_t n)
{
    uint8_t f[ZL_FRAME_MAX];
    const uint16_t len = zl_encode(f, type, p, n);
    if (len && port.availableForWrite() >= (int)len) port.write(f, len);
}

static void pos_cmd(uint8_t cmd, uint8_t arg = 0, float val = 0)
{
    zl_pos_cmd_t c = {}; c.cmd = cmd; c.arg = arg; c.val = val;
    link_send(Serial1, ZL_PKT_POS_CMD, &c, sizeof c);
}

static void link_poll(uint32_t now)
{
    int budget = 1024;
    while (Serial1.available() && budget-- > 0) {
        const uint8_t *pl; uint8_t n;
        switch (zl_rx_byte(&s_link_rx, (uint8_t)Serial1.read(), &pl, &n)) {
        case ZL_PKT_NAV:        if (zl_take(&s_nav, sizeof s_nav, pl, n)) s_nav_ms = now; break;
        case ZL_PKT_POS_STATUS: if (zl_take(&s_pos, sizeof s_pos, pl, n)) s_pos_ms = now; break;
        case ZL_PKT_POS_RAW:    if (zl_take(&s_raw, sizeof s_raw, pl, n)) s_raw_ms = now; break;
        default: break;
        }
    }
    budget = 256;
    while (Serial2.available() && budget-- > 0) {
        const uint8_t *pl; uint8_t n;
        if (zl_rx_byte(&s_umb_rx, (uint8_t)Serial2.read(), &pl, &n) == ZL_PKT_RMP_STATUS) {
            zl_rmp_status_t st;
            if (zl_take(&st, sizeof st, pl, n)) { s_rmp = st; s_rmp_ms = now; }
        }
    }
}

static inline bool nav_fresh(uint32_t now) { return s_nav_ms && (uint32_t)(now - s_nav_ms) < NAV_FRESH_MS; }
static inline bool nav_ok(uint32_t now)    { return nav_fresh(now) && (s_nav.flags & ZL_NAV_VALID); }
static inline bool rampa_online(uint32_t now) { return s_rmp_ms && (uint32_t)(now - s_rmp_ms) < 600u; }

static void rampa_send(uint8_t cmd, int32_t t_rem, uint8_t rk_flags)
{
    zl_rmp_cmd_t c = {};
    c.t_rem_ms = t_rem;
    c.fire_ms = (uint16_t)P[ZP_RAMPA_FIRE_MS];
    c.cmd = cmd;
    c.seq = ++s_rmp_seq;
    c.rocket_flags = rk_flags;
    c.pyro_ch = (uint8_t)P[ZP_RAMPA_CH];
    c.rocket_state = s_state;
    link_send(Serial2, ZL_PKT_RMP_CMD, &c, sizeof c);
}

// =====================================================================
//  7. OUTPUTS: pyros, camera, buzzer, RGB
// =====================================================================
static void pyro_safe_all(void)
{
    for (int i = 0; i < 3; i++) { digitalWrite(PIN_FIRE[i], LOW); s_fire_end[i] = 0; }
    digitalWrite(PIN_ARM, LOW);
    s_armed = false;
}
static void pyro_arm(bool on)
{
    digitalWrite(PIN_ARM, on ? HIGH : LOW);
    if (s_armed != on) log_event(on ? "PYRO RAIL ARMED" : "PYRO RAIL DISARMED");
    s_armed = on;
}
static void pyro_fire(uint8_t mask, uint32_t now)
{
    for (int i = 0; i < 3; i++) {
        if (!(mask & (1u << i))) continue;
        if (!s_armed) pyro_arm(true);
        digitalWrite(PIN_FIRE[i], HIGH);
        s_fire_end[i] = now + (uint32_t)P[ZP_PYRO_PULSE_MS];
        if (!s_fire_end[i]) s_fire_end[i] = 1;
        s_fired |= (uint8_t)(1u << i);
        s_pyro_events++;
    }
}
static void pyro_service(uint32_t now)
{
    for (int i = 0; i < 3; i++)
        if (s_fire_end[i] && (int32_t)(now - s_fire_end[i]) >= 0) { digitalWrite(PIN_FIRE[i], LOW); s_fire_end[i] = 0; }
}

// RunCam device protocol: 0xCC, CMD_CAMERA_CONTROL (0x01), action, crc8 dvb-s2
static void cam_cmd(uint8_t action)
{
    uint8_t p[4] = { 0xCC, 0x01, action, 0 };
    uint8_t crc = 0;
    for (int i = 0; i < 3; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    p[3] = crc;
    s_cam.write(p, 4);
}
static void cam_power(bool on)
{
    if (on == s_cam_on) return;
    pca_set(PCA_CAM, on ? 1 : 0);
    s_cam_on = on;
    s_cam_rec = false;
    log_event(on ? "CAMERA ON" : "CAMERA OFF");
}
static void cam_record(bool rec)
{
    if (!s_cam_on || rec == s_cam_rec) return;
    cam_cmd(rec ? 0x03 : 0x04);
    s_cam_rec = rec;
    log_event(rec ? "CAMERA REC START" : "CAMERA REC STOP");
}

static void buzzer_service(uint32_t now)
{
    bool on = false;
    if (s_buzz_until && (int32_t)(now - s_buzz_until) < 0) on = true;
    else s_buzz_until = 0;
    switch (s_state) {
    case ZS_BOOT: break;
    case ZS_READY:     if ((now % 5000u) < 60u) on = true; break;
    case ZS_COUNTDOWN: {
        const int32_t rem = (int32_t)(s_t0_ms - now);
        const uint32_t per = rem < 10000 ? 500u : 1000u;
        if (rem > 0 && (now % per) < 80u) on = true;
        break; }
    case ZS_ABORT:     if ((now % 2000u) < 400u && (now % 200u) < 100u) on = true; break;
    case ZS_LANDED:
        if (s_locate && (now % 2000u) < 600u) on = true; break;
    default: break;
    }
    if (s_locate && s_state != ZS_LANDED && (now % 2000u) < 600u) on = true;
    if (zl_state_in_flight(s_state)) on = false;          // keep I2C traffic minimal in flight
    if (s_diag_buzz) on = true;
    pca_set(PCA_BUZZER, on ? 2 : 0);
}

static void rgb_service(uint32_t now)
{
    static uint32_t next = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + 50;
    uint32_t c = 0;
    const bool bl = (now % 1000u) < 500u, fast = (now % 250u) < 125u;
    switch (s_state) {
    case ZS_BOOT:      c = 0x0000FF; break;
    case ZS_IDLE:      c = (s_err == ZE_NONE) ? 0x00FF00 : (bl ? 0xFFA000 : 0); break;
    case ZS_READY:     c = (s_err == ZE_NONE) ? 0x00FFFF : (bl ? 0xFFA000 : 0); break;
    case ZS_COUNTDOWN: c = fast ? 0xFF4000 : 0; break;
    case ZS_ABORT:     c = fast ? 0xFF00FF : 0; break;
    case ZS_DIAG:      c = bl ? 0xFFFFFF : 0x202020; break;
    case ZS_LANDED:    c = bl ? 0x00FF00 : 0; break;
    default:           c = 0xFF0000; break;                  // in flight
    }
    s_rgb.setPixelColor(0, s_rgb.Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF));
    s_rgb.show();
}

// =====================================================================
//  8. SENSORS & GO / NO-GO
// =====================================================================
static void read_discretes(void)
{
    s_rbf_armable = digitalRead(PIN_RBF) == LOW;
    s_bw[0] = digitalRead(PIN_BW1) == LOW;
    s_bw[1] = digitalRead(PIN_BW2) == LOW;
    s_umb_mv = (uint16_t)((uint32_t)analogRead(PIN_UMB_SENSE) * 3300u / 4095u);
    s_umb = s_umb_mv > 1800;              // VIN_5V / 2 > 1.8 V
    for (int i = 0; i < 3; i++) s_cont[i] = s_ads_ok && s_ads_mv[i] >= (uint16_t)P[ZP_CONT_MIN_MV];
}

static inline bool hw_ok(void) { return s_ina_ok && s_ads_ok && s_pca_ok; }

/** First failing launch condition, ZE_NONE when everything is GO. */
static uint8_t gonogo(uint32_t now, bool need_rampa)
{
    if (!nav_fresh(now))                       return ZE_POS_LINK;
    if (!(s_nav.flags & ZL_NAV_VALID))         return ZE_NAV_INVALID;
    if (!hw_ok())                              return ZE_HW;
    if (s_vbat_mv < (uint16_t)(P[ZP_VBAT_MIN_V] * 1000.0f)) return ZE_LOW_BATT;
    if (!s_cont[0])                            return ZE_NO_CONT_DROGUE;
    if (!s_cont[1])                            return ZE_NO_CONT_MAIN;
    if (P[ZP_REQ_CONT3] > 0.5f && !s_cont[2])  return ZE_NO_CONT_BACKUP;
    if (!s_rbf_armable)                        return ZE_RBF_IN;
    if (s_nav.tilt_deg > P[ZP_MAX_TILT_DEG])   return ZE_NOT_VERTICAL;
    if (need_rampa) {
        if (!rampa_online(now))                return ZE_RAMPA_OFFLINE;
        if (!(s_rmp.flags & ZL_RMP_READY))     return ZE_RAMPA_NOT_READY;
    }
    return ZE_NONE;
}

static uint8_t rocket_flags(uint32_t now)
{
    uint8_t f = 0;
    if (gonogo(now, false) == ZE_NONE) f |= ZL_RK_GO;
    if (s_armed) f |= ZL_RK_ARMED;
    if (s_force_bat) f |= ZL_RK_ON_BATT;
    if (nav_ok(now)) f |= ZL_RK_NAV_OK;
    return f;
}

// =====================================================================
//  9. STATE MACHINE
// =====================================================================
static const char *STATE_NAME[ZS_COUNT] = {
    "BOOT", "IDLE", "READY", "COUNTDOWN", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED", "ABORT", "DIAG" };

static void set_state(uint8_t st)
{
    if (st == s_state) return;
    log_event("STATE %s -> %s", STATE_NAME[s_state], STATE_NAME[st]);
    s_state = st;
    s_state_ms = millis();
}

static void force_bat(bool on)
{
    if (on == s_force_bat) return;
    pca_set(PCA_FORCE_BAT, on ? 1 : 0);
    s_force_bat = on;
    log_event(on ? "FORCE_BAT on (battery power)" : "FORCE_BAT off");
}

static void do_abort(uint8_t why)
{
    pyro_safe_all();
    force_bat(false);
    s_cd_active = false;
    s_commit_sent = false;
    s_abort_err = why;
    rampa_send(ZR_CMD_ABORT, -1, 0);
    log_event("ABORT reason %u", why);
    set_state(ZS_ABORT);
}

static void go_idle(void)
{
    pyro_safe_all();
    force_bat(false);
    s_cd_active = false;
    s_commit_sent = false;
    s_abort_err = ZE_NONE;
    rampa_send(ZR_CMD_SAFE, -1, 0);
    set_state(ZS_IDLE);
}

static void start_flight(uint32_t now, const char *why)
{
    zf_begin(&s_fl, now);
    s_fl.state = ZS_BOOST;
    s_fired = 0;
    s_cd_active = false;
    if (!s_armed && s_rbf_armable) pyro_arm(true);
    digitalWrite(PIN_SYNC, HIGH);
    set_state(ZS_BOOST);
    log_event("LIFTOFF (%s) bw %d%d umb %d acc %.1fg", why, s_bw[0], s_bw[1], s_umb, (double)s_nav.acc_axial_g);
}

/** Result of a command from the ground station or the USB console. */
static uint8_t do_command(uint8_t cmd, int32_t arg, uint32_t t_ref_ms)
{
    const uint32_t now = millis();
    const bool flying = zl_state_in_flight(s_state);
    switch (cmd) {
    case ZL_UL_NOP: return ZL_RC_OK;
    case ZL_UL_PREPARE:
        if (flying) return ZL_RC_REJ_FLIGHT;
        if (s_state != ZS_IDLE && s_state != ZS_READY) return ZL_RC_REJ_STATE;
        pos_cmd(ZL_PCMD_PADREF);
        if (P[ZP_CAM_MODE] > 0.5f) cam_power(true);
        s_ready_ms = now;
        s_bw_at_start = (uint8_t)((s_bw[0] ? 1 : 0) | (s_bw[1] ? 2 : 0));
        memset(&s_lift, 0, sizeof s_lift);
        set_state(ZS_READY);
        return ZL_RC_OK;
    case ZL_UL_COUNTDOWN: {
        if (flying) return ZL_RC_REJ_FLIGHT;
        if (s_state != ZS_READY) return ZL_RC_REJ_STATE;
        if (arg < (int32_t)ZL_CD_MIN_MS || arg > (int32_t)ZL_CD_MAX_MS) return ZL_RC_REJ_TOKEN;
        if (P[ZP_EXT_IGN] > 0.5f) return ZL_RC_REJ_STATE;          // external ignition: no countdown
        const uint8_t e = gonogo(now, true);
        if (e == ZE_RAMPA_OFFLINE || e == ZE_RAMPA_NOT_READY) return ZL_RC_REJ_RAMPA;
        if (e != ZE_NONE) return ZL_RC_REJ_NOGO;
        s_t0_ms = t_ref_ms + (uint32_t)arg;
        s_cd_active = true;
        s_commit_sent = false;
        s_bat_check_ms = 0;
        s_bw_at_start = (uint8_t)((s_bw[0] ? 1 : 0) | (s_bw[1] ? 2 : 0));
        memset(&s_lift, 0, sizeof s_lift);
        cam_record(true);
        set_state(ZS_COUNTDOWN);
        log_event("COUNTDOWN T-%ld ms", (long)(int32_t)(s_t0_ms - now));
        return ZL_RC_OK; }
    case ZL_UL_ABORT:
        if (flying) return ZL_RC_REJ_FLIGHT;
        if (s_state == ZS_COUNTDOWN || s_state == ZS_READY) { do_abort(ZE_ABORT_GS); return ZL_RC_ABORTED; }
        return ZL_RC_OK;
    case ZL_UL_SAFE:
        if (flying) return ZL_RC_REJ_FLIGHT;
        if (s_state == ZS_COUNTDOWN) { do_abort(ZE_ABORT_GS); return ZL_RC_ABORTED; }
        if (s_state == ZS_LANDED) { pyro_safe_all(); s_locate = false; return ZL_RC_OK; }
        go_idle();
        return ZL_RC_OK;
    case ZL_UL_PADREF:
        if (s_state != ZS_IDLE && s_state != ZS_READY && s_state != ZS_DIAG) return ZL_RC_REJ_STATE;
        pos_cmd(ZL_PCMD_PADREF);
        return ZL_RC_OK;
    case ZL_UL_MAIN_ALT:
        if (s_state != ZS_IDLE && s_state != ZS_READY && s_state != ZS_DIAG) return ZL_RC_REJ_STATE;
        if (arg < (int32_t)ZP_TABLE[ZP_MAIN_ALT].min || arg > (int32_t)ZP_TABLE[ZP_MAIN_ALT].max) return ZL_RC_REJ_TOKEN;
        P[ZP_MAIN_ALT] = (float)arg;
        params_save();
        log_event("main_alt_m = %ld", (long)arg);
        return ZL_RC_OK;
    case ZL_UL_LOCATE:
        s_locate = arg != 0;
        return ZL_RC_OK;
    case ZL_UL_CAMERA:
        if (flying) return ZL_RC_REJ_FLIGHT;
        if (arg) { cam_power(true); cam_record(true); } else { cam_record(false); }
        return ZL_RC_OK;
    case ZL_UL_REBOOT:
        if (s_state != ZS_IDLE && s_state != ZS_ABORT && s_state != ZS_LANDED && s_state != ZS_DIAG) return ZL_RC_REJ_STATE;
        log_event("REBOOT");
        delay(50);
        watchdog_hw->scratch[0] = 0;
        watchdog_enable(1, true);
        while (1) {}
    default:
        return ZL_RC_REJ_TOKEN;
    }
}

static void handle_uplink(const UplinkMsg &m)
{
    const zl_uplink_t &u = m.u;
    if (u.dest != ZL_UL_DEST_ROCKET) return;
    if (u.seq == s_last_ul_seq && s_last_ul_rc != ZL_RC_NONE) return;      // redundant copy
    uint8_t rc;
    if (u.cmd >= ZL_UL_COUNT || u.token != zl_ul_token_for(u.cmd)) rc = ZL_RC_REJ_TOKEN;
    else {
        // arg is "ms to T-0 at the ground station's TX start": TX start = rx end - airtime
        const uint32_t tx_start = m.rx_ms - zl_lora_airtime_us(sizeof(zl_uplink_t)) / 1000u;
        rc = do_command(u.cmd, u.arg, tx_start);
    }
    s_last_ul_seq = u.seq;
    s_last_ul_rc = rc;
    log_event("UPLINK seq %u cmd %u arg %ld -> rc %u (rssi %d)", u.seq, u.cmd, (long)u.arg, rc, m.rssi);
}

/** Breakwire(s) broken relative to what was intact at PREPARE / countdown start. */
static bool bw_broken_now(void)
{
    const uint8_t bw_now = (uint8_t)((s_bw[0] ? 1 : 0) | (s_bw[1] ? 2 : 0));
    const uint8_t broken = (uint8_t)(s_bw_at_start & ~bw_now);
    if (P[ZP_BW_MODE] > 1.5f) return s_bw_at_start && broken == s_bw_at_start;
    if (P[ZP_BW_MODE] > 0.5f) return broken != 0;
    return false;
}

static void countdown_step(uint32_t now)
{
    const int32_t rem = (int32_t)(s_t0_ms - now);
    s_err = gonogo(now, false);

    // ---- liftoff (before T-0 only a strong accelerometer signal counts) ----
    {
        const bool bw_broken = bw_broken_now();
        const bool after = rem <= 0 || s_rmp.state == ZR_ST_FIRED;
        const bool umb_lost = after && !s_umb && !rampa_online(now);
        if (zf_liftoff(&s_lift, after ? 0 : 1, now, nav_fresh(now), s_nav.acc_axial_g, s_nav.flags,
                       after && bw_broken, umb_lost, P)) {
            start_flight(now, after ? "T0" : "EARLY");
            return;
        }
    }

    if (rem > 0) {
        // ---- rocket health ---------------------------------------------------------
        if (s_err != ZE_NONE) { do_abort(s_err); return; }
        // ---- rampa ------------------------------------------------------------------
        if (!rampa_online(now)) { do_abort(ZE_RAMPA_OFFLINE); return; }
        if (s_rmp.state == ZR_ST_ABORT || s_rmp.state == ZR_ST_FAULT) { do_abort(ZE_ABORT_RAMPA); return; }
        if ((uint32_t)(now - s_state_ms) > 1500u && s_rmp.state != ZR_ST_COUNTDOWN && s_rmp.state != ZR_ST_COMMITTED) {
            do_abort(ZE_RAMPA_NOT_READY); return;
        }
        // ---- T-force_bat: power from our own battery ------------------------------
        if (rem <= (int32_t)(P[ZP_CD_FORCE_BAT_S] * 1000.0f) && !s_force_bat) { force_bat(true); s_bat_check_ms = now + 1000; }
        if (s_bat_check_ms && (int32_t)(now - s_bat_check_ms) >= 0) {
            s_bat_check_ms = 0;
            if (s_curr_ma < 40) { do_abort(ZE_BATT_SWITCH); return; }
        }
        // ---- T-arm: pyro rail -----------------------------------------------------------
        if (rem <= (int32_t)(P[ZP_CD_ARM_S] * 1000.0f) && !s_armed) pyro_arm(true);
        // ---- T-300 ms: commit --------------------------------------------------------------
        if (rem <= (int32_t)ZL_RMP_COMMIT_MS && !s_commit_sent) {
            rampa_send(ZR_CMD_COMMIT, rem, rocket_flags(now));
            s_commit_sent = true;
            log_event("COMMIT sent T-%ld", (long)rem);
        } else if (!s_commit_sent) {
            static uint32_t next = 0;
            if ((int32_t)(now - next) >= 0) { next = now + ZL_RMP_POLL_MS; rampa_send(ZR_CMD_COUNTDOWN, rem, rocket_flags(now)); }
        } else {
            static uint32_t next = 0;
            if ((int32_t)(now - next) >= 0) { next = now + ZL_RMP_POLL_MS; rampa_send(ZR_CMD_COMMIT, rem, rocket_flags(now)); }
            if (rem < 100 && s_rmp.state != ZR_ST_COMMITTED && s_rmp.state != ZR_ST_FIRED) { do_abort(ZE_COMMIT_FAIL); return; }
        }
    } else {
        // after T-0: waiting for liftoff
        if (s_rmp.state == ZR_ST_ABORT && rampa_online(now)) { do_abort(ZE_ABORT_RAMPA); return; }
        if (-rem > (int32_t)(P[ZP_IGN_WAIT_S] * 1000.0f)) {
            log_event("HANGFIRE: no liftoff %.1f s after T-0", (double)(-rem) * 0.001);
            do_abort(ZE_HANGFIRE);
        }
    }
}

static void flight_step(uint32_t now)
{
    zf_in_t in = {};
    in.now_ms = now;
    in.nav_ok = nav_ok(now);
    in.alt = s_nav.alt; in.vz = s_nav.vz; in.acc_axial_g = s_nav.acc_axial_g;
    in.acc_mag_g = s_nav.acc_mag_g; in.mach = s_nav.mach; in.nav_flags = s_nav.flags;
    const uint8_t before = s_fl.state;
    const uint8_t fire = zf_step(&s_fl, &in, P);
    if (fire) {
        pyro_fire(fire, now);
        if (fire & ZF_CH1) log_event("PYRO1 DROGUE (%s) alt %.0f vz %.1f mach %.2f", zf_why_name(s_fl.why_drogue), (double)s_nav.alt, (double)s_nav.vz, (double)s_nav.mach);
        if (fire & ZF_CH2) log_event("PYRO2 MAIN (%s) alt %.0f vz %.1f", zf_why_name(s_fl.why_main), (double)s_nav.alt, (double)s_nav.vz);
        if (fire & ZF_CH3) log_event("PYRO3 BACKUP (%s) alt %.0f vz %.1f", zf_why_name(s_fl.why_p3), (double)s_nav.alt, (double)s_nav.vz);
    }
    if (s_fl.state != before) {
        if (s_fl.state == ZS_COAST) log_event("BURNOUT");
        set_state(s_fl.state);
        if (s_fl.state == ZS_LANDED) log_event("LANDED max alt %.0f m, max vz %.0f m/s, max Mach %.2f, max %.1f g",
                                               (double)s_fl.max_alt, (double)s_fl.max_vz, (double)s_fl.max_mach, (double)s_fl.max_g);
    }
}

static void landed_step(uint32_t now)
{
    const uint32_t t = now - s_state_ms;
    if (t > 10000u && s_armed && !s_fire_end[0] && !s_fire_end[1] && !s_fire_end[2]) pyro_arm(false);
    if (!s_locate && t > (uint32_t)(P[ZP_LOCATE_AFTER_S] * 1000.0f)) s_locate = true;
    if (t > 60000u && s_cam_rec) cam_record(false);
    if (t > 66000u && s_cam_on) cam_power(false);
    if (t > 120000u) s_logging = false;
}

// watchdog scratch: resume a flight after a reset
static const uint32_t SCR_MAGIC = 0x5A4E4D31u;   // "ZNM1"
static void scratch_save(uint32_t now)
{
    if (!zl_state_in_flight(s_state)) { watchdog_hw->scratch[0] = 0; return; }
    watchdog_hw->scratch[1] = (uint32_t)s_state | ((uint32_t)s_fl.fired << 8);
    watchdog_hw->scratch[2] = now - s_fl.t_liftoff;
    const uint32_t td = s_fl.t_drogue ? (now - s_fl.t_drogue) / 100u : 0xFFFFu;
    const uint32_t tm = s_fl.t_main ? (now - s_fl.t_main) / 100u : 0xFFFFu;
    watchdog_hw->scratch[3] = (td > 0xFFFFu ? 0xFFFEu : td) | ((tm > 0xFFFFu ? 0xFFFEu : tm) << 16);
    watchdog_hw->scratch[0] = SCR_MAGIC;
}
static bool scratch_resume(uint32_t now)
{
    if (!watchdog_caused_reboot() || watchdog_hw->scratch[0] != SCR_MAGIC) return false;
    const uint8_t st = (uint8_t)(watchdog_hw->scratch[1] & 0xFF);
    if (!zl_state_in_flight(st)) return false;
    zf_begin(&s_fl, now - watchdog_hw->scratch[2]);
    s_fl.state = st;
    s_fl.fired = (uint8_t)((watchdog_hw->scratch[1] >> 8) & 7);
    s_fired = s_fl.fired;
    const uint32_t td = watchdog_hw->scratch[3] & 0xFFFF, tm = watchdog_hw->scratch[3] >> 16;
    if (td != 0xFFFF) s_fl.t_drogue = now - td * 100u;
    if (tm != 0xFFFF) s_fl.t_main = now - tm * 100u;
    s_state = st;
    s_state_ms = now;
    if (s_rbf_armable) pyro_arm(true);
    digitalWrite(PIN_SYNC, HIGH);
    return true;
}

// =====================================================================
//  10. TELEMETRY SNAPSHOT & LOG RECORD (core 0)
// =====================================================================
static uint8_t health_bits(uint32_t now)
{
    uint8_t h = 0;
    if (nav_fresh(now)) h |= ZL_HL_POS_LINK;
    if (nav_ok(now)) h |= ZL_HL_NAV_VALID;
    if (s_nav.flags & ZL_NAV_BARO_TRUST) h |= ZL_HL_BARO_TRUST;
    if (s_nav.flags & ZL_NAV_GPS_FIX) h |= ZL_HL_GPS_FIX;
    if (s_flash_ok) h |= ZL_HL_FLASH;
    if (hw_ok()) h |= ZL_HL_HW;
    if (s_cam_on) h |= ZL_HL_CAM;
    if (s_locate) h |= ZL_HL_LOCATE;
    return h;
}
static uint8_t pyro_bits(void)
{
    uint8_t p = 0;
    for (int i = 0; i < 3; i++) if (s_cont[i]) p |= (uint8_t)(1u << i);
    p |= (uint8_t)((s_fired & 7u) << 3);
    if (s_armed) p |= 0x40;
    return p;
}
static uint8_t tlm_flags(void)
{
    uint8_t f = 0;
    if (s_armed) f |= ZL_TF_ARMED;
    if (s_rbf_armable) f |= ZL_TF_RBF_ARMABLE;
    if (s_bw[0]) f |= ZL_TF_BW1;
    if (s_bw[1]) f |= ZL_TF_BW2;
    if (s_umb) f |= ZL_TF_UMB;
    if (s_force_bat) f |= ZL_TF_ON_BATT;
    if (s_logging) f |= ZL_TF_LOGGING;
    if (s_err == ZE_NONE) f |= ZL_TF_GO;
    return f;
}

static void tlm_publish(uint32_t now)
{
    zl_tlm_t t = {};
    t.state = s_state;
    t.flags = tlm_flags();
    t.lat_1e7 = s_pos.lat_1e7;
    t.lon_1e7 = s_pos.lon_1e7;
    t.alt_cm = (int32_t)(s_nav.alt * 100.0f);
    if (s_state == ZS_COUNTDOWN) t.t_rem_ms = (int32_t)(s_t0_ms - now);
    else if (zl_state_in_flight(s_state) || s_state == ZS_LANDED) t.t_rem_ms = (int32_t)(now - s_fl.t_liftoff);
    else t.t_rem_ms = -1;
    t.vz_dms = (int16_t)constrain(s_nav.vz * 10.0f, -32000.0f, 32000.0f);
    t.vh_dms = (int16_t)constrain(sqrtf(s_nav.vx * s_nav.vx + s_nav.vy * s_nav.vy) * 10.0f, 0.0f, 32000.0f);
    t.acc_cg = (int16_t)constrain(s_nav.acc_axial_g * 100.0f, -32000.0f, 32000.0f);
    const float ma = zl_state_in_flight(s_state) || s_state == ZS_LANDED ? s_fl.max_alt : s_nav.max_alt;
    t.max_alt_m = (uint16_t)constrain(ma, 0.0f, 65000.0f);
    t.vbat_mv = s_vbat_mv;
    t.main_alt_m = (uint16_t)P[ZP_MAIN_ALT];
    t.tilt_dd = (uint16_t)constrain(s_nav.tilt_deg * 10.0f, 0.0f, 1800.0f);
    t.mach_x100 = (uint8_t)constrain(s_nav.mach * 100.0f, 0.0f, 255.0f);
    t.sats = s_pos.sats;
    t.pyro = pyro_bits();
    t.health = health_bits(now);
    t.rampa_state = rampa_online(now) ? s_rmp.state : (uint8_t)ZR_ST_OFFLINE;
    t.rampa_flags = rampa_online(now) ? s_rmp.flags : 0;
    t.last_ul_seq = s_last_ul_seq;
    t.last_ul_rc = s_last_ul_rc;
    t.err = (s_state == ZS_ABORT) ? s_abort_err : s_err;
    critical_section_enter_blocking(&s_tlm_cs);
    s_tlm_shared = t;
    critical_section_exit(&s_tlm_cs);
    s_tlm_t0_ms = s_t0_ms;
    s_tlm_t0_rem_ref = (s_state == ZS_COUNTDOWN) ? 1 : -1;
    uint8_t hz = (uint8_t)P[ZP_TLM_HZ];
    if (s_state == ZS_LANDED || s_state == ZS_IDLE) hz = hz > 2 ? 2 : hz;
    s_tlm_hz = hz ? hz : 1;
}

static void log_data(uint32_t now)
{
    LogData d = {};
    d.magic = 0xA7; d.state = s_state; d.flags = tlm_flags(); d.pyro = pyro_bits();
    d.t_ms = now;
    d.alt = s_nav.alt; d.vz = s_nav.vz; d.baro_alt = s_nav.baro_alt; d.acc_axial = s_nav.acc_axial_g;
    d.acc_mag = s_nav.acc_mag_g; d.mach = s_nav.mach; d.tilt = s_nav.tilt_deg;
    d.lat = s_pos.lat_1e7; d.lon = s_pos.lon_1e7;
    d.vbat_mv = s_vbat_mv; d.curr_ma = s_curr_ma; d.nav_flags = s_nav.flags;
    d.px = (int16_t)constrain(s_nav.px, -32000.0f, 32000.0f);
    d.py = (int16_t)constrain(s_nav.py, -32000.0f, 32000.0f);
    d.max_alt = (uint16_t)constrain(s_fl.max_alt, 0.0f, 65000.0f);
    d.rampa_state = s_rmp.state; d.err = s_err; d.sats = s_pos.sats; d.nav_mode = s_nav.mode;
    d.vh_dms = (int16_t)constrain(sqrtf(s_nav.vx * s_nav.vx + s_nav.vy * s_nav.vy) * 10.0f, 0.0f, 32000.0f);
    d.crc = zl_crc16((const uint8_t *)&d, 62);
    log_push((const uint8_t *)&d);
}

// =====================================================================
//  11. USB DIAGNOSTIC CONSOLE (core 0)
//      Text lines, 115200 (USB CDC). Protocol in docs/USB_PROTOCOL.md.
// =====================================================================
static void usb_status(uint32_t now)
{
    usb_printf("$S,%u,%u,%u,%u,%lu,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%u",
        s_state, (s_state == ZS_ABORT) ? s_abort_err : s_err, tlm_flags(), health_bits(now), (unsigned long)now,
        s_vbat_mv, s_curr_ma, pyro_bits(), s_ads_mv[0], s_ads_mv[1], s_ads_mv[2], s_ads_mv[3],
        rampa_online(now) ? s_rmp.state : 0, s_rmp.flags, s_rmp.faults, s_rmp.vbat_mv,
        (long)(s_state == ZS_COUNTDOWN ? (int32_t)(s_t0_ms - now) : -1),
        s_bw[0], s_bw[1], s_umb, s_rbf_armable, s_umb_mv, s_lora_ok, s_flash_ok, (unsigned)P[ZP_MAIN_ALT],
        (unsigned long)s_log_head, (unsigned long)s_log_dropped, s_log_erasing, s_reset_reason);
}
static void usb_nav(void)
{
    usb_printf("$N,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.4f,%.4f,%.4f,%.4f,%.1f,%u,%u",
        (double)s_nav.alt, (double)s_nav.vz, (double)s_nav.px, (double)s_nav.py, (double)s_nav.vx, (double)s_nav.vy,
        (double)s_nav.acc_axial_g, (double)s_nav.acc_mag_g, (double)s_nav.mach, (double)s_nav.baro_alt,
        (double)s_nav.sig_alt, (double)s_nav.tilt_deg, (double)s_nav.qw, (double)s_nav.qx, (double)s_nav.qy,
        (double)s_nav.qz, (double)s_nav.max_alt, s_nav.flags, s_nav.mode);
}
static void usb_pos(void)
{
    usb_printf("$P,%.7f,%.7f,%.1f,%u,%u,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%u,%u,%u,%lu,%u.%02u,%u,%.7f,%.7f",
        s_pos.lat_1e7 * 1e-7, s_pos.lon_1e7 * 1e-7, s_pos.gps_alt_cm * 0.01, s_pos.sats, s_pos.fix_type,
        s_pos.h_acc_dm * 0.1, (double)s_pos.pressure_pa, (double)s_pos.baro_temp_c,
        (double)s_pos.roll, (double)s_pos.pitch, (double)s_pos.yaw,
        (double)s_pos.gyro_bias[0], (double)s_pos.gyro_bias[1], (double)s_pos.gyro_bias[2],
        (double)s_pos.peak_g, s_pos.imu_hz, s_pos.sensors, s_pos.nose_axis, (unsigned long)s_pos.imu_lost,
        s_pos.fw_major, s_pos.fw_minor, s_pos.reset_reason, (double)s_pos.pad_lat, (double)s_pos.pad_lon);
}
static void usb_raw(void)
{
    usb_printf("$R,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        (double)s_raw.a[0], (double)s_raw.a[1], (double)s_raw.a[2], (double)s_raw.g[0], (double)s_raw.g[1], (double)s_raw.g[2],
        (double)s_raw.m[0], (double)s_raw.m[1], (double)s_raw.m[2], (double)s_raw.hg_weight);
}
static void usb_flight(void)
{
    usb_printf("$F,%u,%u,%u,%u,%.1f,%.1f,%.2f,%.1f,%u",
        s_fl.why_drogue, s_fl.why_main, s_fl.why_p3, s_fired,
        (double)s_fl.max_alt, (double)s_fl.max_vz, (double)s_fl.max_mach, (double)s_fl.max_g, s_pyro_events);
}

static int param_index(const char *name)
{
    for (int i = 0; i < ZP_COUNT; i++) if (!strcmp(name, ZP_TABLE[i].name)) return i;
    return -1;
}

static void log_dump(void)
{
    if (!flash_acquire(3000)) { usb_line("#ERR flash busy"); return; }
    const uint32_t n = s_log_head;
    usb_printf("$LOGBEGIN,%lu", (unsigned long)n);
    uint8_t b[64];
    char line[140];
    static const char hx[] = "0123456789ABCDEF";
    for (uint32_t i = 0; i < n; i++) {
        fl_read(i * LOG_REC, b, 64);
        memcpy(line, "$LOG,", 5);
        for (int k = 0; k < 64; k++) { line[5 + 2 * k] = hx[b[k] >> 4]; line[6 + 2 * k] = hx[b[k] & 15]; }
        line[133] = 0;
        while (Serial.availableForWrite() < 140) { watchdog_update(); if (!Serial) break; }
        if (!Serial) break;
        Serial.write(line, 133); Serial.write("\r\n");
        if ((i & 63u) == 0) watchdog_update();
    }
    flash_release();
    usb_printf("$LOGEND,%lu", (unsigned long)n);
}

static bool diag_allowed(void) { return s_state == ZS_DIAG; }

static void usb_command(char *line)
{
    const uint32_t now = millis();
    char *argv[6] = { 0 };
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 6; t = strtok(nullptr, " \t")) argv[argc++] = t;
    if (!argc) return;
    for (char *p = argv[0]; *p; p++) *p = (char)toupper(*p);
    const char *c = argv[0];
    const bool flying = zl_state_in_flight(s_state);

    if (!strcmp(c, "HELLO")) {
        usb_printf("$ID,Zenith Master,%u.%02u,%u", ZENITH_FW_VERSION / 100, ZENITH_FW_VERSION % 100, ZP_COUNT);
    } else if (!strcmp(c, "PARAM") && argc >= 2) {
        for (char *p = argv[1]; *p; p++) *p = (char)toupper(*p);
        if (!strcmp(argv[1], "LIST")) {
            for (int i = 0; i < ZP_COUNT; i++)
                usb_printf("$PRM,%d,%s,%g,%g,%g,%g,%s", i, ZP_TABLE[i].name, (double)P[i], (double)ZP_TABLE[i].min,
                           (double)ZP_TABLE[i].max, (double)ZP_TABLE[i].def, ZP_TABLE[i].desc);
            usb_line("#OK PARAM LIST");
        } else if (!strcmp(argv[1], "SET") && argc >= 4) {
            const int i = param_index(argv[2]);
            const float v = strtof(argv[3], nullptr);
            if (flying || s_state == ZS_COUNTDOWN) usb_line("#ERR not now");
            else if (i < 0) usb_line("#ERR unknown parameter");
            else if (!(v >= ZP_TABLE[i].min && v <= ZP_TABLE[i].max)) usb_line("#ERR out of range");
            else { P[i] = v; usb_printf("#OK %s=%g", ZP_TABLE[i].name, (double)v);
                   usb_printf("$PRM,%d,%s,%g,%g,%g,%g,%s", i, ZP_TABLE[i].name, (double)P[i], (double)ZP_TABLE[i].min,
                              (double)ZP_TABLE[i].max, (double)ZP_TABLE[i].def, ZP_TABLE[i].desc); }
        } else if (!strcmp(argv[1], "SAVE")) {
            usb_line(params_save() ? "#OK PARAM SAVE" : "#ERR save failed");
        } else if (!strcmp(argv[1], "DEFAULTS")) {
            if (flying || s_state == ZS_COUNTDOWN) usb_line("#ERR not now");
            else { params_defaults(); usb_line("#OK defaults loaded (PARAM SAVE to keep)"); }
        } else usb_line("#ERR PARAM LIST|SET n v|SAVE|DEFAULTS");
    } else if (!strcmp(c, "PREPARE"))   { usb_printf("#RC PREPARE %u", do_command(ZL_UL_PREPARE, 0, now)); }
    else if (!strcmp(c, "COUNTDOWN") && argc >= 2) {
        usb_printf("#RC COUNTDOWN %u", do_command(ZL_UL_COUNTDOWN, (int32_t)(atof(argv[1]) * 1000.0), now));
    } else if (!strcmp(c, "ABORT"))     { usb_printf("#RC ABORT %u", do_command(ZL_UL_ABORT, 0, now)); }
    else if (!strcmp(c, "SAFE"))        { usb_printf("#RC SAFE %u", do_command(ZL_UL_SAFE, 0, now)); }
    else if (!strcmp(c, "PADREF"))      { usb_printf("#RC PADREF %u", do_command(ZL_UL_PADREF, 0, now)); }
    else if (!strcmp(c, "LOCATE") && argc >= 2) { usb_printf("#RC LOCATE %u", do_command(ZL_UL_LOCATE, atoi(argv[1]), now)); }
    else if (!strcmp(c, "CAM") && argc >= 2) { usb_printf("#RC CAM %u", do_command(ZL_UL_CAMERA, atoi(argv[1]), now)); }
    else if (!strcmp(c, "GYROCAL"))     { if (flying) usb_line("#ERR in flight"); else { pos_cmd(ZL_PCMD_GYRO_CAL); usb_line("#OK GYROCAL (keep still 1 s)"); } }
    else if (!strcmp(c, "GPSRECFG"))    { pos_cmd(ZL_PCMD_GPS_RECFG); usb_line("#OK"); }
    else if (!strcmp(c, "RAW") && argc >= 2) { s_raw_stream = atoi(argv[1]) != 0; usb_line("#OK RAW"); }
    else if (!strcmp(c, "DIAG") && argc >= 2) {
        const bool on = atoi(argv[1]) != 0;
        if (on && (s_state == ZS_IDLE || s_state == ZS_ABORT)) { pyro_safe_all(); set_state(ZS_DIAG); usb_line("#OK DIAG ON"); }
        else if (!on && s_state == ZS_DIAG) { pyro_safe_all(); force_bat(false); s_diag_buzz = false; pca_set(PCA_SERVO_PWR, 0); go_idle(); usb_line("#OK DIAG OFF"); }
        else usb_line("#ERR DIAG only from IDLE/ABORT");
    } else if (!strcmp(c, "ARM") && argc >= 2) {
        if (!diag_allowed()) usb_line("#ERR DIAG mode required");
        else { pyro_arm(atoi(argv[1]) != 0); usb_line("#OK ARM"); }
    } else if (!strcmp(c, "FIRE") && argc >= 3) {
        const int ch = atoi(argv[1]);
        if (!diag_allowed()) usb_line("#ERR DIAG mode required");
        else if (ch < 1 || ch > 3 || strcmp(argv[2], "CONFIRM")) usb_line("#ERR FIRE <1..3> CONFIRM");
        else if (!s_rbf_armable) usb_line("#ERR RBF is in (SAFE) - rail cannot be armed");
        else { pyro_fire((uint8_t)(1u << (ch - 1)), now); s_fired = 0; log_event("DIAG FIRE CH%d", ch); usb_line("#OK FIRE"); }
    } else if (!strcmp(c, "BUZZ") && argc >= 2) {
        s_buzz_until = now + (uint32_t)constrain(atoi(argv[1]), 0, 5000); usb_line("#OK BUZZ");
    } else if (!strcmp(c, "FORCEBAT") && argc >= 2) {
        if (!diag_allowed()) usb_line("#ERR DIAG mode required");
        else { force_bat(atoi(argv[1]) != 0); usb_line("#OK FORCEBAT"); }
    } else if (!strcmp(c, "SERVOPWR") && argc >= 2) {
        if (!diag_allowed()) usb_line("#ERR DIAG mode required");
        else { pca_set(PCA_SERVO_PWR, atoi(argv[1]) ? 1 : 0); usb_line("#OK SERVOPWR"); }
    } else if (!strcmp(c, "RAMPA") && argc >= 2) {
        for (char *p = argv[1]; *p; p++) *p = (char)toupper(*p);
        if (!strcmp(argv[1], "PING")) { rampa_send(ZR_CMD_PING, -1, rocket_flags(now)); usb_line("#OK"); }
        else usb_line("#ERR RAMPA PING");
    } else if (!strcmp(c, "LOG") && argc >= 2) {
        for (char *p = argv[1]; *p; p++) *p = (char)toupper(*p);
        if (!strcmp(argv[1], "INFO")) usb_printf("$LOGINFO,%lu,%lu,%lu,%u,%lu", (unsigned long)s_log_head,
                                                (unsigned long)LOG_NREC, (unsigned long)s_log_dropped, s_log_erasing, (unsigned long)s_flash_jedec);
        else if (!strcmp(argv[1], "DUMP")) { if (flying || s_state == ZS_COUNTDOWN) usb_line("#ERR not now"); else log_dump(); }
        else if (!strcmp(argv[1], "ERASE") && argc >= 3 && !strcmp(argv[2], "CONFIRM")) {
            if (s_state != ZS_IDLE && s_state != ZS_DIAG && s_state != ZS_LANDED) usb_line("#ERR only in IDLE/DIAG/LANDED");
            else { s_log_erase_req = true; usb_line("#OK erasing (watch $S log fields)"); }
        } else usb_line("#ERR LOG INFO|DUMP|ERASE CONFIRM");
    } else if (!strcmp(c, "POSREBOOT")) {
        if (flying) usb_line("#ERR in flight"); else { pos_cmd(ZL_PCMD_REBOOT, 0xB0); usb_line("#OK"); }
    } else if (!strcmp(c, "REBOOT")) {
        usb_printf("#RC REBOOT %u", do_command(ZL_UL_REBOOT, 0, now));
    } else if (!strcmp(c, "FLIGHT")) {
        usb_flight();
    } else {
        usb_line("#ERR unknown command");
    }
}

static void usb_poll(uint32_t now)
{
    static char line[96];
    static uint8_t n = 0;
    int budget = 128;
    while (Serial.available() && budget-- > 0) {
        const char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') { if (n) { line[n] = 0; usb_command(line); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = ch;
    }
    if (!Serial) return;
    static uint32_t t_s = 0, t_n = 0, t_p = 0, t_r = 0, t_f = 0;
    if ((int32_t)(now - t_s) >= 0) { t_s = now + 100; usb_status(now); }
    if ((int32_t)(now - t_n) >= 0) { t_n = now + 50;  if (s_nav_ms) usb_nav(); }
    if ((int32_t)(now - t_p) >= 0) { t_p = now + 200; if (s_pos_ms) usb_pos(); }
    if ((int32_t)(now - t_f) >= 0) { t_f = now + 1000; usb_flight(); }
    if (s_raw_stream && (int32_t)(now - t_r) >= 0) { t_r = now + 40; if (s_raw_ms) usb_raw(); }
}

// =====================================================================
//  12. CORE 1: LoRa TDMA and the flash log writer
// =====================================================================
static uint32_t log_find_head(void)
{
    uint32_t lo = 0, hi = LOG_NREC;           // first record whose magic byte is 0xFF
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo) / 2;
        uint8_t b; fl_read(mid * LOG_REC, &b, 1);
        if (b == 0xFF) hi = mid; else lo = mid + 1;
    }
    return lo;
}

static void core1_flash_service(void)
{
    static uint32_t erase_addr = 0;
    if (!s_flash_ok) { LogRec r; while (queue_try_remove(&s_log_q, &r)) {} return; }
    if (s_flash_pause_req) {
        if (fl_busy()) return;
        s_flash_paused = true;
        while (s_flash_pause_req) tight_loop_contents();
        s_flash_paused = false;
        return;
    }
    if (s_log_erase_req && !s_log_erasing) { s_log_erase_req = false; s_log_erasing = true; erase_addr = 0; }
    if (s_log_erasing) {
        if (fl_busy()) return;
        if (erase_addr >= LOG_END) { s_log_erasing = false; s_log_head = 0; return; }
        fl_erase_cmd(0xD8, erase_addr);               // 64 kB block (params live outside)
        erase_addr += 65536u;
        if (erase_addr > LOG_END) erase_addr = LOG_END;
        LogRec r; while (queue_try_remove(&s_log_q, &r)) {}
        return;
    }
    // write queued records, one per call (each ~0.4 ms busy)
    if (fl_busy()) return;
    LogRec r;
    if (!queue_try_remove(&s_log_q, &r)) return;
    if (s_log_head >= LOG_NREC) { s_log_dropped++; return; }
    fl_program(s_log_head * LOG_REC, r.b, LOG_REC);
    s_log_head++;
}

static bool lora_in_window(uint32_t now, int32_t t_rem)
{
    if ((now % ZL_WIN_PERIOD_MS) < ZL_WIN_DUR_MS) {
        // a regular window — unless it was moved away from the last 1.6 s before T-0
        if (t_rem > 0) {
            const int32_t at = t_rem - 0;
            if (at < 1600 && at > -2000) return false;
        }
        return true;
    }
    if (t_rem > 0 && t_rem <= (int32_t)ZL_FINAL_WIN_OPEN_MS && t_rem > (int32_t)(ZL_FINAL_WIN_OPEN_MS - ZL_WIN_DUR_MS)) return true;
    return false;
}

void setup1()
{
    while (!s_core0_ready) delay(1);
    SPI.setSCK(PIN_LORA_SCK);
    SPI.setTX(PIN_LORA_MOSI);
    SPI.setRX(PIN_LORA_MISO);
    LoRa.setSPI(SPI);
    LoRa.setPins(PIN_LORA_CS, -1, PIN_LORA_DIO0);
    if (LoRa.begin(ZL_LORA_FREQ_HZ)) {
        LoRa.setSpreadingFactor(ZL_LORA_SF);
        LoRa.setSignalBandwidth(ZL_LORA_BW_HZ);
        LoRa.setCodingRate4(ZL_LORA_CR4);
        LoRa.setPreambleLength(ZL_LORA_PREAMBLE);
        LoRa.setSyncWord(ZL_LORA_SYNCWORD);
        LoRa.setTxPower(ZL_LORA_TX_DBM);
        LoRa.enableCrc();
        LoRa.idle();
        s_lora_ok = true;
    }
    if (s_flash_ok) s_log_head = log_find_head();
}

void loop1()
{
    static uint32_t next_tx = 0;
    static bool rx_mode = false, ack_due = false;
    const uint32_t now = millis();
    s_core1_alive_ms = now;

    core1_flash_service();
    if (!s_lora_ok) { delay(1); return; }

    const int32_t t_rem = (s_tlm_t0_rem_ref > 0) ? (int32_t)(s_tlm_t0_ms - now) : -1;
    if (lora_in_window(now, t_rem)) {
        if (!rx_mode) { LoRa.receive(); rx_mode = true; }
        const int sz = LoRa.parsePacket();
        if (sz > 0) {
            uint8_t b[32]; int n = 0;
            while (LoRa.available() && n < (int)sizeof b) b[n++] = (uint8_t)LoRa.read();
            while (LoRa.available()) LoRa.read();
            if (zl_lora_check(b, (size_t)n, sizeof(zl_uplink_t), ZL_UL_MAGIC)) {
                UplinkMsg m;
                memcpy(&m.u, b, sizeof m.u);
                m.rx_ms = millis();
                m.rssi = (int16_t)LoRa.packetRssi();
                s_lora_rssi = m.rssi;
                s_lora_rx_count++;
                queue_try_add(&s_ul_q, &m);
                ack_due = true;
            }
        }
        return;
    }
    if (rx_mode) { LoRa.idle(); rx_mode = false; ack_due = true; }

    // ---- downlink ------------------------------------------------------------------
    int fin = 0;
    const uint32_t to_win = zl_next_window_ms(now, t_rem, &fin);
    const uint32_t air = zl_lora_airtime_us(sizeof(zl_tlm_t)) / 1000u + 2u;
    const bool slot_free = to_win > air + ZL_WIN_TX_GUARD_MS / 2u;
    if (slot_free && (ack_due || (int32_t)(now - next_tx) >= 0)) {
        if (ack_due) delay(15);          // let core 0 process the command, so the ACK carries its result
        zl_tlm_t t;
        critical_section_enter_blocking(&s_tlm_cs);
        t = s_tlm_shared;
        critical_section_exit(&s_tlm_cs);
        const uint32_t ts = millis();
        int fin2 = 0;
        const int32_t rem2 = (s_tlm_t0_rem_ref > 0) ? (int32_t)(s_tlm_t0_ms - ts) : -1;
        const uint32_t nw = zl_next_window_ms(ts, rem2, &fin2);
        t.t_ms = ts;
        t.next_rx_in_ms = (uint16_t)(nw > 65000u ? ZL_NEXT_RX_NONE : nw);
        t.win_final = (uint8_t)fin2;
        zl_lora_seal(&t, sizeof t, ZL_TLM_MAGIC);
        if (LoRa.beginPacket()) {
            LoRa.write((const uint8_t *)&t, sizeof t);
            LoRa.endPacket(false);
            s_lora_tx_count++;
        }
        ack_due = false;
        next_tx = ts + 1000u / (s_tlm_hz ? s_tlm_hz : 1);
    }
}

// =====================================================================
//  13. SETUP / LOOP (core 0)
// =====================================================================
void setup()
{
    // FIRST: every pyro output low, before anything can delay.
    for (int i = 0; i < 3; i++) { pinMode(PIN_FIRE[i], OUTPUT); digitalWrite(PIN_FIRE[i], LOW); }
    pinMode(PIN_ARM, OUTPUT); digitalWrite(PIN_ARM, LOW);
    pinMode(PIN_SYNC, OUTPUT); digitalWrite(PIN_SYNC, LOW);
    pinMode(PIN_RBF, INPUT);
    pinMode(PIN_BW1, INPUT);
    pinMode(PIN_BW2, INPUT);
    pinMode(PIN_FL_CS, OUTPUT); digitalWrite(PIN_FL_CS, HIGH);
    analogReadResolution(12);
    s_reset_reason = watchdog_caused_reboot() ? 1 : 0;

    Serial.begin(115200);
    queue_init(&s_ul_q, sizeof(UplinkMsg), 8);
    queue_init(&s_log_q, sizeof(LogRec), 256);
    critical_section_init(&s_tlm_cs);
    params_defaults();

    s_rgb.begin(); s_rgb.setBrightness(60); s_rgb.setPixelColor(0, s_rgb.Color(0, 0, 255)); s_rgb.show();

    // links
    Serial1.setTX(PIN_LINK_TX); Serial1.setRX(PIN_LINK_RX);
    Serial1.setFIFOSize(2048);
    Serial1.begin(ZL_LINK_BAUD);
    Serial2.setTX(PIN_UMB_TX); Serial2.setRX(PIN_UMB_RX);
    Serial2.setFIFOSize(512);
    Serial2.begin(ZL_UMB_BAUD);
    zl_rx_init(&s_link_rx);
    zl_rx_init(&s_umb_rx);
    s_cam.begin(115200);

    // I2C peripherals
    Wire1.setSDA(PIN_I2C_SDA); Wire1.setSCL(PIN_I2C_SCL);
    Wire1.begin(); Wire1.setClock(400000); Wire1.setTimeout(5);
    s_pca_ok = pca_begin();
    s_ina_ok = ina_begin();
    // LoRa reset pulse through the PCA9685 (LED8 low = reset), then release
    pca_set(PCA_LORA_RST, 0); delay(2); pca_set(PCA_LORA_RST, 1); delay(10);

    // flash + parameters (core 1 is not running its loop yet)
    SPI1.setSCK(PIN_FL_SCK); SPI1.setTX(PIN_FL_MOSI); SPI1.setRX(PIN_FL_MISO);
    SPI1.begin();
    s_flash_jedec = fl_jedec();
    s_flash_ok = (s_flash_jedec >> 16) == 0xEF;       // Winbond
    if (s_flash_ok) params_load_raw();

    read_discretes();
    const uint32_t now = millis();
    s_state_ms = now;
    if (scratch_resume(now)) {
        s_logging = true;
        s_core0_ready = true;
        watchdog_enable(WDT_MS, true);
        log_event("RESUMED IN FLIGHT state %s after watchdog reset", STATE_NAME[s_state]);
        return;
    }
    s_logging = true;
    s_core0_ready = true;
    watchdog_enable(WDT_MS, true);
    log_event("BOOT fw %u.%02u flash %06lX params seq %u reset %u", ZENITH_FW_VERSION / 100, ZENITH_FW_VERSION % 100,
              (unsigned long)s_flash_jedec, s_param_seq, s_reset_reason);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    link_poll(now);
    pyro_service(now);

    // ---- uplink commands from core 1 -------------------------------------------
    UplinkMsg m;
    while (queue_try_remove(&s_ul_q, &m)) handle_uplink(m);

    // ---- I2C sensors (ADS every pass, INA 20 Hz) ------------------------------------
    static uint32_t t_ina = 0;
    ads_poll();
    if ((int32_t)(now - t_ina) >= 0) { t_ina = now + 50; ina_read(); if (!s_pca_ok) s_pca_ok = pca_begin(); }
    read_discretes();
    link_poll(now);

    // ---- state machine --------------------------------------------------------------
    switch (s_state) {
    case ZS_BOOT:
        s_err = gonogo(now, false);
        if (now > 3000u && (nav_fresh(now) || now > 8000u)) set_state(ZS_IDLE);
        break;
    case ZS_IDLE:
        s_err = gonogo(now, P[ZP_EXT_IGN] < 0.5f);
        break;
    case ZS_READY:
        s_err = gonogo(now, P[ZP_EXT_IGN] < 0.5f);
        if (P[ZP_EXT_IGN] > 0.5f &&
            zf_liftoff(&s_lift, 2, now, nav_fresh(now), s_nav.acc_axial_g, s_nav.flags,
                       bw_broken_now(), !s_umb, P)) {
            start_flight(now, "EXT");
            break;
        }
        if ((uint32_t)(now - s_ready_ms) > (uint32_t)(P[ZP_READY_TIMEOUT_MIN] * 60000.0f)) { log_event("READY timeout"); go_idle(); }
        break;
    case ZS_COUNTDOWN:
        countdown_step(now);
        break;
    case ZS_BOOST: case ZS_COAST: case ZS_DROGUE: case ZS_MAIN:
        flight_step(now);
        break;
    case ZS_LANDED:
        landed_step(now);
        break;
    case ZS_ABORT:
    case ZS_DIAG:
        s_err = gonogo(now, false);
        break;
    default: break;
    }

    // ---- rampa heartbeat (outside the countdown) -----------------------------------------
    static uint32_t t_rmp = 0;
    if (s_state != ZS_COUNTDOWN && !zl_state_in_flight(s_state) && s_state != ZS_LANDED && (int32_t)(now - t_rmp) >= 0) {
        t_rmp = now + ZL_RMP_POLL_MS;
        uint8_t cmd = ZR_CMD_PING;
        if (s_state == ZS_READY) cmd = ZR_CMD_PREPARE;
        if (s_state == ZS_ABORT) cmd = ZR_CMD_ABORT;
        rampa_send(cmd, -1, rocket_flags(now));
    }

    // ---- to the Position MCU --------------------------------------------------------------
    static uint32_t t_mst = 0;
    static uint8_t last_st = 0xFF;
    if ((int32_t)(now - t_mst) >= 0 || last_st != s_state) {
        t_mst = now + 50;
        last_st = s_state;
        zl_mst_state_t ms = {};
        ms.t_flight_ms = zl_state_in_flight(s_state) ? now - s_fl.t_liftoff : 0;
        ms.state = s_state;
        if (zl_state_in_flight(s_state) || s_state == ZS_LANDED) ms.flags |= ZL_MST_LAUNCHED;
        if (s_state == ZS_COUNTDOWN) ms.flags |= ZL_MST_COUNTDOWN;
        if (s_raw_stream) ms.flags |= ZL_MST_RAW_STREAM;
        if (s_state == ZS_IDLE || s_state == ZS_READY || s_state == ZS_COUNTDOWN || s_state == ZS_ABORT || s_state == ZS_DIAG)
            ms.flags |= ZL_MST_ON_PAD;
        ms.pyro_events = s_pyro_events;
        ms.nose_axis = (uint8_t)P[ZP_NOSE_AXIS];
        link_send(Serial1, ZL_PKT_MST_STATE, &ms, sizeof ms);
    }
    static float last_decl = -999;
    if (P[ZP_DECL_DEG] != last_decl && nav_fresh(now)) { last_decl = P[ZP_DECL_DEG]; pos_cmd(ZL_PCMD_MAG_DECL, 0, last_decl); }

    // ---- telemetry snapshot, log, scratch --------------------------------------------------
    static uint32_t t_tlm = 0, t_log = 0, t_scr = 0;
    if ((int32_t)(now - t_tlm) >= 0) { t_tlm = now + 50; tlm_publish(now); }
    const uint32_t log_per = zl_state_in_flight(s_state) ? 10u :
                             (s_state == ZS_COUNTDOWN || s_state == ZS_LANDED) ? 50u : 1000u;
    if ((int32_t)(now - t_log) >= 0) { t_log = now + log_per; log_data(now); }
    if ((int32_t)(now - t_scr) >= 0) { t_scr = now + 100; scratch_save(now); }

    // ---- annunciators & console -------------------------------------------------------------
    buzzer_service(now);
    rgb_service(now);
    usb_poll(now);
}
