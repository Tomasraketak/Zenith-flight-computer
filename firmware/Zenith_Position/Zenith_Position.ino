// =====================================================================
//  ZENITH v2.4 — POSITION MCU (U16, Seeed XIAO RP2350)
//
//  "Where am I?" — and nothing else. Owns every navigation sensor, runs the
//  estimator (zenith_nav.h) and streams the solution to the Master.
//
//  Board : Seeed XIAO RP2350 (arduino-pico core, rp2040:rp2040:seeed_xiao_rp2350)
//  Needs : Voltino_TriSense_V2 (github.com/Tomasraketak/VoltinoTrisenseV2),
//          Adafruit NeoPixel
//
//  Pinout (zenith_netlist_v2_4.txt, BLOCK 7/8/10 — see docs/PINOUT.md):
//    LSM6DSV32X   SPI0  SCK GP2  MOSI GP3  MISO GP4  CS GP5   INT1 GP27
//    LSM6DSV320X  SPI1  SCK GP10 MOSI GP11 MISO GP12 CS GP9   INT1 GP28
//    BMP580 0x46 + MMC5983MA 0x30   I2C1 SDA GP6 SCL GP7      BARO_INT GP26
//    MAX-M10S GPS  UART1 (Serial2)  TX GP20 -> GPS RXD, RX GP21 <- GPS TXD, PPS GP16
//    Master link   UART0 (Serial1)  TX GP0 -> R76 -> Pico GP1, RX GP1 <- R75 <- Pico GP0
//    XIAO_SYNC     GP17 <- R77 <- Pico GP26 (Master drives HIGH from liftoff)
//
//  Cores:
//    core 0  IMU FIFO, estimator, barometer, magnetometer, Master link  (never blocks)
//    core 1  GPS: UBX configuration and NAV-PVT parsing                 (may block)
// =====================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TriSense.h>
#include <TriSenseZenith.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/watchdog.h"
#include "pico/critical_section.h"

#include "zenith_link.h"
#include "zenith_nav.h"

// ---------------------------------------------------------------- pins
static const uint8_t PIN_LINK_TX   = 0;    // UART0 TX -> Master
static const uint8_t PIN_LINK_RX   = 1;    // UART0 RX <- Master
static const uint8_t PIN_GPS_TX    = 20;   // UART1 TX -> GPS RXD
static const uint8_t PIN_GPS_RX    = 21;   // UART1 RX <- GPS TXD
static const uint8_t PIN_GPS_PPS   = 16;
static const uint8_t PIN_XIAO_SYNC = 17;   // HIGH = Master says "in flight"
#ifdef PIN_NEOPIXEL
static const uint8_t PIN_RGB_LED       = PIN_NEOPIXEL;
#else
static const uint8_t PIN_RGB_LED       = 22;
#endif
#ifdef NEOPIXEL_POWER
static const int8_t  PIN_LED_PWR   = NEOPIXEL_POWER;
#else
static const int8_t  PIN_LED_PWR   = 23;
#endif

static const uint32_t GPS_RATE_MS  = 200;  // 5 Hz NAV-PVT (safe for MAX-M10S with 4 GNSS)
static const uint32_t WDT_MS       = 500;

// ---------------------------------------------------------------- objects
static TriSense          s_sens;
static zn_t              s_nav;
static Adafruit_NeoPixel s_led(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);
static zl_rx_t           s_rx;

static bool     s_imu_ok = false, s_hg_ok = false, s_baro_ok = false, s_mag_ok = false;
static float    s_imu_dt = 1.0f / 960.0f;
static uint32_t s_imu_count = 0, s_imu_hz = 0, s_imu_lost = 0;
static zl_mst_state_t s_mst = {};
static uint32_t s_mst_rx_ms = 0;
static float    s_last_m[3] = {0, 0, 0};
static float    s_last_a[3] = {0, 0, 0}, s_last_g[3] = {0, 0, 0};
static float    s_hg_weight = 0;
static float    s_peak_g = 0;
static float    s_pressure = 0, s_baro_t = 0;
static uint8_t  s_reset_reason = 0;
static volatile bool s_core0_ready = false;

// watchdog scratch: survive a reset in flight
static const uint32_t SCR_MAGIC = 0x5A4E5031u;   // "ZNP1"

// =====================================================================
//  GPS (core 1) — u-blox M10 UBX
// =====================================================================
struct GpsShared {
    zn_gps_t fix;
    uint32_t rx_ms;
    uint32_t tow_ms;
    float    pad_lat, pad_lon;
    uint32_t last_ubx_ms;
    bool     cfg_ack;
};
static critical_section_t s_gps_cs;
static GpsShared s_gps = {};
static volatile bool s_gps_recfg = false;

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t n)
{
    uint8_t hdr[6] = { 0xB5, 0x62, cls, id, (uint8_t)(n & 0xFF), (uint8_t)(n >> 8) };
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++) { a += hdr[i]; b += a; }
    for (uint16_t i = 0; i < n; i++) { a += pl[i]; b += a; }
    Serial2.write(hdr, 6);
    if (n) Serial2.write(pl, n);
    Serial2.write(a); Serial2.write(b);
    Serial2.flush();
}

static uint16_t valset_put(uint8_t *p, uint16_t o, uint32_t key, uint32_t val, uint8_t size)
{
    for (int i = 0; i < 4; i++) p[o++] = (uint8_t)(key >> (8 * i));
    for (int i = 0; i < size; i++) p[o++] = (uint8_t)(val >> (8 * i));
    return o;
}

static void gps_configure(void)
{
    uint8_t p[64];
    // 1) switch the receiver UART to 115200 — try every likely current rate
    const uint32_t bauds[] = { 38400, 9600, 115200 };
    for (uint32_t bd : bauds) {
        Serial2.end();
        Serial2.begin(bd);
        uint16_t o = 0;
        p[o++] = 0; p[o++] = 0x01; p[o++] = 0; p[o++] = 0;     // version 0, layer RAM
        o = valset_put(p, o, 0x40520001u, 115200u, 4);         // CFG-UART1-BAUDRATE
        ubx_send(0x06, 0x8A, p, o);
        delay(60);
    }
    Serial2.end();
    Serial2.setFIFOSize(1024);
    Serial2.begin(115200);
    delay(50);
    // 2) navigation configuration
    uint16_t o = 0;
    p[o++] = 0; p[o++] = 0x01; p[o++] = 0; p[o++] = 0;
    o = valset_put(p, o, 0x30210001u, GPS_RATE_MS, 2);          // CFG-RATE-MEAS
    o = valset_put(p, o, 0x20110021u, 8u, 1);                   // CFG-NAVSPG-DYNMODEL = AIR4
    o = valset_put(p, o, 0x20910007u, 1u, 1);                   // CFG-MSGOUT-UBX_NAV_PVT_UART1
    o = valset_put(p, o, 0x10740002u, 0u, 1);                   // CFG-UART1OUTPROT-NMEA off
    o = valset_put(p, o, 0x10730001u, 1u, 1);                   // CFG-UART1INPROT-UBX on
    ubx_send(0x06, 0x8A, p, o);
}

static inline int32_t rd_i32(const uint8_t *b) { return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24)); }
static inline uint32_t rd_u32(const uint8_t *b) { return (uint32_t)rd_i32(b); }

static void gps_on_pvt(const uint8_t *b, uint16_t n)
{
    if (n < 92) return;
    zn_gps_t f = {};
    f.valid    = 1;
    f.fix_type = b[20];
    f.fix_ok   = b[21] & 0x01;
    f.sats     = b[23];
    f.lon_1e7  = rd_i32(b + 24);
    f.lat_1e7  = rd_i32(b + 28);
    f.h_msl    = rd_i32(b + 36) * 0.001f;
    f.h_acc    = rd_u32(b + 40) * 0.001f;
    f.v_acc    = rd_u32(b + 44) * 0.001f;
    f.vn       = rd_i32(b + 48) * 0.001f;
    f.ve       = rd_i32(b + 52) * 0.001f;
    f.vd       = rd_i32(b + 56) * 0.001f;
    f.s_acc    = rd_u32(b + 68) * 0.001f;
    critical_section_enter_blocking(&s_gps_cs);
    s_gps.fix = f;
    s_gps.rx_ms = millis();
    s_gps.tow_ms = rd_u32(b);
    critical_section_exit(&s_gps_cs);
}

void setup1()
{
    critical_section_init(&s_gps_cs);
    while (!s_core0_ready) delay(1);
    Serial2.setTX(PIN_GPS_TX);
    Serial2.setRX(PIN_GPS_RX);
    gps_configure();
}

void loop1()
{
    static uint8_t  st = 0, cls = 0, id = 0, ca = 0, cb = 0;
    static uint16_t len = 0, idx = 0;
    static uint8_t  buf[128];
    static uint32_t last_cfg_ms = millis();

    if (s_gps_recfg) { s_gps_recfg = false; gps_configure(); last_cfg_ms = millis(); }

    while (Serial2.available()) {
        const uint8_t c = (uint8_t)Serial2.read();
        switch (st) {
        case 0: st = (c == 0xB5) ? 1 : 0; break;
        case 1: st = (c == 0x62) ? 2 : 0; break;
        case 2: cls = c; ca = c; cb = ca; st = 3; break;
        case 3: id = c; ca += c; cb += ca; st = 4; break;
        case 4: len = c; ca += c; cb += ca; st = 5; break;
        case 5: len |= (uint16_t)c << 8; ca += c; cb += ca; idx = 0; st = (len > sizeof(buf)) ? 0 : (len ? 6 : 7); break;
        case 6: buf[idx++] = c; ca += c; cb += ca; if (idx >= len) st = 7; break;
        case 7: st = (c == ca) ? 8 : 0; break;
        case 8:
            if (c == cb) {
                critical_section_enter_blocking(&s_gps_cs);
                s_gps.last_ubx_ms = millis();
                if (cls == 0x05 && id == 0x01 && len >= 2 && buf[0] == 0x06 && buf[1] == 0x8A) s_gps.cfg_ack = true;
                critical_section_exit(&s_gps_cs);
                if (cls == 0x01 && id == 0x07) gps_on_pvt(buf, len);
            }
            st = 0;
            break;
        }
    }
    // no UBX for 3 s -> configure again (receiver power-cycled, wrong baud ...)
    uint32_t last;
    critical_section_enter_blocking(&s_gps_cs);
    last = s_gps.last_ubx_ms;
    critical_section_exit(&s_gps_cs);
    if ((uint32_t)(millis() - (last ? last : last_cfg_ms)) > 3000u &&
        (uint32_t)(millis() - last_cfg_ms) > 3000u) {
        gps_configure();
        last_cfg_ms = millis();
    }
    delay(1);
}

// =====================================================================
//  Link to the Master
// =====================================================================
static void link_send(uint8_t type, const void *p, uint8_t n)
{
    uint8_t f[ZL_FRAME_MAX];
    const uint16_t len = zl_encode(f, type, p, n);
    if (len && Serial1.availableForWrite() >= (int)len) Serial1.write(f, len);
}

static void handle_cmd(const zl_pos_cmd_t &c)
{
    switch (c.cmd) {
    case ZL_PCMD_PADREF:
        if (s_nav.mode == ZN_MODE_PAD) {
            zn_padref(&s_nav);
            uint32_t b; memcpy(&b, &s_nav.p0, 4);
            watchdog_hw->scratch[1] = b;   // remember for a reset in flight
            watchdog_hw->scratch[0] = SCR_MAGIC;
        }
        break;
    case ZL_PCMD_GYRO_CAL:
        if (s_nav.mode == ZN_MODE_PAD || s_nav.mode == ZN_MODE_INIT) {
            s_nav.mode = ZN_MODE_INIT;
            s_nav.calib_n = 0;
            memset(s_nav.calib_sum, 0, sizeof s_nav.calib_sum);
            memset(s_nav.f_lp, 0, sizeof s_nav.f_lp);
        }
        break;
    case ZL_PCMD_REBOOT:
        if (c.arg == 0xB0 && !zl_state_in_flight(s_mst.state)) { watchdog_enable(1, true); while (1) {} }
        break;
    case ZL_PCMD_GPS_RECFG:
        s_gps_recfg = true;
        break;
    case ZL_PCMD_MAG_DECL:
        if (c.val > -40.0f && c.val < 40.0f) s_nav.cfg.declination_deg = c.val;
        break;
    default: break;
    }
}

static void link_poll(uint32_t now)
{
    int budget = 512;
    while (Serial1.available() && budget-- > 0) {
        const uint8_t *pl; uint8_t n;
        const uint8_t t = zl_rx_byte(&s_rx, (uint8_t)Serial1.read(), &pl, &n);
        if (t == ZL_PKT_MST_STATE) {
            zl_mst_state_t m;
            if (zl_take(&m, sizeof m, pl, n)) {
                s_mst = m;
                s_mst_rx_ms = now;
                s_nav.nose_forced = m.nose_axis <= 6 ? m.nose_axis : 0;
            }
        } else if (t == ZL_PKT_POS_CMD) {
            zl_pos_cmd_t c;
            if (zl_take(&c, sizeof c, pl, n)) handle_cmd(c);
        }
    }
}

// =====================================================================
//  LED
// =====================================================================
static void led_update(uint32_t now)
{
    static uint32_t next = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + 50;
    uint32_t c;
    const bool blink = (now % 1000u) < 500u;
    if (!s_imu_ok || !s_baro_ok)            c = blink ? 0xFF0000 : 0;              // sensor failure
    else if (s_nav.mode == ZN_MODE_INIT)    c = 0x0000FF;                          // calibrating
    else if (s_nav.mode == ZN_MODE_FLIGHT)  c = (now % 200u) < 60u ? 0xFFFFFF : 0; // flying
    else if (s_nav.mode == ZN_MODE_BARO_ONLY) c = blink ? 0xFF00FF : 0;
    else if (!s_nav.gps_fix)                c = 0xFFA000;                          // pad, no GPS
    else                                    c = 0x00FF00;                          // pad, all good
    s_led.setPixelColor(0, s_led.Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF));
    s_led.show();
}

// =====================================================================
//  setup / loop (core 0)
// =====================================================================
void setup()
{
    s_reset_reason = watchdog_caused_reboot() ? 1 : 0;
    Serial.begin(115200);

    if (PIN_LED_PWR >= 0) { pinMode(PIN_LED_PWR, OUTPUT); digitalWrite(PIN_LED_PWR, HIGH); }
    s_led.begin();
    s_led.setBrightness(40);
    s_led.setPixelColor(0, s_led.Color(0, 0, 255));
    s_led.show();

    pinMode(PIN_XIAO_SYNC, INPUT_PULLDOWN);
    pinMode(PIN_GPS_PPS, INPUT);

    Serial1.setTX(PIN_LINK_TX);
    Serial1.setRX(PIN_LINK_RX);
    Serial1.setFIFOSize(2048);            // the default 32 B loses frames at 921600
    Serial1.begin(ZL_LINK_BAUD);
    zl_rx_init(&s_rx);

    zn_init(&s_nav);

    TriSenseConfig cfg;
    cfg.imuOdr     = ODR_960HZ;
    cfg.imuAccelFs = AFS_32G;
    cfg.imuGyroFs  = GFS_4000DPS;       // a rolling rocket easily exceeds 2000 dps
    cfg.highG      = HIGHG_AUTO;
    cfg.highGRange = HGFS_320G;
    cfg.baroOdr    = BMP580_ODR_240Hz;
    cfg.magOdr     = MMC_ODR_100HZ;
    s_imu_ok = beginZenith(s_sens, cfg);
    s_hg_ok  = s_sens.hasHighG();
    s_baro_ok = s_imu_ok && s_sens.baroConfigValid();
    s_mag_ok = s_imu_ok;
    if (s_imu_ok) s_imu_dt = 1.0f / s_sens.imu.getODRHzExact();
    if (!(s_imu_dt > 0.0001f && s_imu_dt < 0.02f)) s_imu_dt = 1.0f / 960.0f;

    // Reset in flight? (watchdog reboot + Master's sync line high)
    delay(20);
    if (digitalRead(PIN_XIAO_SYNC) == HIGH) {
        float p0 = 0;
        if (watchdog_hw->scratch[0] == SCR_MAGIC) { uint32_t b = watchdog_hw->scratch[1]; memcpy(&p0, &b, 4); }
        zn_start_in_flight(&s_nav, p0, 10.0f);
    }

    s_core0_ready = true;
    watchdog_enable(WDT_MS, true);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    link_poll(now);

    // ---- IMU: drain the fused FIFO through the estimator ----------------
    if (s_imu_ok) {
        TriSenseIMUSample smp;
        int guard = 64;
        while (guard-- > 0 && s_sens.imu.readSample(smp)) {
            zn_imu(&s_nav, smp.a, smp.g, s_imu_dt, millis());
            s_imu_count++;
            memcpy(s_last_a, smp.a, sizeof s_last_a);
            memcpy(s_last_g, smp.g, sizeof s_last_g);
            float hw = smp.highGWeight[0];
            if (smp.highGWeight[1] > hw) hw = smp.highGWeight[1];
            if (smp.highGWeight[2] > hw) hw = smp.highGWeight[2];
            s_hg_weight = hw;
            const float am = sqrtf(smp.a[0]*smp.a[0] + smp.a[1]*smp.a[1] + smp.a[2]*smp.a[2]);
            if (am > s_peak_g) s_peak_g = am;
        }
        link_poll(now);
    }

    // ---- barometer + magnetometer, 100 Hz ---------------------------------
    static uint32_t next_baro = 0;
    if ((int32_t)(now - next_baro) >= 0) {
        next_baro = now + 10;
        if (s_imu_ok) {
            s_pressure = s_sens.bmp.readPressure();
            s_baro_t   = s_sens.bmp.readTemperature();
            s_baro_ok  = s_pressure > 1000.0f && s_pressure < 130000.0f;
            zn_baro(&s_nav, s_pressure, s_baro_t, now);
            if (s_sens.mag.readData()) {
                s_last_m[0] = s_sens.mag.x; s_last_m[1] = s_sens.mag.y; s_last_m[2] = s_sens.mag.z;
                zn_mag(&s_nav, s_last_m);
            }
        }
    }

    // ---- GPS fix from core 1 ------------------------------------------------
    static uint32_t last_fix_ms = 0;
    GpsShared g;
    critical_section_enter_blocking(&s_gps_cs);
    g = s_gps;
    critical_section_exit(&s_gps_cs);
    if (g.rx_ms != last_fix_ms) {
        last_fix_ms = g.rx_ms;
        zn_gps(&s_nav, &g.fix, (float)(uint32_t)(now - g.rx_ms) * 0.001f + 0.05f, now);
    }

    // ---- Master state (and the hard sync line) -----------------------------
    {
        uint8_t fl = s_mst.flags;
        if (digitalRead(PIN_XIAO_SYNC) == HIGH) fl |= ZL_MST_LAUNCHED;
        const bool fresh = (uint32_t)(now - s_mst_rx_ms) < 1000u;
        if (!fresh) fl &= (uint8_t)~ZL_MST_ON_PAD;     // never undo a liftoff on stale info
        zn_master(&s_nav, fl, s_mst.pyro_events, now);
    }

    // ---- outgoing packets ------------------------------------------------------
    static uint32_t next_nav = 0, next_stat = 0, next_raw = 0, next_hz = 0;
    if ((int32_t)(now - next_nav) >= 0) {
        next_nav = now + 10;
        zl_nav_t o;
        zn_output(&s_nav, &o, s_imu_ok, s_mag_ok);
        link_send(ZL_PKT_NAV, &o, sizeof o);
    }
    if ((int32_t)(now - next_hz) >= 0) {
        next_hz = now + 1000;
        s_imu_hz = s_imu_count;
        s_imu_count = 0;
    }
    if ((int32_t)(now - next_stat) >= 0) {
        next_stat = now + 100;
        zl_pos_status_t s = {};
        s.lat_1e7 = g.fix.lat_1e7;
        s.lon_1e7 = g.fix.lon_1e7;
        s.gps_alt_cm = (int32_t)(g.fix.h_msl * 100.0f);
        s.gps_tow_ms = g.tow_ms;
        s.pressure_pa = s_pressure;
        s.baro_temp_c = s_baro_t;
        s.pad_pressure_pa = s_nav.pad_ref ? s_nav.p0 : 0.0f;
        zn_euler(&s_nav, &s.roll, &s.pitch, &s.yaw);
        memcpy(s.gyro_bias, s_nav.gbias, sizeof s.gyro_bias);
        s.peak_g = s_peak_g;
        s.pad_lat = s_nav.have_org ? s_nav.org_lat * 1e-7f : 0.0f;
        s.pad_lon = s_nav.have_org ? s_nav.org_lon * 1e-7f : 0.0f;
        s_imu_lost = s_imu_ok ? s_sens.imu.getLostPacketCount() : 0;
        s.imu_lost = s_imu_lost;
        uint16_t sf = 0;
        if (s_imu_ok) sf |= ZL_SENS_IMU32;
        if (s_hg_ok) sf |= ZL_SENS_IMU320;
        if (s_baro_ok) sf |= ZL_SENS_BARO;
        if (s_mag_ok && s_nav.mag_ok) sf |= ZL_SENS_MAG;
        if (g.last_ubx_ms && (uint32_t)(now - g.last_ubx_ms) < 2000u) sf |= ZL_SENS_GPS;
        if (g.cfg_ack) sf |= ZL_SENS_GPS_CFG;
        s.sensors = sf;
        s.imu_hz = (uint16_t)s_imu_hz;
        s.h_acc_dm = (uint16_t)fminf(g.fix.h_acc * 10.0f, 65535.0f);
        s.v_acc_dm = (uint16_t)fminf(g.fix.v_acc * 10.0f, 65535.0f);
        s.s_acc_cms = (uint16_t)fminf(g.fix.s_acc * 100.0f, 65535.0f);
        s.sats = g.fix.sats;
        s.fix_type = g.fix.fix_type;
        s.nose_axis = s_nav.nose_axis;
        s.fw_major = ZENITH_FW_VERSION / 100;
        s.fw_minor = ZENITH_FW_VERSION % 100;
        s.reset_reason = s_reset_reason;
        link_send(ZL_PKT_POS_STATUS, &s, sizeof s);
    }
    if ((s_mst.flags & ZL_MST_RAW_STREAM) && (int32_t)(now - next_raw) >= 0) {
        next_raw = now + 40;
        zl_pos_raw_t r;
        memcpy(r.a, s_last_a, sizeof r.a);
        memcpy(r.g, s_last_g, sizeof r.g);
        memcpy(r.m, s_last_m, sizeof r.m);
        r.hg_weight = s_hg_weight;
        r.imu_temp_c = s_baro_t;
        link_send(ZL_PKT_POS_RAW, &r, sizeof r);
    }

    led_update(now);
}
