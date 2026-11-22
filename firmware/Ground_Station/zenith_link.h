// =====================================================================
//  Zenith — every wire and radio format of the system, in one header
//
//  Used by: Zenith_Master (Pico 2), Zenith_Position (XIAO RP2350),
//           Ground_Station (Pico 2 W) and Rampa_MCU (RampaCode repo).
//  Plain C, no Arduino dependency: ../../test/link_test.cpp checks it on
//  a PC. Every sketch carries a COPY of this file in its own folder (the
//  Arduino builder does not follow "../common"); `python3 tools/sync_common.py`
//  checks the copies, `--fix` refreshes them.
//
//  ---------------------------------------------------------------------
//  1. SERIAL FRAMING (inter-MCU UART 921600 Bd, umbilical UART 115200 Bd)
//  ---------------------------------------------------------------------
//      byte 0      SYNC0 0x5A
//      byte 1      SYNC1 0xE7
//      byte 2      TYPE  (ZL_PKT_*)
//      byte 3      LEN   payload length 0..ZL_PAYLOAD_MAX
//      byte 4..    payload (packed little-endian struct, memcpy = wire format)
//      last 2      CRC-16/CCITT-FALSE over TYPE, LEN and payload (LSB first)
//
//  The decoder (zl_rx_byte) resynchronises inside already-buffered bytes
//  after a CRC/length failure, so one corrupted byte costs one frame.
//
//  2. LORA (433 MHz, SF7, BW 250 kHz, CR 4/5) — raw packets, own magic+CRC
//      zl_tlm_t           rocket  -> ground   (ZL_TLM_HZ, carries the TDMA sync)
//      zl_rampa_beacon_t  rampa   -> ground   (1 Hz, only when NO rocket on pogo)
//      zl_uplink_t        ground  -> rocket/rampa (inside the RX window)
//
//  TDMA: the transmitter (rocket, or rampa when alone) opens a 300 ms RX
//  window every 10 s. Every downlink packet says how many ms after ITS OWN
//  TX START the next window opens (next_rx_in_ms), so the ground station
//  locks to the window to ~2 ms without any shared clock. During a
//  countdown an extra final-abort window always opens at T-1.15 .. T-0.85 s.
// =====================================================================
#ifndef ZENITH_LINK_H
#define ZENITH_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
#define ZL_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define ZL_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

#define ZENITH_FW_VERSION      100   // 1.00

// ------------------------------------------------------------------ framing
#define ZL_SYNC0               0x5Au
#define ZL_SYNC1               0xE7u
#define ZL_HDR_LEN             4u
#define ZL_CRC_LEN             2u
#define ZL_PAYLOAD_MAX         120u
#define ZL_FRAME_MAX           (ZL_HDR_LEN + ZL_PAYLOAD_MAX + ZL_CRC_LEN)

#define ZL_LINK_BAUD           921600UL   // Master <-> Position (PCB traces)
#define ZL_UMB_BAUD            115200UL   // Rocket <-> Rampa over the pogo pins

typedef enum {
    ZL_PKT_NAV         = 0x10,  // Position -> Master, 100 Hz
    ZL_PKT_POS_STATUS  = 0x11,  // Position -> Master, 10 Hz
    ZL_PKT_POS_RAW     = 0x12,  // Position -> Master, 25 Hz while diag raw stream on
    ZL_PKT_MST_STATE   = 0x20,  // Master -> Position, 20 Hz + on change
    ZL_PKT_POS_CMD     = 0x21,  // Master -> Position, on event
    ZL_PKT_RMP_CMD     = 0x30,  // Rocket -> Rampa, 20 Hz
    ZL_PKT_RMP_STATUS  = 0x31   // Rampa  -> Rocket, reply to every RMP_CMD
} zl_pkt_type_t;

// =====================================================================
//  Master mission states (shared by telemetry, GUI, Position)
// =====================================================================
typedef enum {
    ZS_BOOT      = 0,   // power-on self test
    ZS_IDLE      = 1,   // on the pad, safe, pyros disarmed
    ZS_READY     = 2,   // "launch soon": pad reference taken, rampa READY
    ZS_COUNTDOWN = 3,   // synchronised countdown with the rampa
    ZS_BOOST     = 4,   // liftoff detected, motor burning
    ZS_COAST     = 5,   // burnout, waiting for apogee
    ZS_DROGUE    = 6,   // drogue fired (PYRO1), descending
    ZS_MAIN      = 7,   // main fired (PYRO2)
    ZS_LANDED    = 8,   // on the ground, recovery beacon
    ZS_ABORT     = 9,   // countdown aborted, pyros safe
    ZS_DIAG      = 10,  // USB diagnostic test mode (ground only)
    ZS_COUNT
} zl_state_t;

static inline int zl_state_in_flight(uint8_t s)
{
    return s == ZS_BOOST || s == ZS_COAST || s == ZS_DROGUE || s == ZS_MAIN;
}

// No-go / error reasons (first active reason is reported in telemetry)
typedef enum {
    ZE_NONE = 0,
    ZE_POS_LINK,        //  1 no NAV packets from the Position MCU
    ZE_NAV_INVALID,     //  2 navigation solution not valid
    ZE_RAMPA_OFFLINE,   //  3 no rampa status on the umbilical
    ZE_RAMPA_NOT_READY, //  4 rampa reports a fault / key off / no igniter
    ZE_LOW_BATT,        //  5 rocket battery below minimum
    ZE_NO_CONT_DROGUE,  //  6 PYRO1 (drogue) has no continuity
    ZE_NO_CONT_MAIN,    //  7 PYRO2 (main) has no continuity
    ZE_NO_CONT_BACKUP,  //  8 PYRO3 (backup) has no continuity (only if required)
    ZE_RBF_IN,          //  9 RBF switch in SAFE: pyro rail cannot be armed
    ZE_FLASH,           // 10 log flash missing
    ZE_HW,              // 11 INA226 / ADS1115 / PCA9685 not answering
    ZE_BATT_SWITCH,     // 12 FORCE_BAT did not move the load onto the battery
    ZE_ARM_FAIL,        // 13 pyro rail did not come up after arming
    ZE_COMMIT_FAIL,     // 14 rampa did not confirm COMMIT
    ZE_HANGFIRE,        // 15 no liftoff after T-0
    ZE_ABORT_GS,        // 16 aborted by the ground station / USB
    ZE_ABORT_RAMPA,     // 17 rampa aborted (button, timeout, fault)
    ZE_NOT_VERTICAL,    // 18 rocket tilted more than the limit on the pad
    ZE_COUNT
} zl_err_t;

// =====================================================================
//  Position -> Master : navigation solution (100 Hz)
// =====================================================================
#define ZL_NAV_VALID        (1u << 0)   // solution usable for decisions
#define ZL_NAV_BARO_OK      (1u << 1)   // barometer answering
#define ZL_NAV_BARO_TRUST   (1u << 2)   // barometer currently fused (subsonic, no burn, no ejection)
#define ZL_NAV_GPS_FUSED    (1u << 3)   // a GPS fix was fused in the last 1.5 s
#define ZL_NAV_ATT_OK       (1u << 4)   // attitude initialised from gravity (+mag)
#define ZL_NAV_LIFTOFF      (1u << 5)   // Position's own liftoff detector
#define ZL_NAV_BURNOUT      (1u << 6)   // axial specific force went negative
#define ZL_NAV_APO_KF       (1u << 7)   // KF vertical speed crossed zero (coast)
#define ZL_NAV_APO_BARO     (1u << 8)   // barometric altitude descending (median)
#define ZL_NAV_TRANSONIC    (1u << 9)   // estimated Mach in the unreliable band
#define ZL_NAV_HIGHG        (1u << 10)  // > 4 g: GPS ignored
#define ZL_NAV_PAD_REF      (1u << 11)  // pad pressure/position reference captured
#define ZL_NAV_IMU_OK       (1u << 12)
#define ZL_NAV_MAG_OK       (1u << 13)
#define ZL_NAV_GPS_FIX      (1u << 14)  // receiver has a 3-D fix
#define ZL_NAV_STILL        (1u << 15)  // vehicle at rest (ZUPT active)

typedef enum {
    ZN_MODE_INIT     = 0,  // sensors starting / calibrating
    ZN_MODE_PAD      = 1,  // levelling, gyro-bias learning, ZUPT
    ZN_MODE_FLIGHT   = 2,  // gyro-only attitude, full 3-axis KF
    ZN_MODE_BARO_ONLY= 3   // rebooted in flight: no attitude, vertical from baro
} zl_nav_mode_t;

#pragma pack(push, 1)
typedef struct {
    uint32_t t_ms;          //  0 Position millis()
    float    alt;           //  4 m above pad (KF)
    float    vz;            //  8 m/s, up +
    float    px, py;        // 12 m east / north of the pad
    float    vx, vy;        // 20 m/s east / north
    float    az_w;          // 28 m/s^2 world-vertical acceleration, gravity removed
    float    acc_axial_g;   // 32 g, specific force along the nose axis (+ = toward nose)
    float    acc_mag_g;     // 36 g, |specific force|
    float    mach;          // 40 speed / local speed of sound
    float    baro_alt;      // 44 m above pad, raw barometric
    float    sig_alt;       // 48 m, 1-sigma of alt
    float    tilt_deg;      // 52 nose angle from vertical
    float    qw, qx, qy, qz;// 56 body -> pad ENU quaternion
    float    max_alt;       // 72 highest KF altitude since liftoff
    uint16_t flags;         // 76 ZL_NAV_*
    uint8_t  mode;          // 78 zl_nav_mode_t
    uint8_t  seq;           // 79
} zl_nav_t;

// =====================================================================
//  Position -> Master : status (10 Hz)
// =====================================================================
#define ZL_SENS_IMU32   (1u << 0)
#define ZL_SENS_IMU320  (1u << 1)
#define ZL_SENS_BARO    (1u << 2)
#define ZL_SENS_MAG     (1u << 3)
#define ZL_SENS_GPS     (1u << 4)   // UBX traffic seen in the last 2 s
#define ZL_SENS_GPS_CFG (1u << 5)   // receiver acknowledged our configuration

typedef struct {
    int32_t  lat_1e7;       //  0 last GPS fix
    int32_t  lon_1e7;       //  4
    int32_t  gps_alt_cm;    //  8 hMSL
    uint32_t gps_tow_ms;    // 12 GPS time of week
    float    pressure_pa;   // 16
    float    baro_temp_c;   // 20
    float    pad_pressure_pa;//24 pad reference (0 = none)
    float    roll, pitch, yaw; // 28 deg (yaw = heading of body X from north)
    float    gyro_bias[3];  // 40 dps
    float    peak_g;        // 52
    float    pad_lat, pad_lon; // 56 deg (float is fine for display)
    uint32_t imu_lost;      // 64 lost IMU samples
    uint16_t sensors;       // 68 ZL_SENS_*
    uint16_t imu_hz;        // 70 measured fused-IMU rate
    uint16_t h_acc_dm;      // 72 GPS horizontal accuracy, dm
    uint16_t v_acc_dm;      // 74
    uint16_t s_acc_cms;     // 76 speed accuracy, cm/s
    uint8_t  sats;          // 78
    uint8_t  fix_type;      // 79 UBX fixType
    uint8_t  nose_axis;     // 80 1..6 = +X,-X,+Y,-Y,+Z,-Z
    uint8_t  fw_major;      // 81
    uint8_t  fw_minor;      // 82
    uint8_t  reset_reason;  // 83 0 power-on, 1 watchdog
} zl_pos_status_t;

typedef struct {
    float a[3];             // g, board axes (fused IMU)
    float g[3];             // dps
    float m[3];             // uT
    float hg_weight;        // max share of the high-g channel
    float imu_temp_c;
} zl_pos_raw_t;

// =====================================================================
//  Master -> Position
// =====================================================================
#define ZL_MST_LAUNCHED    (1u << 0)   // liftoff confirmed by the Master
#define ZL_MST_COUNTDOWN   (1u << 1)
#define ZL_MST_RAW_STREAM  (1u << 2)   // send ZL_PKT_POS_RAW (diagnostics)
#define ZL_MST_ON_PAD      (1u << 3)   // IDLE/READY/COUNTDOWN/ABORT/DIAG

typedef struct {
    uint32_t t_flight_ms;   // ms since liftoff (0 before)
    uint8_t  state;         // zl_state_t
    uint8_t  flags;         // ZL_MST_*
    uint8_t  pyro_events;   // incremented on every pyro firing (baro ignores ejection)
    uint8_t  nose_axis;     // 0 = auto-detect on the pad, 1..6 forced
} zl_mst_state_t;

typedef enum {
    ZL_PCMD_PADREF    = 1,  // capture pad pressure, GPS origin, re-level
    ZL_PCMD_GYRO_CAL  = 2,  // re-learn gyro bias now (must be still)
    ZL_PCMD_REBOOT    = 3,  // arg must be 0xB0
    ZL_PCMD_GPS_RECFG = 4,  // re-send receiver configuration
    ZL_PCMD_MAG_DECL  = 5   // val = magnetic declination, deg
} zl_pcmd_t;

typedef struct {
    uint8_t  cmd;           // zl_pcmd_t
    uint8_t  arg;
    uint16_t _pad;
    float    val;
} zl_pos_cmd_t;

// =====================================================================
//  Rocket <-> Rampa umbilical (pogo J1 <-> rampa J3, UART 115200)
//
//      Zenith J1.1 GND  ----- J3.1 GND
//      Zenith J1.2 5V   <---- J3.2 UMB_5V  (rampa powers the rocket on the pad)
//      Zenith J1.3 TX   ----> J3.3 -> R22 -> rampa GP1 (UART0 RX)
//      Zenith J1.4 RX   <---- J3.4 <- R21 <- rampa GP0 (UART0 TX)
//
//  The rocket is the master: it sends ZL_PKT_RMP_CMD at 20 Hz, the rampa
//  answers every valid command with ZL_PKT_RMP_STATUS. While commands keep
//  arriving (< ZL_RMP_TIMEOUT_MS) the rampa keeps ITS radio powered off and
//  everything goes through the rocket's LoRa.
// =====================================================================
#define ZL_RMP_POLL_MS        50u
#define ZL_RMP_TIMEOUT_MS     1500u
#define ZL_RMP_COMMIT_MS      300u      // rocket sends COMMIT this long before T-0
#define ZL_CD_MIN_MS          10000u    // shortest countdown accepted
#define ZL_CD_MAX_MS          3600000u  // longest (1 h)

typedef enum {
    ZR_CMD_PING      = 0,   // heartbeat; clears a finished FIRED/ABORT back to IDLE
    ZR_CMD_PREPARE   = 1,   // launch soon: check key, igniter, battery
    ZR_CMD_COUNTDOWN = 2,   // t_rem_ms to T-0 (re-synchronised on every frame)
    ZR_CMD_COMMIT    = 3,   // final lock-in, T-0 within ZL_RMP_COMMIT_MS
    ZR_CMD_ABORT     = 4,   // stop everything, pyros off
    ZR_CMD_SAFE      = 5    // back to IDLE (pyros off), clears ABORT latch
} zl_rmp_cmd_id_t;

typedef enum {
    ZR_ST_OFFLINE   = 0,    // (rocket-side: no answer)
    ZR_ST_IDLE      = 1,
    ZR_ST_READY     = 2,
    ZR_ST_COUNTDOWN = 3,
    ZR_ST_COMMITTED = 4,
    ZR_ST_FIRED     = 5,
    ZR_ST_ABORT     = 6,
    ZR_ST_FAULT     = 7
} zl_rmp_state_t;

// rocket_flags
#define ZL_RK_GO            (1u << 0)   // rocket Go/No-Go passes right now
#define ZL_RK_ARMED         (1u << 1)   // rocket pyro rail armed
#define ZL_RK_ON_BATT       (1u << 2)   // rocket switched to its own battery
#define ZL_RK_NAV_OK        (1u << 3)

// rampa status flags
#define ZL_RMP_KEY          (1u << 0)   // physical key closed
#define ZL_RMP_CONT1        (1u << 1)
#define ZL_RMP_CONT2        (1u << 2)
#define ZL_RMP_CONT3        (1u << 3)
#define ZL_RMP_PYRO_ARMED   (1u << 4)
#define ZL_RMP_LORA_OFF     (1u << 5)
#define ZL_RMP_ROCKET_PWR   (1u << 6)   // UMB_5V switched on
#define ZL_RMP_READY        (1u << 7)   // every rampa interlock passes

// rampa fault bits
#define ZL_RMPF_LOW_VBAT    (1u << 0)
#define ZL_RMPF_NO_KEY      (1u << 1)
#define ZL_RMPF_NO_CONT     (1u << 2)
#define ZL_RMPF_TIMEOUT     (1u << 3)
#define ZL_RMPF_ROCKET      (1u << 4)   // rocket dropped its GO flag
#define ZL_RMPF_HW          (1u << 5)
#define ZL_RMPF_BTN_ABORT   (1u << 6)
#define ZL_RMPF_CLOCK       (1u << 7)   // countdown/commit timing out of range

typedef struct {
    int32_t  t_rem_ms;      //  0 ms to T-0 measured at frame TX start (-1 none)
    uint16_t fire_ms;       //  4 igniter pulse length
    uint8_t  cmd;           //  6 zl_rmp_cmd_id_t
    uint8_t  seq;           //  7
    uint8_t  rocket_flags;  //  8 ZL_RK_*
    uint8_t  pyro_ch;       //  9 rampa igniter channel 1..3
    uint8_t  rocket_state;  // 10 zl_state_t (for the rampa display/log)
    uint8_t  _pad;          // 11
} zl_rmp_cmd_t;

typedef struct {
    int32_t  t_rem_ms;      //  0 rampa's own countdown (-1 none)
    uint16_t vbat_mv;       //  4 rampa battery
    uint16_t umb_ma;        //  6 current into the rocket on UMB_5V
    uint16_t cont_mv[3];    //  8 rampa igniter continuity
    uint8_t  state;         // 14 zl_rmp_state_t
    uint8_t  flags;         // 15 ZL_RMP_*
    uint8_t  faults;        // 16 ZL_RMPF_*
    uint8_t  last_seq;      // 17
    uint8_t  fw;            // 18
    uint8_t  _pad;          // 19
} zl_rmp_status_t;
#pragma pack(pop)

ZL_STATIC_ASSERT(sizeof(zl_nav_t) == 80, "nav layout");
ZL_STATIC_ASSERT(sizeof(zl_pos_status_t) == 84, "pos status layout");
ZL_STATIC_ASSERT(sizeof(zl_pos_raw_t) == 44, "pos raw layout");
ZL_STATIC_ASSERT(sizeof(zl_mst_state_t) == 8, "mst state layout");
ZL_STATIC_ASSERT(sizeof(zl_pos_cmd_t) == 8, "pos cmd layout");
ZL_STATIC_ASSERT(sizeof(zl_rmp_cmd_t) == 12, "rmp cmd layout");
ZL_STATIC_ASSERT(sizeof(zl_rmp_status_t) == 20, "rmp status layout");

// =====================================================================
//  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
// =====================================================================
static inline uint16_t zl_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) {
        crc ^= (uint16_t)((uint16_t)(*p++) << 8);
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/** Build a complete frame into out[ZL_FRAME_MAX]. Returns its length or 0. */
static inline uint16_t zl_encode(uint8_t *out, uint8_t type, const void *payload, uint8_t len)
{
    if (len > ZL_PAYLOAD_MAX) return 0;
    out[0] = ZL_SYNC0;
    out[1] = ZL_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len && payload) memcpy(&out[4], payload, len);
    const uint16_t crc = zl_crc16(&out[2], (size_t)len + 2u);
    out[4 + len] = (uint8_t)(crc & 0xFFu);
    out[5 + len] = (uint8_t)(crc >> 8);
    return (uint16_t)(ZL_HDR_LEN + len + ZL_CRC_LEN);
}

// ---------------------------------------------------------------- decoder
typedef struct {
    uint8_t  buf[ZL_FRAME_MAX];
    uint16_t n;
    uint16_t consume;
    uint32_t rx_ok, rx_crc, rx_len;
} zl_rx_t;

static inline void zl_rx_init(zl_rx_t *rx) { memset(rx, 0, sizeof(*rx)); }

/** Drop buf[0..from-1] and re-hunt for a sync pair in what is left. */
static inline void zl_rx_resync(zl_rx_t *rx, uint16_t from)
{
    for (uint16_t k = from; k + 1u < rx->n; k++) {
        if (rx->buf[k] == ZL_SYNC0 && rx->buf[k + 1u] == ZL_SYNC1) {
            memmove(rx->buf, &rx->buf[k], (size_t)(rx->n - k));
            rx->n = (uint16_t)(rx->n - k);
            return;
        }
    }
    if (rx->n > from && rx->buf[rx->n - 1u] == ZL_SYNC0) {
        rx->buf[0] = ZL_SYNC0;
        rx->n = 1;
    } else {
        rx->n = 0;
    }
}

/**
 * Feed one byte. Returns the packet type (> 0) when a frame passes its CRC;
 * *payload / *len then point into rx->buf until the next call.
 */
static inline uint8_t zl_rx_byte(zl_rx_t *rx, uint8_t b, const uint8_t **payload, uint8_t *len)
{
    if (rx->consume) {
        if (rx->n > rx->consume) {
            memmove(rx->buf, &rx->buf[rx->consume], (size_t)(rx->n - rx->consume));
            rx->n = (uint16_t)(rx->n - rx->consume);
        } else {
            rx->n = 0;
        }
        rx->consume = 0;
    }
    if (rx->n == 0) {
        if (b == ZL_SYNC0) { rx->buf[0] = b; rx->n = 1; }
        return 0;
    }
    if (rx->n == 1) {
        if (b == ZL_SYNC1)      { rx->buf[1] = b; rx->n = 2; }
        else if (b == ZL_SYNC0) { rx->n = 1; }
        else                    { rx->n = 0; }
        return 0;
    }
    if (rx->n >= ZL_FRAME_MAX) rx->n = 0;   // never overrun (unreachable)
    rx->buf[rx->n++] = b;

    for (;;) {
        if (rx->n < ZL_HDR_LEN) return 0;
        const uint8_t l = rx->buf[3];
        if (l > ZL_PAYLOAD_MAX) {
            rx->rx_len++;
            zl_rx_resync(rx, 1);
            continue;
        }
        const uint16_t need = (uint16_t)(ZL_HDR_LEN + l + ZL_CRC_LEN);
        if (rx->n < need) return 0;
        const uint16_t crc = (uint16_t)(rx->buf[4 + l] | ((uint16_t)rx->buf[5 + l] << 8));
        if (zl_crc16(&rx->buf[2], (size_t)l + 2u) != crc) {
            rx->rx_crc++;
            zl_rx_resync(rx, 1);
            continue;
        }
        rx->rx_ok++;
        *payload = &rx->buf[4];
        *len = l;
        rx->consume = need;
        return rx->buf[2] ? rx->buf[2] : 0xFFu;
    }
}

/** Copy a received payload into a struct only if the size matches exactly. */
static inline int zl_take(void *dst, size_t want, const uint8_t *payload, uint8_t len)
{
    if ((size_t)len != want) return 0;
    memcpy(dst, payload, want);
    return 1;
}

// =====================================================================
//  LoRa radio
// =====================================================================
#define ZL_LORA_FREQ_HZ        433000000UL
#define ZL_LORA_SF             7
#define ZL_LORA_BW_HZ          250000UL
#define ZL_LORA_CR4            5
#define ZL_LORA_PREAMBLE       8
#define ZL_LORA_TX_DBM         20
#define ZL_LORA_SYNCWORD       0x2B     // private network word (Daidalos uses default 0x12)

#define ZL_TLM_HZ_DEFAULT      5

/** SX127x time on air (explicit header, CRC on, LDRO off), microseconds. */
static inline uint32_t zl_lora_airtime_us(uint16_t pl)
{
    const uint32_t sf = ZL_LORA_SF;
    const uint32_t cr = ZL_LORA_CR4 - 4u;
    const uint32_t t_sym_ns = (uint32_t)(((uint64_t)(1u << sf) * 1000000000ULL) / ZL_LORA_BW_HZ);
    int32_t num = 8 * (int32_t)pl - 4 * (int32_t)sf + 28 + 16;
    int32_t den = 4 * (int32_t)sf;
    int32_t blocks = (num + den - 1) / den;
    if (blocks < 0) blocks = 0;
    const uint32_t n_payload = 8u + (uint32_t)blocks * (cr + 4u);
    const uint32_t sym_x4 = (ZL_LORA_PREAMBLE * 4u + 17u) + n_payload * 4u;
    return (uint32_t)(((uint64_t)sym_x4 * t_sym_ns) / 4000ULL);
}

// ------------------------------------------------------------------ TDMA
#define ZL_WIN_PERIOD_MS       10000u
#define ZL_WIN_DUR_MS          300u
#define ZL_WIN_TX_GUARD_MS     70u      // no downlink TX this close before a window
#define ZL_UL_OFFSET1_MS       60u      // ground TX offsets inside the window
#define ZL_UL_OFFSET2_MS       170u     // redundant copy (ABORT / COUNTDOWN)
#define ZL_FINAL_WIN_OPEN_MS   1150u    // final abort window T-1150 .. T-850 ms
#define ZL_NEXT_RX_NONE        0xFFFFu

/**
 * ms from `now` to the next RX window (periodic grid on the transmitter's
 * clock, plus the final-abort window when a countdown is running).
 * `grid_ms` = now modulo period is computed by the caller from its own epoch.
 */
static inline uint32_t zl_next_window_ms(uint32_t now_ms, int32_t t_rem_ms, int *is_final)
{
    const uint32_t rem = now_ms % ZL_WIN_PERIOD_MS;
    uint32_t next = (rem == 0u) ? 0u : (ZL_WIN_PERIOD_MS - rem);
    if (t_rem_ms > 0) {
        // keep regular windows away from the last 1.5 s before T-0
        const int32_t at = t_rem_ms - (int32_t)next;
        if (at < 1600 && at > -2000) next += ZL_WIN_PERIOD_MS;
    }
    int fin = 0;
    if (t_rem_ms > (int32_t)ZL_FINAL_WIN_OPEN_MS) {
        const uint32_t to_final = (uint32_t)(t_rem_ms - (int32_t)ZL_FINAL_WIN_OPEN_MS);
        if (to_final < next) { next = to_final; fin = 1; }
    }
    if (is_final) *is_final = fin;
    return next;
}

// ------------------------------------------------------------ uplink
#define ZL_UL_MAGIC        0xC9u
#define ZL_TLM_MAGIC       0xC5u
#define ZL_BEACON_MAGIC    0xC6u
#define ZL_LORA_VER        1u

typedef enum {
    ZL_UL_NOP        = 0,   // ping
    ZL_UL_PREPARE    = 1,   // launch soon (token)
    ZL_UL_COUNTDOWN  = 2,   // arg = ms to T-0 at TX start (token)
    ZL_UL_ABORT      = 3,   // token
    ZL_UL_SAFE       = 4,   // back to IDLE / disarm
    ZL_UL_PADREF     = 5,   // new pad reference
    ZL_UL_MAIN_ALT   = 6,   // arg = main deployment altitude, m (ground only)
    ZL_UL_LOCATE     = 7,   // arg 1/0 = recovery buzzer on/off
    ZL_UL_CAMERA     = 8,   // arg 1/0 = camera record on/off
    ZL_UL_REBOOT     = 9,   // token, ground only
    ZL_UL_ROCKET_PWR = 10,  // rampa: arg 1/0 = umbilical 5 V on/off
    ZL_UL_COUNT
} zl_ul_cmd_t;

#define ZL_UL_DEST_ROCKET  0u
#define ZL_UL_DEST_RAMPA   1u

#define ZL_TOK_NONE        0x0000u
#define ZL_TOK_PREPARE     0x5052u
#define ZL_TOK_COUNTDOWN   0xD41Du
#define ZL_TOK_ABORT       0xAB07u
#define ZL_TOK_REBOOT      0x5E5Eu

static inline uint16_t zl_ul_token_for(uint8_t cmd)
{
    switch (cmd) {
    case ZL_UL_PREPARE:   return ZL_TOK_PREPARE;
    case ZL_UL_COUNTDOWN: return ZL_TOK_COUNTDOWN;
    case ZL_UL_ABORT:     return ZL_TOK_ABORT;
    case ZL_UL_REBOOT:    return ZL_TOK_REBOOT;
    default:              return ZL_TOK_NONE;
    }
}

typedef enum {
    ZL_RC_NONE         = 0,
    ZL_RC_OK           = 1,
    ZL_RC_REJ_FLIGHT   = 2,   // refused: in flight
    ZL_RC_REJ_STATE    = 3,   // refused: wrong state
    ZL_RC_REJ_RAMPA    = 4,   // refused: rampa not ready / offline
    ZL_RC_REJ_TOKEN    = 5,   // refused: bad token / argument
    ZL_RC_ABORTED      = 6,   // abort executed
    ZL_RC_REJ_NOGO     = 7    // refused: rocket no-go (see err in telemetry)
} zl_ul_rc_t;

#define ZL_ULF_FINAL_WIN   (1u << 0)
#define ZL_ULF_RETRY       (1u << 1)

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;     //  0 ZL_UL_MAGIC
    uint8_t  ver;       //  1
    uint8_t  seq;       //  2 1..255
    uint8_t  cmd;       //  3 zl_ul_cmd_t
    int32_t  arg;       //  4
    uint16_t token;     //  8
    uint8_t  dest;      // 10 ZL_UL_DEST_*
    uint8_t  flags;     // 11 ZL_ULF_*
    uint16_t _pad;      // 12
    uint16_t crc;       // 14 over bytes 0..13
} zl_uplink_t;

// ----------------------------------------------------------- telemetry
// tlm.flags
#define ZL_TF_ARMED        (1u << 0)   // pyro rail switched on
#define ZL_TF_RBF_ARMABLE  (1u << 1)   // RBF out: arming path closed
#define ZL_TF_BW1          (1u << 2)   // breakwire 1 intact
#define ZL_TF_BW2          (1u << 3)   // breakwire 2 intact
#define ZL_TF_UMB          (1u << 4)   // pogo 5 V present
#define ZL_TF_ON_BATT      (1u << 5)   // FORCE_BAT on
#define ZL_TF_LOGGING      (1u << 6)
#define ZL_TF_GO           (1u << 7)   // Go/No-Go passes
// tlm.health
#define ZL_HL_POS_LINK     (1u << 0)
#define ZL_HL_NAV_VALID    (1u << 1)
#define ZL_HL_BARO_TRUST   (1u << 2)
#define ZL_HL_GPS_FIX      (1u << 3)
#define ZL_HL_FLASH        (1u << 4)
#define ZL_HL_HW           (1u << 5)   // INA226 + ADS1115 + PCA9685 answer
#define ZL_HL_CAM          (1u << 6)   // camera powered
#define ZL_HL_LOCATE       (1u << 7)   // recovery buzzer on

typedef struct {
    uint8_t  magic;          //  0 ZL_TLM_MAGIC
    uint8_t  ver;            //  1
    uint8_t  state;          //  2 zl_state_t
    uint8_t  flags;          //  3 ZL_TF_*
    uint32_t t_ms;           //  4 rocket millis at TX start
    int32_t  lat_1e7;        //  8
    int32_t  lon_1e7;        // 12
    int32_t  alt_cm;         // 16 KF altitude above pad
    int32_t  t_rem_ms;       // 20 countdown (-1 none) / since liftoff when flying (>=0, see state)
    int16_t  vz_dms;         // 24 vertical speed, dm/s
    int16_t  vh_dms;         // 26 horizontal speed, dm/s
    int16_t  acc_cg;         // 28 axial specific force, centi-g
    uint16_t max_alt_m;      // 30
    uint16_t vbat_mv;        // 32
    uint16_t next_rx_in_ms;  // 34 from THIS packet's TX start to the next RX window
    uint16_t main_alt_m;     // 36 configured main altitude
    uint16_t tilt_dd;        // 38 tilt from vertical, 0.1 deg
    uint8_t  mach_x100;      // 40
    uint8_t  sats;           // 41
    uint8_t  pyro;           // 42 bit0-2 continuity CH1..3, bit3-5 fired CH1..3, bit6 rail live
    uint8_t  health;         // 43 ZL_HL_*
    uint8_t  rampa_state;    // 44 zl_rmp_state_t
    uint8_t  rampa_flags;    // 45 ZL_RMP_*
    uint8_t  last_ul_seq;    // 46
    uint8_t  last_ul_rc;     // 47 zl_ul_rc_t
    uint8_t  err;            // 48 zl_err_t, first active no-go reason
    uint8_t  win_final;      // 49 1 = next window is the T-1 s final abort window
    uint16_t crc;            // 50 over bytes 0..49
} zl_tlm_t;

typedef struct {
    uint8_t  magic;          //  0 ZL_BEACON_MAGIC
    uint8_t  ver;            //  1
    uint8_t  state;          //  2 zl_rmp_state_t
    uint8_t  flags;          //  3 ZL_RMP_*
    uint32_t t_ms;           //  4
    uint16_t vbat_mv;        //  8
    uint16_t umb_ma;         // 10
    uint16_t cont_mv[3];     // 12
    uint16_t next_rx_in_ms;  // 18
    uint8_t  faults;         // 20
    uint8_t  last_ul_seq;    // 21
    uint8_t  last_ul_rc;     // 22
    uint8_t  fw;             // 23
    uint16_t crc;            // 24 over bytes 0..23
} zl_rampa_beacon_t;
#pragma pack(pop)

ZL_STATIC_ASSERT(sizeof(zl_uplink_t) == 16, "uplink layout");
ZL_STATIC_ASSERT(sizeof(zl_tlm_t) == 52, "tlm layout");
ZL_STATIC_ASSERT(sizeof(zl_rampa_beacon_t) == 26, "beacon layout");

/** Seal any LoRa packet whose LAST two bytes are its CRC (over all bytes before). */
static inline void zl_lora_seal(void *pkt, size_t size, uint8_t magic)
{
    uint8_t *p = (uint8_t *)pkt;
    p[0] = magic;
    p[1] = ZL_LORA_VER;
    const uint16_t c = zl_crc16(p, size - 2u);
    p[size - 2u] = (uint8_t)(c & 0xFFu);
    p[size - 1u] = (uint8_t)(c >> 8);
}

/** Validate a received LoRa packet of exactly `size` bytes. */
static inline int zl_lora_check(const uint8_t *buf, size_t n, size_t size, uint8_t magic)
{
    if (n != size || buf[0] != magic || buf[1] != ZL_LORA_VER) return 0;
    const uint16_t c = (uint16_t)(buf[size - 2u] | ((uint16_t)buf[size - 1u] << 8));
    return zl_crc16(buf, size - 2u) == c;
}

#endif // ZENITH_LINK_H
