// =====================================================================
//  Zenith — parameter table (Master MCU, saved in the W25Q128 flash)
//
//  Every tunable number of the mission logic lives here, with limits and a
//  description. The diagnostic GUI reads the table at runtime ("PARAM LIST")
//  so this file is the only place a parameter is defined.
// =====================================================================
#ifndef ZENITH_PARAMS_H
#define ZENITH_PARAMS_H

#include <stdint.h>

typedef struct {
    const char *name;
    float def, min, max;
    const char *desc;
} zp_def_t;

enum {
    ZP_MAIN_ALT = 0,    ZP_DROGUE_FAIL_VZ, ZP_MAIN_FAIL_VZ, ZP_DROGUE_CHECK_S, ZP_MAIN_CHECK_S,
    ZP_APO_LOCKOUT_S,   ZP_APO_MACH_MAX,   ZP_APO_BACKUP_S, ZP_APO_MAX_S,      ZP_MAIN_BACKUP_S,
    ZP_P3_ENABLE,       ZP_P3_BLIND_S,     ZP_PYRO_PULSE_MS, ZP_LIFTOFF_G,     ZP_BURN_MAX_S,
    ZP_CD_FORCE_BAT_S,  ZP_CD_ARM_S,       ZP_IGN_WAIT_S,   ZP_RAMPA_CH,       ZP_RAMPA_FIRE_MS,
    ZP_VBAT_MIN_V,      ZP_REQ_CONT3,      ZP_MAX_TILT_DEG, ZP_NOSE_AXIS,      ZP_TLM_HZ,
    ZP_LOCATE_AFTER_S,  ZP_CAM_MODE,       ZP_EXT_IGN,      ZP_BW_MODE,        ZP_DECL_DEG,
    ZP_LANDED_STILL_S,  ZP_CONT_MIN_MV,    ZP_MAIN_MIN_DELAY_S, ZP_READY_TIMEOUT_MIN,
    ZP_COUNT
};

static const zp_def_t ZP_TABLE[ZP_COUNT] = {
    { "main_alt_m",       300,   100,  1000, "Vyska vypusteni hlavniho padaku (PYRO2) nad rampou [m]" },
    { "drogue_fail_vz",   35,    10,   150,  "Selhani drogue: klesani rychlejsi nez tato [m/s] po drogue_check_s -> PYRO3" },
    { "main_fail_vz",     15,    4,    60,   "Selhani hlavniho: klesani rychlejsi nez tato [m/s] po main_check_s -> PYRO3" },
    { "drogue_check_s",   4,     1,    20,   "Za kolik s po PYRO1 se zacne kontrolovat rychlost klesani [s]" },
    { "main_check_s",     4,     1,    20,   "Za kolik s po PYRO2 se zacne kontrolovat rychlost klesani [s]" },
    { "apo_lockout_s",    6,     1,    60,   "Nejdrive po startu, kdy smi byt detekovan apogeum [s]" },
    { "apo_mach_max",     0.5f,  0.2f, 0.9f, "Apogeum se neuzna nad timto Machem (nav. odhad)" },
    { "apo_backup_s",     25,    3,    180,  "Drogue casovacem, kdyz navigace neplatna, T+ [s]" },
    { "apo_max_s",        35,    3,    300,  "Absolutni pojistka: drogue nejpozdeji v T+ [s]" },
    { "main_backup_s",    100,   3,    900,  "Main casovacem po drogue, kdyz navigace neplatna [s]" },
    { "p3_enable",        1,     0,    1,    "PYRO3 (zaloha drogue+main) povolen" },
    { "p3_blind_s",       8,     0,    900,  "Bez navigace: PYRO3 tolik s po PYRO2 (0 = nikdy)" },
    { "pyro_pulse_ms",    300,   20,   2000, "Delka pulzu pyro kanalu [ms] (pozor na 2W odpory 2x4.7R)" },
    { "liftoff_g",        2.5f,  1.5f, 10,   "Detekce startu: osove zrychleni nad [g] po 60 ms" },
    { "burn_max_s",       6,     0.5f, 30,   "Nejdelsi mozne horeni motoru [s] (pak COAST)" },
    { "cd_force_bat_s",   10,    3,    120,  "V T- prepnout napajeni na baterii (FORCE_BAT) [s]" },
    { "cd_arm_s",         5,     1,    60,   "V T- zapnout pyro listu (PYRO_SW_ARM) [s]" },
    { "ign_wait_s",       5,     1,    30,   "Cekani na start po T-0, pak HANGFIRE [s]" },
    { "rampa_ch",         1,     1,    3,    "Kanal rozbusky motoru na rampe (1..3)" },
    { "rampa_fire_ms",    1000,  100,  5000, "Delka zazehovaciho pulzu rampy [ms]" },
    { "vbat_min_v",       7.2f,  6.0f, 8.4f, "Minimalni napeti baterie rakety pro start [V] (2S)" },
    { "req_cont3",        0,     0,    1,    "Vyzadovat kontinuitu PYRO3 pro start" },
    { "max_tilt_deg",     20,    2,    60,   "Max. naklon rakety na rampe pro start [deg]" },
    { "nose_axis",        0,     0,    6,    "Osa spicky IMU: 0 auto, 1 +X, 2 -X, 3 +Y, 4 -Y, 5 +Z, 6 -Z" },
    { "tlm_hz",           5,     1,    10,   "Frekvence LoRa telemetrie [Hz]" },
    { "locate_after_s",   30,    0,    900,  "Bzucak pro dohledani po pristani za [s] (0 = hned)" },
    { "cam_mode",         1,     0,    1,    "Kamera: 1 = zapnout a nahravat od READY" },
    { "ext_ign",          0,     0,    1,    "Povolit start bez rampy (externi zazeh, detekce akcelerometrem)" },
    { "bw_mode",          1,     0,    2,    "Breakwire: 0 nepouzito, 1 kterykoli, 2 oba" },
    { "decl_deg",         5,     -30,  30,   "Magneticka deklinace [deg]" },
    { "landed_still_s",   5,     2,    60,   "Pristani: v klidu po dobu [s]" },
    { "cont_min_mv",      300,   50,   3000, "Prah kontinuity pyro kanalu na ADS1115 [mV]" },
    { "main_min_delay_s", 1.5f,  0.5f, 10,   "Min. odstup PYRO2 po PYRO1 [s]" },
    { "ready_timeout_min",30,    1,    240,  "READY bez countdownu se vrati do IDLE po [min]" },
};

#endif // ZENITH_PARAMS_H
