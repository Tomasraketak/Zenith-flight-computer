// =====================================================================
//  Zenith — flight decision logic (Master MCU)
//
//  Liftoff, burnout, apogee (PYRO1 drogue), main (PYRO2), backup (PYRO3),
//  landing. Pure C, no hardware: the Master calls zf_step() every loop and
//  fires whatever channels it returns; test/flight_test.cpp drives it with
//  simulated flights, failed parachutes and a dead navigation link.
//
//  DECISION RULES (all limits are parameters, see zenith_params.h)
//
//   BOOST -> COAST  burnout flag from the Position MCU, or axial specific
//                   force < -0.05 g for 100 ms, or burn_max_s elapsed.
//   COAST -> DROGUE (PYRO1) when ALL of
//                     t > apo_lockout_s and Mach < apo_mach_max
//                   and ANY of
//                     KF apogee vote, barometric apogee vote, vz < 0 for 200 ms
//                   -- or, without valid navigation, at apo_backup_s,
//                   -- or always at apo_max_s (absolute timer).
//   DROGUE -> MAIN  (PYRO2) alt < main_alt_m while descending, at least
//                   main_min_delay_s after PYRO1; nav lost -> main_backup_s.
//   PYRO3 (backup charges of BOTH chutes) when
//                     from drogue_check_s after PYRO1: descent > drogue_fail_vz
//                       for 0.5 s (drogue did not open), or
//                     from main_check_s after PYRO2: descent > main_fail_vz
//                       for 0.5 s (main did not open), or
//                     navigation lost: p3_blind_s after PYRO2.
//   MAIN -> LANDED  |vz| < 1 m/s and |a| ~ 1 g for landed_still_s, or 20 min.
// =====================================================================
#ifndef ZENITH_FLIGHT_H
#define ZENITH_FLIGHT_H

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "zenith_link.h"
#include "zenith_params.h"

#define ZF_CH1 0x01u
#define ZF_CH2 0x02u
#define ZF_CH3 0x04u

typedef enum {
    ZF_WHY_NONE = 0,
    ZF_WHY_APO_KF, ZF_WHY_APO_BARO, ZF_WHY_APO_VZ, ZF_WHY_APO_TIMER_NAVLOST, ZF_WHY_APO_TIMER_MAX,
    ZF_WHY_MAIN_ALT, ZF_WHY_MAIN_TIMER, ZF_WHY_MAIN_LOW_APOGEE,
    ZF_WHY_P3_DROGUE_FAIL, ZF_WHY_P3_MAIN_FAIL, ZF_WHY_P3_BLIND
} zf_why_t;

typedef struct {
    uint32_t now_ms;
    int      nav_ok;          // NAV fresh (< 200 ms) and VALID
    float    alt, vz, acc_axial_g, acc_mag_g, mach;
    uint16_t nav_flags;
} zf_in_t;

typedef struct {
    uint8_t  state;           // ZS_BOOST .. ZS_LANDED
    uint32_t t_liftoff, t_burnout, t_drogue, t_main, t_p3, t_still0, t_neg_vz0, t_fail0;
    uint8_t  fired;           // ZF_CH*
    uint8_t  why_drogue, why_main, why_p3;
    float    max_alt;
    float    max_vz;
    float    max_mach;
    float    max_g;
} zf_t;

static inline void zf_begin(zf_t *f, uint32_t now)
{
    memset(f, 0, sizeof(*f));
    f->state = ZS_BOOST;
    f->t_liftoff = now ? now : 1;
}

/** Elapsed seconds since an event timestamp (0 = never). */
static inline float zf_since(uint32_t now, uint32_t t) { return t ? (float)(uint32_t)(now - t) * 0.001f : -1.0f; }

/**
 * One decision step. Returns the channels to fire NOW (caller pulses them).
 * p = parameter values indexed by ZP_*.
 */
static inline uint8_t zf_step(zf_t *f, const zf_in_t *in, const float *p)
{
    const uint32_t now = in->now_ms;
    const float t = zf_since(now, f->t_liftoff);
    uint8_t fire = 0;

    if (in->nav_ok) {
        if (in->alt > f->max_alt) f->max_alt = in->alt;
        if (in->vz > f->max_vz) f->max_vz = in->vz;
        if (in->mach > f->max_mach) f->max_mach = in->mach;
        if (in->acc_mag_g > f->max_g) f->max_g = in->acc_mag_g;
    }

    // ------------------------------------------------------------- BOOST
    if (f->state == ZS_BOOST) {
        int burnt = 0;
        if (in->nav_ok && (in->nav_flags & ZL_NAV_BURNOUT)) burnt = 1;
        if (t > p[ZP_BURN_MAX_S]) burnt = 1;
        if (burnt) { f->state = ZS_COAST; f->t_burnout = now; }
    }

    // ------------------------------------------------------------- COAST
    if (f->state == ZS_COAST) {
        uint8_t why = ZF_WHY_NONE;
        if (t >= p[ZP_APO_LOCKOUT_S]) {
            if (in->nav_ok) {
                // own vz vote (independent of the Position MCU's latch)
                if (in->vz < 0.0f) { if (!f->t_neg_vz0) f->t_neg_vz0 = now; }
                else f->t_neg_vz0 = 0;
                const int slow = in->mach < p[ZP_APO_MACH_MAX];
                if (slow) {
                    if (in->nav_flags & ZL_NAV_APO_KF) why = ZF_WHY_APO_KF;
                    else if (in->nav_flags & ZL_NAV_APO_BARO) why = ZF_WHY_APO_BARO;
                    else if (f->t_neg_vz0 && zf_since(now, f->t_neg_vz0) >= 0.2f) why = ZF_WHY_APO_VZ;
                }
            } else if (t >= p[ZP_APO_BACKUP_S]) {
                why = ZF_WHY_APO_TIMER_NAVLOST;
            }
        }
        if (!why && t >= p[ZP_APO_MAX_S]) why = ZF_WHY_APO_TIMER_MAX;
        if (why) {
            fire |= ZF_CH1;
            f->fired |= ZF_CH1;
            f->why_drogue = why;
            f->t_drogue = now;
            f->state = ZS_DROGUE;
        }
    }

    // ------------------------------------------------------------ DROGUE
    else if (f->state == ZS_DROGUE) {
        const float td = zf_since(now, f->t_drogue);
        uint8_t why = ZF_WHY_NONE;
        if (td >= p[ZP_MAIN_MIN_DELAY_S]) {
            if (in->nav_ok) {
                if (in->alt < p[ZP_MAIN_ALT] && in->vz < -1.0f)
                    why = (f->max_alt < p[ZP_MAIN_ALT]) ? ZF_WHY_MAIN_LOW_APOGEE : ZF_WHY_MAIN_ALT;
            } else if (td >= p[ZP_MAIN_BACKUP_S]) {
                why = ZF_WHY_MAIN_TIMER;
            }
        }
        // drogue failure -> backup charges of both chutes
        if (p[ZP_P3_ENABLE] > 0.5f && !(f->fired & ZF_CH3) && td >= p[ZP_DROGUE_CHECK_S] && in->nav_ok) {
            if (in->vz < -p[ZP_DROGUE_FAIL_VZ]) {
                if (!f->t_fail0) f->t_fail0 = now;
                if (zf_since(now, f->t_fail0) >= 0.5f) {
                    fire |= ZF_CH3; f->fired |= ZF_CH3; f->t_p3 = now; f->why_p3 = ZF_WHY_P3_DROGUE_FAIL;
                }
            } else f->t_fail0 = 0;
        }
        if (why) {
            fire |= ZF_CH2;
            f->fired |= ZF_CH2;
            f->why_main = why;
            f->t_main = now;
            f->t_fail0 = 0;
            f->state = ZS_MAIN;
        }
    }

    // -------------------------------------------------------------- MAIN
    else if (f->state == ZS_MAIN) {
        const float tm = zf_since(now, f->t_main);
        if (p[ZP_P3_ENABLE] > 0.5f && !(f->fired & ZF_CH3)) {
            if (in->nav_ok && tm >= p[ZP_MAIN_CHECK_S]) {
                if (in->vz < -p[ZP_MAIN_FAIL_VZ]) {
                    if (!f->t_fail0) f->t_fail0 = now;
                    if (zf_since(now, f->t_fail0) >= 0.5f) {
                        fire |= ZF_CH3; f->fired |= ZF_CH3; f->t_p3 = now; f->why_p3 = ZF_WHY_P3_MAIN_FAIL;
                    }
                } else f->t_fail0 = 0;
            } else if (!in->nav_ok && p[ZP_P3_BLIND_S] > 0.0f && tm >= p[ZP_P3_BLIND_S]) {
                fire |= ZF_CH3; f->fired |= ZF_CH3; f->t_p3 = now; f->why_p3 = ZF_WHY_P3_BLIND;
            }
        }
        // landing
        const int still = in->nav_ok && fabsf(in->vz) < 1.0f &&
                          in->acc_mag_g > 0.85f && in->acc_mag_g < 1.15f;
        if (still) { if (!f->t_still0) f->t_still0 = now; }
        else f->t_still0 = 0;
        if ((f->t_still0 && zf_since(now, f->t_still0) >= p[ZP_LANDED_STILL_S] && tm > 3.0f) || t > 1200.0f)
            f->state = ZS_LANDED;
    }
    return fire;
}

// =====================================================================
//  Liftoff detector (Master side). Call every loop while a liftoff is
//  possible. Returns 1 once liftoff is confirmed.
//
//  mode 0 (countdown, after T-0 or rampa FIRED):
//        accel > liftoff_g for 60 ms   OR breakwire(s) broken 30 ms
//        OR pogo 5 V lost (rocket left the pad)
//  mode 1 (countdown, before T-0 — premature ignition):
//        accel > liftoff_g + 0.5 for 100 ms only
//  mode 2 (external ignition, READY):
//        accel > liftoff_g for 100 ms AND (breakwire broken OR pogo lost
//        OR Position's own liftoff flag)
// =====================================================================
typedef struct {
    uint32_t t_acc0, t_bw0;
} zf_lift_t;

static inline int zf_liftoff(zf_lift_t *l, int mode, uint32_t now, int nav_fresh, float acc_axial_g,
                             uint16_t nav_flags, int bw_broken, int umb_lost, const float *p)
{
    const float thr = p[ZP_LIFTOFF_G] + (mode == 1 ? 0.5f : 0.0f);
    if (nav_fresh && acc_axial_g > thr) { if (!l->t_acc0) l->t_acc0 = now ? now : 1; }
    else l->t_acc0 = 0;
    if (bw_broken) { if (!l->t_bw0) l->t_bw0 = now ? now : 1; }
    else l->t_bw0 = 0;
    const float ta = zf_since(now, l->t_acc0);
    const float tb = zf_since(now, l->t_bw0);
    switch (mode) {
    case 0:  return ta >= 0.06f || tb >= 0.03f || umb_lost;
    case 1:  return ta >= 0.10f;
    default: return ta >= 0.10f && (tb >= 0.03f || umb_lost || (nav_flags & ZL_NAV_LIFTOFF));
    }
}

static inline const char *zf_why_name(uint8_t w)
{
    static const char *n[] = { "-", "APO_KF", "APO_BARO", "APO_VZ", "APO_TIMER_NAVLOST", "APO_TIMER_MAX",
                               "MAIN_ALT", "MAIN_TIMER", "MAIN_LOW_APOGEE",
                               "P3_DROGUE_FAIL", "P3_MAIN_FAIL", "P3_BLIND" };
    return w < sizeof(n) / sizeof(n[0]) ? n[w] : "?";
}

#endif // ZENITH_FLIGHT_H
