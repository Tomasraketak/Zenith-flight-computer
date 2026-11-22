// =====================================================================
//  Zenith — navigation estimator (runs on the Position MCU)
//
//  Pure arithmetic, no Arduino: the sketch feeds it sensor samples, a
//  host simulation (test/nav_test.cpp) feeds it a synthetic 3 km / Mach 1.3
//  flight with a lying barometer and a GPS that drops out above 4 g.
//
//  WHAT IT DOES
//   * ON THE PAD: attitude straight from low-passed gravity + tilt-compensated
//     magnetometer (no gyro drift), gyro bias learned while still, nose axis
//     auto-detected (the body axis that points up), zero-velocity updates.
//   * IN FLIGHT: attitude by integrating the gyro alone (exact rotation per
//     sample, so 10 rev/s of roll is no problem); the accelerometer measures
//     thrust/drag there, never gravity, so it must not steer the attitude.
//     World acceleration drives three 3-state Kalman filters (E, N, Up).
//   * BAROMETER: fused only while it can be believed —
//       - not during the burn (base-pressure / plume effects),
//       - not while Mach > mach_distrust (shock waves around the static ports
//         make the reading useless between ~0.75 and ~1.3 Mach; we stop
//         trusting it at 0.6 already because our own Mach is an estimate),
//       - not for 2 s after any pyro event (ejection charge pressurises the bay).
//     While untrusted the innovation gate's re-anchor counter is frozen, so a
//     bad barometer can never drag the filter. When trust returns the filter
//     re-anchors smoothly if the dead-reckoned altitude drifted.
//   * GPS: ignored above 4 g and for 2 s after (receiver loses lock / u-blox
//     airborne-4g model limit), gated at 5 sigma, latency-compensated,
//     re-anchored after a run of rejections (Daidalos approach).
//   * APOGEE votes (the Master decides): KF vertical speed < 0 for 150 ms in
//     coast, and a median-filtered barometric descent of `apo_baro_drop_m`
//     measured only over trusted samples.
//   * REBOOT IN FLIGHT: attitude is unknown, so it runs a vertical-only
//     barometric filter (ZN_MODE_BARO_ONLY) and still votes for apogee.
// =====================================================================
#ifndef ZENITH_NAV_H
#define ZENITH_NAV_H

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "zenith_link.h"
#include "zenith_kf.h"

#define ZN_G0        9.80665f
#define ZN_D2R       0.017453293f
#define ZN_R2D       57.29578f

typedef struct {
    int      valid;          // a NAV-PVT arrived
    uint8_t  fix_type;       // 3 = 3-D
    uint8_t  fix_ok;         // gnssFixOK
    uint8_t  sats;
    int32_t  lat_1e7, lon_1e7;
    float    h_msl;          // m
    float    vn, ve, vd;     // m/s
    float    h_acc, v_acc, s_acc;  // m, m, m/s
} zn_gps_t;

typedef struct {
    // barometer
    float baro_sigma;        // m, trusted
    float mach_distrust;     // stop trusting above this Mach
    float mach_retrust;      // trust again below this (hysteresis)
    uint32_t pyro_blank_ms;  // ignore baro after an ejection
    uint32_t burnout_blank_ms;
    // KF tuning
    float acc_psd_v, acc_psd_h, bias_rw, bias_lim;
    float gate_sigma;
    uint8_t baro_max_rejects;
    // GPS
    float gps_max_g;
    uint32_t gps_holdoff_ms;
    uint8_t gps_min_sats;
    float gps_max_hacc;
    uint8_t gps_max_rejects;
    float gps_latency_s;
    // events
    float liftoff_g;         // own liftoff detector: axial > this ...
    uint32_t liftoff_ms;     //  ... for this long
    float apo_baro_drop_m;
    uint32_t apo_kf_ms;
    // pad
    float still_acc_tol_g, still_gyro_dps;
    float declination_deg;
} zn_cfg_t;

static inline void zn_cfg_default(zn_cfg_t *c)
{
    c->baro_sigma = 0.8f;
    c->mach_distrust = 0.60f;
    c->mach_retrust = 0.50f;
    c->pyro_blank_ms = 2000u;
    c->burnout_blank_ms = 500u;
    c->acc_psd_v = 0.8f;
    c->acc_psd_h = 1.0f;
    c->bias_rw = 0.01f;
    c->bias_lim = 1.0f;
    c->gate_sigma = 5.0f;
    c->baro_max_rejects = 40;
    c->gps_max_g = 4.0f;
    c->gps_holdoff_ms = 2000u;
    c->gps_min_sats = 5;
    c->gps_max_hacc = 25.0f;
    c->gps_max_rejects = 10;
    c->gps_latency_s = 0.05f;
    c->liftoff_g = 3.0f;
    c->liftoff_ms = 60u;
    c->apo_baro_drop_m = 4.0f;
    c->apo_kf_ms = 150u;
    c->still_acc_tol_g = 0.05f;
    c->still_gyro_dps = 3.0f;
    c->declination_deg = 5.0f;   // central Europe ~ +5 deg
}

// ------------------------------------------------------------ quaternion
static inline void zn_q_norm(float q[4])
{
    const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-9f) { const float i = 1.0f / n; q[0]*=i; q[1]*=i; q[2]*=i; q[3]*=i; }
    else { q[0] = 1; q[1] = q[2] = q[3] = 0; }
}

/** v_world = R(q) v_body */
static inline void zn_q_rot(const float q[4], const float v[3], float o[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2*(y*y + z*z))*v[0] + 2*(x*y - w*z)*v[1] + 2*(x*z + w*y)*v[2];
    o[1] = 2*(x*y + w*z)*v[0] + (1 - 2*(x*x + z*z))*v[1] + 2*(y*z - w*x)*v[2];
    o[2] = 2*(x*z - w*y)*v[0] + 2*(y*z + w*x)*v[1] + (1 - 2*(x*x + y*y))*v[2];
}

/** Quaternion from rotation matrix whose ROWS are (E, N, U) expressed in body axes. */
static inline void zn_q_from_rows(const float E[3], const float N[3], const float U[3], float q[4])
{
    const float m00 = E[0], m01 = E[1], m02 = E[2];
    const float m10 = N[0], m11 = N[1], m12 = N[2];
    const float m20 = U[0], m21 = U[1], m22 = U[2];
    const float tr = m00 + m11 + m22;
    if (tr > 0) {
        const float s = sqrtf(tr + 1.0f) * 2;
        q[0] = 0.25f * s; q[1] = (m21 - m12) / s; q[2] = (m02 - m20) / s; q[3] = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const float s = sqrtf(1.0f + m00 - m11 - m22) * 2;
        q[0] = (m21 - m12) / s; q[1] = 0.25f * s; q[2] = (m01 + m10) / s; q[3] = (m02 + m20) / s;
    } else if (m11 > m22) {
        const float s = sqrtf(1.0f + m11 - m00 - m22) * 2;
        q[0] = (m02 - m20) / s; q[1] = (m01 + m10) / s; q[2] = 0.25f * s; q[3] = (m12 + m21) / s;
    } else {
        const float s = sqrtf(1.0f + m22 - m00 - m11) * 2;
        q[0] = (m10 - m01) / s; q[1] = (m02 + m20) / s; q[2] = (m12 + m21) / s; q[3] = 0.25f * s;
    }
    zn_q_norm(q);
}

static inline void zn_cross(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1]*b[2] - a[2]*b[1];
    o[1] = a[2]*b[0] - a[0]*b[2];
    o[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float zn_dot(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static inline float zn_norm3(float v[3])
{
    const float n = sqrtf(zn_dot(v, v));
    if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
    return n;
}

/** unit vector of nose axis code 1..6 (+X,-X,+Y,-Y,+Z,-Z) */
static inline void zn_axis_vec(uint8_t code, float v[3])
{
    v[0] = v[1] = v[2] = 0;
    if (code < 1 || code > 6) code = 1;
    const int i = (code - 1) / 2;
    v[i] = ((code - 1) % 2) ? -1.0f : 1.0f;
}

/** Standard atmosphere altitude difference from pressure ratio, m. */
static inline float zn_baro_alt(float p, float p0)
{
    if (!(p > 100.0f) || !(p0 > 100.0f)) return 0.0f;
    return 44330.77f * (1.0f - powf(p / p0, 0.190263f));
}

// =====================================================================
typedef struct {
    zn_cfg_t cfg;
    uint8_t  mode;
    uint8_t  nose_forced;       // 0 = auto
    uint8_t  nose_axis;         // in use, 1..6
    // attitude
    float    q[4];
    int      att_ok;
    float    f_lp[3];           // low-passed specific force (g), pad
    float    m_lp[3];           // low-passed mag (uT)
    int      mag_ok;
    float    gbias[3];          // dps
    float    f1g;               // |f| at rest, calibrates the 1 g scale
    float    still_t;           // s continuously still
    uint32_t calib_n;           // gyro-bias samples averaged at start
    float    calib_sum[3];
    // filters
    zkf_t    kx, ky, kz;
    int      pad_ref;
    float    p0, t0_k;          // pad pressure, Pa / temperature, K
    float    p_acc_sum, t_acc_sum; uint32_t p_acc_n;
    int32_t  org_lat, org_lon;  // GPS origin
    float    org_h;
    int      have_org;
    float    m_per_deg_lat, m_per_deg_lon;
    // barometer
    int      baro_ok, baro_trust;
    float    baro_alt;
    float    med_buf[5]; uint8_t med_n, med_i;
    float    baro_max;
    uint8_t  baro_rej;
    uint32_t last_baro_ms;
    // gps
    uint32_t last_gps_fused_ms, highg_end_ms;
    int      highg;
    uint8_t  gps_rej;
    int      gps_fix;
    // flight events
    int      liftoff, burnout, apo_kf, apo_baro, transonic;
    uint32_t t_liftoff_ms, t_burnout_ms, t_pyro_ms;
    uint32_t lift_t0, burn_t0, apo_t0;
    uint8_t  pyro_events_seen;
    float    max_alt;
    float    mach;
    // last sample
    float    f_body[3], acc_axial_g, acc_mag_g, az_w;
    float    imu_temp;
    uint32_t now_ms;
    uint8_t  seq;
    uint8_t  zupt_cnt;
} zn_t;

static inline void zn_init(zn_t *n)
{
    memset(n, 0, sizeof(*n));
    zn_cfg_default(&n->cfg);
    n->mode = ZN_MODE_INIT;
    n->q[0] = 1;
    n->nose_axis = 1;
    n->f1g = 1.0f;
    n->t0_k = 288.15f;
    zkf_reset(&n->kx, 0, 0, 1.0f, 0.1f, 0.2f);
    zkf_reset(&n->ky, 0, 0, 1.0f, 0.1f, 0.2f);
    zkf_reset(&n->kz, 0, 0, 2.0f, 0.1f, 0.2f);
}

/** Choose the body axis that points up (auto) or the forced one. */
static inline void zn_pick_nose(zn_t *n)
{
    if (n->nose_forced >= 1 && n->nose_forced <= 6) { n->nose_axis = n->nose_forced; return; }
    int best = 0; float bv = 0;
    for (int i = 0; i < 3; i++) if (fabsf(n->f_lp[i]) > bv) { bv = fabsf(n->f_lp[i]); best = i; }
    n->nose_axis = (uint8_t)(1 + 2 * best + (n->f_lp[best] < 0 ? 1 : 0));
}

/** Pad attitude from low-passed gravity and magnetometer. */
static inline void zn_level(zn_t *n)
{
    float U[3] = { n->f_lp[0], n->f_lp[1], n->f_lp[2] };
    if (zn_norm3(U) < 0.5f) return;
    float Nh[3];
    int use_mag = 0;
    if (n->mag_ok) {
        float m[3] = { n->m_lp[0], n->m_lp[1], n->m_lp[2] };
        const float d = zn_dot(m, U);
        Nh[0] = m[0] - d*U[0]; Nh[1] = m[1] - d*U[1]; Nh[2] = m[2] - d*U[2];
        if (zn_norm3(Nh) > 3.0f) use_mag = 1;   // > 3 uT horizontal component
    }
    if (!use_mag) {
        // no compass: call the most horizontal body axis "north"
        float best[3] = {1, 0, 0}; float bd = 2;
        for (int i = 0; i < 3; i++) {
            float a[3] = {0, 0, 0}; a[i] = 1;
            const float d = fabsf(zn_dot(a, U));
            if (d < bd) { bd = d; best[0] = a[0]; best[1] = a[1]; best[2] = a[2]; }
        }
        const float d = zn_dot(best, U);
        Nh[0] = best[0] - d*U[0]; Nh[1] = best[1] - d*U[1]; Nh[2] = best[2] - d*U[2];
        zn_norm3(Nh);
    }
    float E[3]; zn_cross(Nh, U, E); zn_norm3(E);
    // rotate the magnetic frame by the declination: true north = mag north rotated
    const float dc = n->cfg.declination_deg * ZN_D2R;
    if (use_mag && dc != 0.0f) {
        const float c = cosf(dc), s = sinf(dc);
        float N2[3], E2[3];
        for (int i = 0; i < 3; i++) { N2[i] = c*Nh[i] - s*E[i]; E2[i] = c*E[i] + s*Nh[i]; }
        memcpy(Nh, N2, sizeof Nh); memcpy(E, E2, sizeof E);
    }
    zn_q_from_rows(E, Nh, U, n->q);
    n->att_ok = 1;
}

/** Capture pad reference (pressure, temperature, GPS origin) from what was averaged. */
static inline void zn_padref(zn_t *n)
{
    if (n->p_acc_n >= 10) {
        n->p0 = n->p_acc_sum / (float)n->p_acc_n;
        n->t0_k = n->t_acc_sum / (float)n->p_acc_n + 273.15f;
        n->pad_ref = 1;
    }
    n->p_acc_sum = n->t_acc_sum = 0; n->p_acc_n = 0;
    n->have_org = 0;                 // next good fix becomes the origin
    n->baro_max = 0; n->med_n = 0;
    n->max_alt = 0;
    zkf_reset(&n->kx, 0, 0, 0.5f, 0.05f, 0.2f);
    zkf_reset(&n->ky, 0, 0, 0.5f, 0.05f, 0.2f);
    zkf_reset(&n->kz, 0, 0, 0.5f, 0.05f, 0.2f);
    zn_pick_nose(n);
}

static inline void zn_enter_flight(zn_t *n, uint32_t now)
{
    if (n->mode == ZN_MODE_FLIGHT || n->mode == ZN_MODE_BARO_ONLY) return;
    n->mode = n->att_ok ? ZN_MODE_FLIGHT : ZN_MODE_BARO_ONLY;
    n->liftoff = 1;
    n->t_liftoff_ms = now;
    n->max_alt = n->kz.x[0];
    n->baro_max = -1e9f;
    n->med_n = 0;
    // pad reference must exist; if nobody asked for one, use what we averaged
    if (!n->pad_ref) zn_padref(n);
}

/** Return to pad mode (false liftoff / hangfire on the pad). */
static inline void zn_back_to_pad(zn_t *n)
{
    n->mode = ZN_MODE_PAD;
    n->liftoff = n->burnout = n->apo_kf = n->apo_baro = 0;
    n->max_alt = 0;
}

/** Restart in flight (watchdog) without a pad reference: baro-only. */
static inline void zn_start_in_flight(zn_t *n, float p0_guess, float t_flight_s)
{
    n->mode = ZN_MODE_BARO_ONLY;
    n->liftoff = 1; n->burnout = (t_flight_s > 6.0f);
    n->p0 = p0_guess; n->pad_ref = (p0_guess > 1000.0f);
    n->baro_max = -1e9f;
    zkf_reset(&n->kz, 0, 0, 50.0f, 50.0f, 0.1f);
}

// ---------------------------------------------------------------- IMU
/**
 * One fused IMU sample. a_g: specific force in g (board axes), g_dps: rate.
 */
static inline void zn_imu(zn_t *n, const float a_g[3], const float g_dps[3], float dt, uint32_t now)
{
    n->now_ms = now;
    if (!(dt > 0.0f) || dt > 0.05f) dt = 0.001f;
    memcpy(n->f_body, a_g, sizeof(n->f_body));
    const float fm = sqrtf(zn_dot(a_g, a_g));
    n->acc_mag_g = fm;
    float nose[3]; zn_axis_vec(n->nose_axis, nose);
    n->acc_axial_g = zn_dot(a_g, nose);

    float gc[3] = { g_dps[0] - n->gbias[0], g_dps[1] - n->gbias[1], g_dps[2] - n->gbias[2] };
    const float gn = sqrtf(zn_dot(gc, gc));

    // ---- high-g bookkeeping for GPS ---------------------------------------
    if (fm > n->cfg.gps_max_g) { n->highg = 1; n->highg_end_ms = now; }
    else if (n->highg && (uint32_t)(now - n->highg_end_ms) > n->cfg.gps_holdoff_ms) n->highg = 0;

    if (n->mode == ZN_MODE_INIT || n->mode == ZN_MODE_PAD) {
        // gyro bias: 1 s average at start, then slow learning while still
        const int still = fabsf(fm - n->f1g) < n->cfg.still_acc_tol_g && gn < n->cfg.still_gyro_dps;
        if (n->mode == ZN_MODE_INIT) {
            // plain averages; a sample that is clearly not gravity (a knock)
            // is left out of both sums
            if (fm > 0.8f && fm < 1.2f) {
                for (int i = 0; i < 3; i++) { n->calib_sum[i] += g_dps[i]; n->f_lp[i] += a_g[i]; }
                n->calib_n++;
            }
            if (n->calib_n >= 800) {
                for (int i = 0; i < 3; i++) {
                    n->gbias[i] = n->calib_sum[i] / (float)n->calib_n;
                    n->f_lp[i] /= (float)n->calib_n;
                }
                n->f1g = sqrtf(zn_dot(n->f_lp, n->f_lp));
                if (n->f1g < 0.8f || n->f1g > 1.2f) n->f1g = 1.0f;
                zn_pick_nose(n);
                zn_level(n);
                n->mode = ZN_MODE_PAD;
            }
            n->still_t = 0;
        } else {
            n->still_t = still ? n->still_t + dt : 0.0f;
            // Level only from samples that look like gravity: the first tens of
            // ms of the burn (before liftoff is confirmed) must not tip the
            // attitude toward the thrust vector.
            if (fabsf(fm - n->f1g) < 0.2f) {
                const float al = dt / 0.5f;
                for (int i = 0; i < 3; i++) n->f_lp[i] += al * (a_g[i] - n->f_lp[i]);
                zn_level(n);
            }
            if (n->still_t > 1.0f) {
                const float ab = dt / 20.0f;
                for (int i = 0; i < 3; i++) n->gbias[i] += ab * (g_dps[i] - n->gbias[i]);
            }
            // 1 g scale: learnt whenever the gyro is quiet and |f| is near 1 g
            // (independent of f1g itself, so a bad start cannot lock itself in)
            if (gn < n->cfg.still_gyro_dps && fabsf(fm - 1.0f) < 0.15f)
                n->f1g += (dt / 10.0f) * (fm - n->f1g);
            if (!n->pad_ref || n->still_t > 1.0f) zn_pick_nose(n);
        }
        // own liftoff detector (Master's LAUNCHED flag also switches us)
        if (n->mode == ZN_MODE_PAD && n->acc_axial_g > n->cfg.liftoff_g) {
            if (!n->lift_t0) n->lift_t0 = now ? now : 1;
            if ((uint32_t)(now - n->lift_t0) >= n->cfg.liftoff_ms) zn_enter_flight(n, n->lift_t0);
        } else {
            n->lift_t0 = 0;
        }
    } else if (n->mode == ZN_MODE_FLIGHT) {
        // exact rotation increment: q <- q * [cos(|w|dt/2), sin(|w|dt/2) w/|w|]
        const float wr = gn * ZN_D2R;
        const float ang = wr * dt;
        if (ang > 1e-9f) {
            const float s = sinf(0.5f * ang) / wr, c = cosf(0.5f * ang);
            const float bx = gc[0]*ZN_D2R*s, by = gc[1]*ZN_D2R*s, bz = gc[2]*ZN_D2R*s;
            const float w = n->q[0], x = n->q[1], y = n->q[2], z = n->q[3];
            n->q[0] = w*c - x*bx - y*by - z*bz;
            n->q[1] = w*bx + x*c + y*bz - z*by;
            n->q[2] = w*by - x*bz + y*c + z*bx;
            n->q[3] = w*bz + x*by - y*bx + z*c;
            zn_q_norm(n->q);
        }
    }

    // ---- world acceleration and KF prediction --------------------------------
    float fw[3];
    zn_q_rot(n->q, a_g, fw);
    const float sc = ZN_G0 / n->f1g;
    float ax = fw[0] * sc, ay = fw[1] * sc, az = fw[2] * sc - ZN_G0;
    if (n->mode == ZN_MODE_BARO_ONLY || !n->att_ok) { ax = ay = 0; az = 0; }
    n->az_w = az;

    if (n->mode == ZN_MODE_BARO_ONLY) {
        zkf_predict(&n->kz, 0.0f, dt, 25.0f, 0.0f);
    } else {
        zkf_predict(&n->kx, ax, dt, n->cfg.acc_psd_h, n->cfg.bias_rw);
        zkf_predict(&n->ky, ay, dt, n->cfg.acc_psd_h, n->cfg.bias_rw);
        zkf_predict(&n->kz, az, dt, n->cfg.acc_psd_v, n->cfg.bias_rw);
        zkf_clamp_bias(&n->kx, n->cfg.bias_lim);
        zkf_clamp_bias(&n->ky, n->cfg.bias_lim);
        zkf_clamp_bias(&n->kz, n->cfg.bias_lim);
    }

    // ---- zero-velocity update on the pad ---------------------------------------
    if (n->mode == ZN_MODE_PAD && n->still_t > 0.2f) {
        // only while really at rest: a ZUPT in the first ms of the burn
        // (before liftoff is confirmed) would throw away real velocity.
        // decimate: one ZUPT every 8 samples is plenty
        const float Rv = 0.03f * 0.03f, Rp = 0.3f * 0.3f;
        if (++n->zupt_cnt >= 8u) {
            n->zupt_cnt = 0;
            zkf_correct(&n->kx, 1, 0.0f, Rv, 0); zkf_correct(&n->kx, 0, 0.0f, Rp, 0);
            zkf_correct(&n->ky, 1, 0.0f, Rv, 0); zkf_correct(&n->ky, 0, 0.0f, Rp, 0);
            zkf_correct(&n->kz, 1, 0.0f, Rv, 0);
        }
    }

    // ---- Mach, burnout, apogee votes ---------------------------------------------
    const float spd = sqrtf(n->kx.x[1]*n->kx.x[1] + n->ky.x[1]*n->ky.x[1] + n->kz.x[1]*n->kz.x[1]);
    float tk = n->t0_k - 0.0065f * n->kz.x[0];
    if (tk < 200.0f) tk = 200.0f;
    n->mach = spd / (20.0468f * sqrtf(tk));
    if (n->mode == ZN_MODE_BARO_ONLY) n->mach = fabsf(n->kz.x[1]) / (20.0468f * sqrtf(tk));

    if (n->liftoff && !n->burnout) {
        const int dec = (n->mode == ZN_MODE_BARO_ONLY) ? 0 : (n->acc_axial_g < -0.05f);
        if (dec) {
            if (!n->burn_t0) n->burn_t0 = now ? now : 1;
            if ((uint32_t)(now - n->burn_t0) >= 100u) { n->burnout = 1; n->t_burnout_ms = now; }
        } else n->burn_t0 = 0;
    }
    if (n->liftoff) {
        if (n->kz.x[0] > n->max_alt) n->max_alt = n->kz.x[0];
        if (n->burnout && !n->apo_kf && n->kz.x[1] < 0.0f && n->mach < 0.5f) {
            if (!n->apo_t0) n->apo_t0 = now ? now : 1;
            if ((uint32_t)(now - n->apo_t0) >= n->cfg.apo_kf_ms) n->apo_kf = 1;
        } else if (!n->apo_kf) n->apo_t0 = 0;
    }
    // transonic flag with hysteresis
    if (n->mach > n->cfg.mach_distrust) n->transonic = 1;
    else if (n->mach < n->cfg.mach_retrust) n->transonic = 0;
}

static inline void zn_mag(zn_t *n, const float m[3])
{
    const float mm = sqrtf(zn_dot(m, m));
    if (mm < 10.0f || mm > 120.0f) { n->mag_ok = 0; return; }   // Earth field 25..65 uT
    if (!n->mag_ok) memcpy(n->m_lp, m, sizeof(n->m_lp));
    for (int i = 0; i < 3; i++) n->m_lp[i] += 0.05f * (m[i] - n->m_lp[i]);
    n->mag_ok = 1;
}

/** Master state feed: launched flag and pyro-event counter (baro blanking). */
static inline void zn_master(zn_t *n, uint8_t flags, uint8_t pyro_events, uint32_t now)
{
    if (pyro_events != n->pyro_events_seen) { n->pyro_events_seen = pyro_events; n->t_pyro_ms = now ? now : 1; }
    if ((flags & ZL_MST_LAUNCHED) && (n->mode == ZN_MODE_PAD || n->mode == ZN_MODE_INIT)) zn_enter_flight(n, now);
    // Master says we are on the pad and not launched, and we are still: undo a false liftoff
    if ((flags & ZL_MST_ON_PAD) && !(flags & ZL_MST_LAUNCHED) && n->mode == ZN_MODE_FLIGHT &&
        n->acc_mag_g > 0.9f && n->acc_mag_g < 1.1f && fabsf(n->kz.x[1]) < 2.0f &&
        (uint32_t)(now - n->t_liftoff_ms) > 5000u) {
        zn_back_to_pad(n);
    }
}

// ---------------------------------------------------------------- baro
static inline float zn_median5(const float *b, uint8_t cnt)
{
    float t[5]; uint8_t m = cnt > 5 ? 5 : cnt;
    for (uint8_t i = 0; i < m; i++) t[i] = b[i];
    for (uint8_t i = 1; i < m; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j+1] = t[j]; j--; } t[j+1] = v; }
    return t[m / 2];
}

static inline void zn_baro(zn_t *n, float p_pa, float t_c, uint32_t now)
{
    if (!(p_pa > 1000.0f && p_pa < 130000.0f)) { n->baro_ok = 0; return; }
    n->baro_ok = 1;
    n->last_baro_ms = now;
    if (n->mode == ZN_MODE_PAD || n->mode == ZN_MODE_INIT) {
        // running average for the pad reference (slow, 10 s)
        if (n->p_acc_n < 2000) { n->p_acc_sum += p_pa; n->t_acc_sum += t_c; n->p_acc_n++; }
        else {
            n->p_acc_sum += p_pa - n->p_acc_sum / (float)n->p_acc_n;
            n->t_acc_sum += t_c - n->t_acc_sum / (float)n->p_acc_n;
        }
        if (!n->pad_ref && n->p_acc_n >= 50) {    // provisional reference
            n->p0 = n->p_acc_sum / (float)n->p_acc_n;
            n->t0_k = n->t_acc_sum / (float)n->p_acc_n + 273.15f;
            n->pad_ref = 1;
        }
    }
    if (!n->pad_ref) return;
    const float h = zn_baro_alt(p_pa, n->p0);
    n->baro_alt = h;

    // ---- trust decision --------------------------------------------------------
    int trust = 1;
    if (n->mode == ZN_MODE_FLIGHT) {
        if (n->liftoff && !n->burnout) trust = 0;
        if (n->burnout && (uint32_t)(now - n->t_burnout_ms) < n->cfg.burnout_blank_ms) trust = 0;
        if (n->transonic) trust = 0;
    }
    if (n->t_pyro_ms && (uint32_t)(now - n->t_pyro_ms) < n->cfg.pyro_blank_ms) trust = 0;
    const int regained = trust && !n->baro_trust;
    n->baro_trust = trust;

    // ---- median-filtered barometric apogee vote ---------------------------------
    if (trust) {
        if (regained) n->med_n = 0;
        n->med_buf[n->med_i] = h; n->med_i = (uint8_t)((n->med_i + 1) % 5);
        if (n->med_n < 5) n->med_n++;
        if (n->med_n >= 5) {
            const float med = zn_median5(n->med_buf, n->med_n);
            if (regained || med > n->baro_max) n->baro_max = med;
            if (n->liftoff && n->burnout && n->mach < 0.5f && med < n->baro_max - n->cfg.apo_baro_drop_m)
                n->apo_baro = 1;
        }
    }

    // ---- fuse ------------------------------------------------------------------------
    if (n->mode == ZN_MODE_BARO_ONLY) {
        zkf_correct(&n->kz, 0, h, 2.0f * 2.0f, 0);
        return;
    }
    if (!trust) return;                  // reject counter frozen while untrusted
    const float R = n->cfg.baro_sigma * n->cfg.baro_sigma;
    if (zkf_correct(&n->kz, 0, h, R, n->cfg.gate_sigma)) { n->baro_rej = 0; return; }
    if (++n->baro_rej >= n->cfg.baro_max_rejects || regained) {
        // the dead-reckoned altitude drifted while the baro was blanked: re-anchor
        const float sv = zkf_sigma(&n->kz, 1);
        zkf_reset(&n->kz, h, n->kz.x[1], n->cfg.baro_sigma * 2.0f, sv > 2.0f ? sv : 2.0f, 0.2f);
        n->baro_rej = 0;
    }
}

// ---------------------------------------------------------------- GPS
static inline void zn_gps(zn_t *n, const zn_gps_t *g, float age_s, uint32_t now)
{
    n->gps_fix = g->valid && g->fix_type >= 3 && g->fix_ok;
    if (!n->gps_fix) return;
    if (g->sats < n->cfg.gps_min_sats || g->h_acc > n->cfg.gps_max_hacc) return;
    if (n->highg) return;
    if (n->mode == ZN_MODE_BARO_ONLY) return;
    if (!n->have_org) {
        if (n->mode != ZN_MODE_PAD) return;          // origin only on the pad
        n->org_lat = g->lat_1e7; n->org_lon = g->lon_1e7; n->org_h = g->h_msl;
        n->m_per_deg_lat = 111132.0f;
        n->m_per_deg_lon = 111320.0f * cosf((float)g->lat_1e7 * 1e-7f * ZN_D2R);
        n->have_org = 1;
        return;
    }
    if (n->mode == ZN_MODE_PAD) {
        // on the pad the vehicle does not move: average the origin slowly
        n->org_h += 0.02f * (g->h_msl - n->org_h);
        n->last_gps_fused_ms = now;
        return;
    }
    if (age_s < 0) age_s = 0;
    if (age_s > 1.0f) age_s = 1.0f;
    const float zx = (float)(g->lon_1e7 - n->org_lon) * 1e-7f * n->m_per_deg_lon + n->kx.x[1] * age_s;
    const float zy = (float)(g->lat_1e7 - n->org_lat) * 1e-7f * n->m_per_deg_lat + n->ky.x[1] * age_s;
    float sp = g->h_acc * 1.5f; if (sp < 2.0f) sp = 2.0f;
    float sv = g->s_acc * 1.5f; if (sv < 0.3f) sv = 0.3f;
    const int okx = zkf_correct(&n->kx, 0, zx, sp*sp, n->cfg.gate_sigma);
    const int oky = zkf_correct(&n->ky, 0, zy, sp*sp, n->cfg.gate_sigma);
    zkf_correct(&n->kx, 1, g->ve, sv*sv, n->cfg.gate_sigma);
    zkf_correct(&n->ky, 1, g->vn, sv*sv, n->cfg.gate_sigma);
    zkf_correct(&n->kz, 1, -g->vd, (sv*1.5f)*(sv*1.5f), n->cfg.gate_sigma);
    if (!n->baro_trust) {
        // barometer blanked (transonic): GPS height keeps the vertical channel honest
        float sh = g->v_acc * 2.0f; if (sh < 8.0f) sh = 8.0f;
        zkf_correct(&n->kz, 0, g->h_msl - n->org_h, sh*sh, n->cfg.gate_sigma);
    }
    if (okx && oky) { n->gps_rej = 0; n->last_gps_fused_ms = now; }
    else if (++n->gps_rej >= n->cfg.gps_max_rejects) {
        zkf_reset(&n->kx, zx, g->ve, sp, sv, 0.2f);
        zkf_reset(&n->ky, zy, g->vn, sp, sv, 0.2f);
        n->gps_rej = 0;
        n->last_gps_fused_ms = now;
    }
}

// ---------------------------------------------------------------- output
static inline void zn_output(zn_t *n, zl_nav_t *o, int imu_ok, int mag_present)
{
    memset(o, 0, sizeof(*o));
    o->t_ms = n->now_ms;
    o->alt = n->kz.x[0];
    o->vz = n->kz.x[1];
    o->px = n->kx.x[0]; o->py = n->ky.x[0];
    o->vx = n->kx.x[1]; o->vy = n->ky.x[1];
    o->az_w = n->az_w;
    o->acc_axial_g = n->acc_axial_g;
    o->acc_mag_g = n->acc_mag_g;
    o->mach = n->mach;
    o->baro_alt = n->baro_alt;
    o->sig_alt = zkf_sigma(&n->kz, 0);
    float nose[3], up[3];
    zn_axis_vec(n->nose_axis, nose);
    zn_q_rot(n->q, nose, up);
    float c = up[2]; if (c > 1) c = 1; if (c < -1) c = -1;
    o->tilt_deg = (n->att_ok && n->mode != ZN_MODE_BARO_ONLY) ? acosf(c) * ZN_R2D : -1.0f;
    o->qw = n->q[0]; o->qx = n->q[1]; o->qy = n->q[2]; o->qz = n->q[3];
    o->max_alt = n->max_alt;
    uint16_t f = 0;
    const int valid = imu_ok && n->pad_ref && n->baro_ok &&
                      (n->mode == ZN_MODE_PAD || n->mode == ZN_MODE_FLIGHT || n->mode == ZN_MODE_BARO_ONLY) &&
                      o->sig_alt < 30.0f && (o->alt == o->alt);
    if (valid) f |= ZL_NAV_VALID;
    if (n->baro_ok) f |= ZL_NAV_BARO_OK;
    if (n->baro_trust) f |= ZL_NAV_BARO_TRUST;
    if (n->last_gps_fused_ms && (uint32_t)(n->now_ms - n->last_gps_fused_ms) < 1500u) f |= ZL_NAV_GPS_FUSED;
    if (n->att_ok) f |= ZL_NAV_ATT_OK;
    if (n->liftoff) f |= ZL_NAV_LIFTOFF;
    if (n->burnout) f |= ZL_NAV_BURNOUT;
    if (n->apo_kf) f |= ZL_NAV_APO_KF;
    if (n->apo_baro) f |= ZL_NAV_APO_BARO;
    if (n->transonic) f |= ZL_NAV_TRANSONIC;
    if (n->highg) f |= ZL_NAV_HIGHG;
    if (n->pad_ref) f |= ZL_NAV_PAD_REF;
    if (imu_ok) f |= ZL_NAV_IMU_OK;
    if (mag_present && n->mag_ok) f |= ZL_NAV_MAG_OK;
    if (n->gps_fix) f |= ZL_NAV_GPS_FIX;
    if (n->mode == ZN_MODE_PAD && n->still_t > 1.0f) f |= ZL_NAV_STILL;
    o->flags = f;
    o->mode = n->mode;
    o->seq = n->seq++;
}

/** Roll / pitch / yaw (deg) of the body frame for display (ZYX, ENU). */
static inline void zn_euler(const zn_t *n, float *roll, float *pitch, float *yaw)
{
    const float w = n->q[0], x = n->q[1], y = n->q[2], z = n->q[3];
    *roll  = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * ZN_R2D;
    float s = 2*(w*y - z*x); if (s > 1) s = 1; if (s < -1) s = -1;
    *pitch = asinf(s) * ZN_R2D;
    // heading of body X from north, clockwise
    const float ex = 1 - 2*(y*y + z*z), nx = 2*(x*y + w*z);
    float h = atan2f(ex, nx) * ZN_R2D; if (h < 0) h += 360.0f;
    *yaw = h;
}

#endif // ZENITH_NAV_H
