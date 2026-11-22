// =====================================================================
//  Zenith host test: a simulated 3 km / Mach 1.3 flight through the REAL
//  navigation estimator (zenith_nav.h) and the REAL flight logic
//  (zenith_flight.h), with a barometer that lies in the transonic band,
//  during the burn and at every ejection, and a GPS that drops out above
//  4 g. Scenarios: nominal, drogue failure, main failure, navigation link
//  lost, no GPS, extreme transonic baro error, pad vibration.
//
//      make -C test        (or:  g++ -std=c++17 -O2 -I../firmware/common sim_test.cpp)
// =====================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <random>
#include <string>
#include "zenith_nav.h"
#include "zenith_flight.h"

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("   FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
                           else { printf("   ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Scenario {
    std::string name;
    bool drogue_fails = false;
    bool main_fails = false;
    bool nav_lost = false;        // Position link dies at t_nav_lost
    float t_nav_lost = 10.0f;
    bool no_gps = false;
    float transonic_err_m = 120;  // baro error in the 0.75..1.3 Mach band
    float main_alt = 300;
    uint32_t seed = 1;
};

struct Result {
    float t_apo_true = 0, apo_true = 0, max_mach = 0;
    float t_drogue = -1, t_main = -1, t_p3 = -1, alt_main_true = -1, t_landed = -1;
    float max_err_alt_subsonic = 0;
    uint8_t why_d = 0, why_m = 0, why_p3 = 0;
    bool false_liftoff = false;
};

// ----------------------------------------------------------- quaternions
static void qmul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
static void rot_inv(const float q[4], const float v[3], float o[3])   // world -> body
{
    const float qc[4] = { q[0], -q[1], -q[2], -q[3] };
    zn_q_rot(qc, v, o);
}

static Result run(const Scenario &sc, bool verbose)
{
    std::mt19937 rng(sc.seed);
    std::normal_distribution<float> N(0.0f, 1.0f);

    float par[ZP_COUNT];
    for (int i = 0; i < ZP_COUNT; i++) par[i] = ZP_TABLE[i].def;
    par[ZP_MAIN_ALT] = sc.main_alt;

    zn_t nav; zn_init(&nav);
    zf_t fl;  memset(&fl, 0, sizeof fl);
    zf_lift_t lift; memset(&lift, 0, sizeof lift);
    Result r;

    // truth: rocket nose = body +X. On the pad body +X points up, tilted 4 deg toward east.
    const float tilt0 = 4.0f * ZN_D2R;
    // q_true maps body -> ENU.  Start: rotate body X to (sin t, 0, cos t)
    // rotation taking X->Z is -90 deg about Y; then tilt
    float q_true[4];
    {
        const float ang = -(90.0f * ZN_D2R - tilt0);
        float qa[4] = { cosf(ang/2), 0, sinf(ang/2), 0 };
        memcpy(q_true, qa, sizeof q_true);
    }
    const float mag_w[3] = { 1.7f, 20.0f, -44.0f };     // uT, ENU (north + down), ~declination ignored
    const float gyro_bias[3] = { 0.4f, -0.3f, 0.2f };   // dps
    const float acc_bias[3] = { 0.01f, -0.015f, 0.01f };// g

    // vehicle state (world)
    double z = 0, vz = 0, x = 0, vx = 0;
    const float dt = 1.0f / 960.0f;
    const float t_pad = 30.0f;          // seconds on the pad before ignition
    const float t_burn = 2.4f, a_burn = 21.0f * ZN_G0;
    const double kdrag = 0.00040;       // per m, body; gives ~3 km apogee
    bool launched = false; float t_launch = 0;
    int chute = 0;                      // 0 none, 1 drogue, 2 main
    float t_chute_drogue = -1, t_chute_main = -1;
    uint8_t pyro_events = 0;
    uint32_t last_nav_ms = 0;
    zl_nav_t navout{}; bool have_nav = false;
    int state = ZS_COUNTDOWN;
    bool landed_logged = false;
    float baro_spike_until = -1;
    float last_gps_t = 0;

    const float t_end = t_pad + 900.0f;
    for (float t = 0; t < t_end; t += dt) {
        const uint32_t now = (uint32_t)(t * 1000.0f) + 1000;
        const float tf = launched ? t - t_launch : -1;

        // ---------------- truth dynamics -------------------------------------
        double a = 0;   // world vertical acceleration (true, not specific force)
        double ax = 0;
        if (t >= t_pad && !launched) { launched = true; t_launch = t; }
        if (launched) {
            const double sp = sqrt(vz*vz + vx*vx);
            double drag = kdrag * sp * sp;
            if (chute == 1) drag = 9.80665 * (sp*sp) / (25.0*25.0);
            if (chute == 2) drag = 9.80665 * (sp*sp) / (6.0*6.0);
            const double thrust = (tf < t_burn) ? a_burn : 0.0;
            const double ux = sp > 1e-3 ? vx / sp : 0, uz = sp > 1e-3 ? vz / sp : 1;
            // thrust along the nose (tilt), drag against velocity
            const double nz = cos(tilt0), nx = sin(tilt0);
            a  = thrust * nz - drag * uz - 9.80665;
            ax = thrust * nx - drag * ux;
            if (z <= 0 && a < 0 && tf > 5) { a = 0; ax = 0; vz = 0; vx = 0; }
            vz += a * dt; z += vz * dt; vx += ax * dt; x += vx * dt;
            if (z < 0) { z = 0; vz = 0; vx = 0; }
            if (vz <= 0 && r.t_apo_true == 0 && tf > 3) { r.t_apo_true = tf; r.apo_true = (float)z; }
            const float mach_t = (float)(sp / 340.0);
            if (mach_t > r.max_mach) r.max_mach = mach_t;
        }
        // roll 1.5 rev/s in flight about the nose axis (body +X)
        float wb[3] = { 0, 0, 0 };
        if (launched && z > 1) wb[0] = 540.0f;
        {
            const float ang = wb[0] * ZN_D2R * dt;
            float dq[4] = { cosf(ang/2), sinf(ang/2), 0, 0 }, qn[4];
            qmul(q_true, dq, qn); memcpy(q_true, qn, sizeof q_true); zn_q_norm(q_true);
        }

        // ---------------- sensors ---------------------------------------------
        float fw[3] = { (float)ax, 0.0f, (float)(a + 9.80665) };
        if (!launched || (z <= 0 && tf > 5)) { fw[0] = 0; fw[1] = 0; fw[2] = 9.80665f; }
        for (int i = 0; i < 3; i++) fw[i] /= ZN_G0;
        float fb[3]; rot_inv(q_true, fw, fb);
        // pad vibration bumps (wind, people): short 1.8 g spikes
        if (!launched && fmodf(t, 7.3f) < 0.02f) fb[0] += 0.8f;
        float am[3], gm[3];
        for (int i = 0; i < 3; i++) {
            am[i] = fb[i] + acc_bias[i] + 0.01f * N(rng);
            gm[i] = wb[i] + gyro_bias[i] + 0.05f * N(rng);
        }
        zn_imu(&nav, am, gm, dt, now);

        // magnetometer 100 Hz
        if ((int)(t * 960) % 10 == 0) {
            float mb[3]; rot_inv(q_true, mag_w, mb);
            for (int i = 0; i < 3; i++) mb[i] += 0.3f * N(rng);
            zn_mag(&nav, mb);
        }
        // barometer 100 Hz
        if ((int)(t * 960) % 10 == 0) {
            const float sp = (float)sqrt(vz*vz + vx*vx), mach = sp / 340.0f;
            float err = 0.3f * N(rng);
            if (launched && tf < t_burn) err += -25.0f;                         // plume / base pressure
            if (mach > 0.75f && mach < 1.3f) err += sc.transonic_err_m * (0.6f + 0.4f * sinf(t * 7.0f));
            if (t < baro_spike_until) err += -60.0f;                            // ejection overpressure
            const double h = z + err;
            const float p0 = 98000.0f;
            const float p = p0 * powf(1.0f - (float)h / 44330.77f, 1.0f / 0.190263f);
            zn_baro(&nav, p, 22.0f, now);
            if (launched && !(sp / 340.0f > 0.75f && sp / 340.0f < 1.3f) && tf > t_burn + 1 && have_nav) {
                const float e = fabsf(navout.alt - (float)z);
                if (e > r.max_err_alt_subsonic && chute == 0) r.max_err_alt_subsonic = e;
            }
        }
        // GPS 5 Hz, lost above 4 g, 3 s re-acquisition
        if (!sc.no_gps && t - last_gps_t >= 0.2f) {
            last_gps_t = t;
            static float lost_until = -1;
            if (fabsf((float)a) / ZN_G0 + 1 > 4.0f) lost_until = t + 3.0f;
            zn_gps_t g{};
            g.valid = 1; g.fix_ok = 1; g.fix_type = 3; g.sats = 12;
            if (t < lost_until) { g.fix_ok = 0; g.fix_type = 0; }
            g.lat_1e7 = (int32_t)(49.8 * 1e7 + (0 / 111132.0) * 1e7);
            g.lon_1e7 = (int32_t)((16.69 + x / (111320.0 * cos(49.8 * ZN_D2R))) * 1e7);
            g.h_msl = (float)(300.0 + z + 2.0f * N(rng));
            g.vn = 0.1f * N(rng); g.ve = (float)vx + 0.1f * N(rng); g.vd = (float)-vz + 0.15f * N(rng);
            g.h_acc = 2.0f; g.v_acc = 3.0f; g.s_acc = 0.2f;
            zn_gps(&nav, &g, 0.05f, now);
        }

        // ---------------- Position -> Master at 100 Hz --------------------------
        bool link_alive = !(sc.nav_lost && launched && tf > sc.t_nav_lost);
        if (now - last_nav_ms >= 10) {
            last_nav_ms = now;
            if (link_alive) { zn_output(&nav, &navout, 1, 1); have_nav = true; }
            // Master state back to Position
            uint8_t mflags = 0;
            if (state >= ZS_BOOST && state <= ZS_LANDED) mflags |= ZL_MST_LAUNCHED;
            else mflags |= ZL_MST_ON_PAD;
            zn_master(&nav, mflags, pyro_events, now);
        }
        const bool nav_fresh = have_nav && link_alive;
        const bool nav_ok = nav_fresh && (navout.flags & ZL_NAV_VALID);

        // ---------------- Master logic ----------------------------------------------
        if (state == ZS_COUNTDOWN) {
            const bool after_t0 = t >= t_pad - 0.05f;
            const bool bw_broken = launched && z > 0.3;
            int lo = zf_liftoff(&lift, after_t0 ? 0 : 1, now, nav_fresh, navout.acc_axial_g, navout.flags,
                                bw_broken, 0, par);
            if (lo) {
                if (!launched) r.false_liftoff = true;
                zf_begin(&fl, now); state = ZS_BOOST;
            }
        } else if (state >= ZS_BOOST && state < ZS_LANDED) {
            zf_in_t in{};
            in.now_ms = now; in.nav_ok = nav_ok;
            in.alt = navout.alt; in.vz = navout.vz; in.acc_axial_g = navout.acc_axial_g;
            in.acc_mag_g = navout.acc_mag_g; in.mach = navout.mach; in.nav_flags = navout.flags;
            const uint8_t f = zf_step(&fl, &in, par);
            state = fl.state;
            if (f) {
                pyro_events++;
                baro_spike_until = t + 0.4f;
                if (f & ZF_CH1) { r.t_drogue = tf; r.why_d = fl.why_drogue;
                                  if (!sc.drogue_fails) t_chute_drogue = t + 1.0f; }
                if (f & ZF_CH2) { r.t_main = tf; r.why_m = fl.why_main; r.alt_main_true = (float)z;
                                  if (!sc.main_fails) t_chute_main = t + 1.5f; }
                if (f & ZF_CH3) { r.t_p3 = tf; r.why_p3 = fl.why_p3;
                                  t_chute_drogue = t + 1.0f; t_chute_main = t + 1.5f; }
                if (verbose) printf("     T+%6.2f  fire 0x%x  alt_true %6.0f m  nav_alt %6.0f vz %6.1f  mach %.2f\n",
                                    tf, f, z, navout.alt, navout.vz, navout.mach);
            }
            if (state == ZS_LANDED && !landed_logged) { landed_logged = true; r.t_landed = tf; }
        }
        if (t_chute_drogue > 0 && t >= t_chute_drogue && chute < 1) chute = 1;
        if (t_chute_main > 0 && t >= t_chute_main && chute < 2) chute = 2;
        if (landed_logged && tf > r.t_landed + 5) break;
        if (getenv("SIMDBG") && launched && tf < 3 && fmodf(tf, 0.25f) < dt)
            printf("     dbg T+%5.1f z %7.1f nav %7.1f vz %6.1f navvz %6.1f mach %.2f/%.2f flags %04x tilt %.1f a %.2f azw %.2f f1g %.3f bias %.3f mode %d\n",
                   tf, z, navout.alt, vz, navout.vz, sqrt(vz*vz+vx*vx)/340.0, navout.mach, navout.flags, navout.tilt_deg, a, navout.az_w, nav.f1g, nav.kz.x[2], nav.mode);
    }
    return r;
}

static void report(const Scenario &sc, const Result &r)
{
    printf("   apogee %.0f m at T+%.2f s, max Mach %.2f | drogue T+%.2f (%s) | main T+%.2f (%s) at %.0f m | P3 T+%.2f (%s) | landed T+%.1f\n",
           r.apo_true, r.t_apo_true, r.max_mach, r.t_drogue, zf_why_name(r.why_d), r.t_main, zf_why_name(r.why_m),
           r.alt_main_true, r.t_p3, zf_why_name(r.why_p3), r.t_landed);
    (void)sc;
}

int main()
{
    printf("== nominal\n");
    { Scenario s; s.name = "nominal"; Result r = run(s, true); report(s, r);
      CHECK(r.apo_true > 2500 && r.apo_true < 3600, "profile reaches ~3 km (%.0f m)", r.apo_true);
      CHECK(r.max_mach > 1.2f, "profile is supersonic (Mach %.2f)", r.max_mach);
      CHECK(!r.false_liftoff, "no false liftoff on the pad");
      CHECK(r.t_drogue >= r.t_apo_true - 0.5f && r.t_drogue <= r.t_apo_true + 1.5f, "drogue at apogee (%.2f vs %.2f)", r.t_drogue, r.t_apo_true);
      CHECK(r.why_d == ZF_WHY_APO_KF || r.why_d == ZF_WHY_APO_BARO || r.why_d == ZF_WHY_APO_VZ, "drogue by navigation");
      CHECK(fabsf(r.alt_main_true - 300) < 40, "main at 300 m (%.0f)", r.alt_main_true);
      CHECK(r.t_p3 < 0, "no backup charge");
      CHECK(r.t_landed > 0, "landing detected");
      CHECK(r.max_err_alt_subsonic < 30, "subsonic altitude error %.1f m", r.max_err_alt_subsonic); }

    printf("== main altitude 500 m\n");
    { Scenario s; s.main_alt = 500; Result r = run(s, false); report(s, r);
      CHECK(fabsf(r.alt_main_true - 500) < 40, "main at 500 m (%.0f)", r.alt_main_true); }

    printf("== drogue does not open\n");
    { Scenario s; s.drogue_fails = true; Result r = run(s, true); report(s, r);
      CHECK(r.why_p3 == ZF_WHY_P3_DROGUE_FAIL, "PYRO3 for drogue failure");
      CHECK(r.t_p3 > r.t_drogue + 3.9f && r.t_p3 < r.t_drogue + 8, "PYRO3 %.1f s after drogue", r.t_p3 - r.t_drogue);
      CHECK(r.t_landed > 0, "landed"); }

    printf("== main does not open\n");
    { Scenario s; s.main_fails = true; Result r = run(s, true); report(s, r);
      CHECK(r.why_p3 == ZF_WHY_P3_MAIN_FAIL, "PYRO3 for main failure");
      CHECK(r.t_p3 > r.t_main + 3.9f && r.t_p3 < r.t_main + 8, "PYRO3 %.1f s after main", r.t_p3 - r.t_main); }

    printf("== navigation link lost at T+10 s\n");
    { Scenario s; s.nav_lost = true; Result r = run(s, true); report(s, r);
      CHECK(r.why_d == ZF_WHY_APO_TIMER_NAVLOST && fabsf(r.t_drogue - 25) < 0.1f, "drogue on backup timer");
      CHECK(r.why_m == ZF_WHY_MAIN_TIMER, "main on backup timer");
      CHECK(r.why_p3 == ZF_WHY_P3_BLIND, "blind backup charge"); }

    printf("== no GPS at all, +300 m transonic baro error\n");
    { Scenario s; s.no_gps = true; s.transonic_err_m = 300; s.seed = 7; Result r = run(s, false); report(s, r);
      CHECK(r.t_drogue >= r.t_apo_true - 0.5f && r.t_drogue <= r.t_apo_true + 1.5f, "drogue at apogee (%.2f vs %.2f)", r.t_drogue, r.t_apo_true);
      CHECK(fabsf(r.alt_main_true - 300) < 50, "main at 300 m (%.0f)", r.alt_main_true);
      CHECK(r.max_err_alt_subsonic < 40, "subsonic altitude error %.1f m", r.max_err_alt_subsonic); }

    printf("== -300 m transonic baro error (reads LOW: must not fake an apogee)\n");
    { Scenario s; s.transonic_err_m = -300; s.seed = 3; Result r = run(s, false); report(s, r);
      CHECK(r.t_drogue >= r.t_apo_true - 0.5f, "no premature drogue (%.2f vs %.2f)", r.t_drogue, r.t_apo_true); }

    printf("\n%s (%d failures)\n", g_fail ? "SIM TEST FAILED" : "SIM TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}
