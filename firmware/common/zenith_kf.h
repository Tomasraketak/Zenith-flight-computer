// =====================================================================
//  Zenith — one-axis navigation Kalman filter  x = [pos, vel, acc_bias]
//
//  Same structure as Daidalos' ariadna_kf.h: attitude is supplied from
//  outside, so the 9-state ENU filter is exactly block-diagonal and runs as
//  three independent 3x3 filters. Every measurement is scalar and observes
//  one state directly -> no matrix inversion anywhere. Single precision
//  only (the RP2350 FPU has no double). Host-tested in test/nav_test.cpp.
// =====================================================================
#ifndef ZENITH_KF_H
#define ZENITH_KF_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float x[3];
    float P[3][3];
} zkf_t;

static inline void zkf_reset(zkf_t *k, float p0, float v0, float sig_p, float sig_v, float sig_b)
{
    k->x[0] = p0; k->x[1] = v0; k->x[2] = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) k->P[i][j] = 0.0f;
    k->P[0][0] = sig_p * sig_p;
    k->P[1][1] = sig_v * sig_v;
    k->P[2][2] = sig_b * sig_b;
}

/** Predict with measured acceleration u (m/s^2). F = [[1,dt,-dt^2/2],[0,1,-dt],[0,0,1]]. */
static inline void zkf_predict(zkf_t *k, float u, float dt, float acc_psd, float bias_rw)
{
    const float h = 0.5f * dt * dt;
    const float a = u - k->x[2];
    k->x[0] += k->x[1] * dt + a * h;
    k->x[1] += a * dt;

    float A[3][3];
    for (int j = 0; j < 3; j++) {
        A[0][j] = k->P[0][j] + dt * k->P[1][j] - h * k->P[2][j];
        A[1][j] = k->P[1][j] - dt * k->P[2][j];
        A[2][j] = k->P[2][j];
    }
    // P = A F^T  (F applied across the rows of A, not down the columns)
    for (int i = 0; i < 3; i++) {
        const float a0 = A[i][0], a1 = A[i][1], a2 = A[i][2];
        k->P[i][0] = a0 + dt * a1 - h * a2;
        k->P[i][1] = a1 - dt * a2;
        k->P[i][2] = a2;
    }
    const float qa = acc_psd * acc_psd, qb = bias_rw * bias_rw, dt2 = dt * dt;
    k->P[0][0] += qa * dt2 * dt * (1.0f / 3.0f);
    k->P[0][1] += qa * dt2 * 0.5f;
    k->P[1][0] += qa * dt2 * 0.5f;
    k->P[1][1] += qa * dt;
    k->P[2][2] += qb * dt;
}

/**
 * Scalar update of state idx with measurement z, variance R.
 * gate_sigma > 0: reject if |innovation| > gate_sigma * sqrt(S). Returns 1 if applied.
 */
static inline int zkf_correct(zkf_t *k, int idx, float z, float R, float gate_sigma)
{
    const float S = k->P[idx][idx] + R;
    if (!(S > 0.0f)) return 0;
    const float y = z - k->x[idx];
    if (!(y == y)) return 0;                                   // NaN measurement
    if (gate_sigma > 0.0f && (y * y) > (gate_sigma * gate_sigma * S)) return 0;
    const float iS = 1.0f / S;
    const float K0 = k->P[0][idx] * iS, K1 = k->P[1][idx] * iS, K2 = k->P[2][idx] * iS;
    k->x[0] += K0 * y; k->x[1] += K1 * y; k->x[2] += K2 * y;
    const float r0 = k->P[idx][0], r1 = k->P[idx][1], r2 = k->P[idx][2];
    k->P[0][0] -= K0 * r0; k->P[0][1] -= K0 * r1; k->P[0][2] -= K0 * r2;
    k->P[1][0] -= K1 * r0; k->P[1][1] -= K1 * r1; k->P[1][2] -= K1 * r2;
    k->P[2][0] -= K2 * r0; k->P[2][1] -= K2 * r1; k->P[2][2] -= K2 * r2;
    k->P[0][1] = k->P[1][0] = 0.5f * (k->P[0][1] + k->P[1][0]);
    k->P[0][2] = k->P[2][0] = 0.5f * (k->P[0][2] + k->P[2][0]);
    k->P[1][2] = k->P[2][1] = 0.5f * (k->P[1][2] + k->P[2][1]);
    return 1;
}

static inline float zkf_sigma(const zkf_t *k, int idx)
{
    return k->P[idx][idx] > 0.0f ? sqrtf(k->P[idx][idx]) : 0.0f;
}

static inline void zkf_clamp_bias(zkf_t *k, float lim)
{
    if (k->x[2] > lim) k->x[2] = lim;
    if (k->x[2] < -lim) k->x[2] = -lim;
}

#endif // ZENITH_KF_H
