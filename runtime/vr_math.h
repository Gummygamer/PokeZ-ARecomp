// Pure VR coordinate conversion helpers. Game matrices use column-major storage.
#pragma once
#include <math.h>
#include <stdbool.h>

static inline bool vr_world_velocity(const float vp[16], const float velocity[3], float scale, float out[3]) {
    // Clip x gives camera right; clip w gives forward for this RH perspective projection.
    // Drop pitch so a physical upward throw stays upward in the game world.
    for (int i = 0; i < 16; ++i) if (!isfinite(vp[i])) return false;
    for (int i = 0; i < 3; ++i) if (!isfinite(velocity[i])) return false;
    if (!isfinite(scale) || scale <= 0) return false;
    float rx = vp[0], rz = vp[8], fx = vp[3], fz = vp[11];
    float rn = hypotf(rx, rz), fn = hypotf(fx, fz);
    if (rn < 1e-6f || fn < 1e-6f) return false;
    float v[3] = {
        scale * (rx / rn * velocity[0] - fx / fn * velocity[2]),
        scale * velocity[1],
        scale * (rz / rn * velocity[0] - fz / fn * velocity[2])
    };
    for (int i = 0; i < 3; ++i) if (!isfinite(v[i])) return false;
    for (int i = 0; i < 3; ++i) out[i] = v[i];
    return true;
}

// A play-space vector in the world: rot (row-major, world = rot * v; vr_world_rotation) times v, times scale. False on
// anything not finite. Unlike vr_world_velocity, a throw's pitch is kept: the camera's own pitch is not in rot.
static inline bool vr_rot_apply(const float rot[3][3], const float v[3], float scale, float out[3]) {
    float r[3];
    for (int i = 0; i < 3; ++i) {
        r[i] = scale * (rot[i][0] * v[0] + rot[i][1] * v[1] + rot[i][2] * v[2]);
        if (!isfinite(r[i])) return false;
    }
    for (int i = 0; i < 3; ++i) out[i] = r[i];
    return true;
}

// The game's SetBallVelocity node gives the body cur + (param - cur) * k (k: its LerpRate) each time it runs. From what
// it is about to set (lerped) and its parameter, the body's own velocity before it: what keeps a thrown ball on the path
// the hand gave it. False when k leaves nothing of it (k near 1: the node sets its parameter outright) or not finite.
static inline bool vr_ball_unlerp(const float lerped[3], const float param[3], float k, float out[3]) {
    if (!isfinite(k) || k < 0 || k > 0.98f) return false;
    float r[3];
    for (int i = 0; i < 3; ++i) {
        r[i] = (lerped[i] - k * param[i]) / (1 - k);
        if (!isfinite(r[i])) return false;
    }
    for (int i = 0; i < 3; ++i) out[i] = r[i];
    return true;
}

// The mean of n vectors (samples of the hand around its release). False for none.
static inline bool vr_mean3(const float (*v)[3], int n, float out[3]) {
    if (n <= 0) return false;
    float s[3] = {0, 0, 0};
    for (int k = 0; k < n; ++k) for (int i = 0; i < 3; ++i) s[i] += v[k][i];
    for (int i = 0; i < 3; ++i) {
        s[i] /= (float)n;
        if (!isfinite(s[i])) return false;
    }
    for (int i = 0; i < 3; ++i) out[i] = s[i];
    return true;
}
