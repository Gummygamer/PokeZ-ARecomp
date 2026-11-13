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
