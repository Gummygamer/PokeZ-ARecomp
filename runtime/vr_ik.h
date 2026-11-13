// Pure model-space arm IK and quaternion math. Quaternions are (x,y,z,w).
#pragma once
#include <math.h>
#include <float.h>

static inline int vr_ik_vec_normalize(const float v[3], float out[3]) {
    double n = hypot(hypot((double)v[0], v[1]), v[2]);
    if (!isfinite(n) || n < 1e-12) return 0;
    float r[3];
    for (int i = 0; i < 3; ++i) r[i] = (float)(v[i] / n);
    for (int i = 0; i < 3; ++i) out[i] = r[i];
    return 1;
}

static inline int vr_ik_quat_normalize(const float q[4], float out[4]) {
    double n = hypot(hypot((double)q[0], q[1]), hypot((double)q[2], q[3]));
    if (!isfinite(n) || n < 1e-12) return 0;
    float r[4];
    for (int i = 0; i < 4; ++i) r[i] = (float)(q[i] / n);
    for (int i = 0; i < 4; ++i) out[i] = r[i];
    return 1;
}

// Hamilton product: applies b, then a. Inputs should be unit quaternions.
static inline void vr_ik_quat_mul(const float a[4], const float b[4], float out[4]) {
    float r[4] = {
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]
    };
    for (int i = 0; i < 4; ++i) out[i] = r[i];
}

static inline void vr_ik_quat_conjugate(const float q[4], float out[4]) {
    float r[4] = {-q[0], -q[1], -q[2], q[3]};
    for (int i = 0; i < 4; ++i) out[i] = r[i];
}

// q must be normalized. All helpers allow output to alias an input.
static inline void vr_ik_quat_rotate(const float q[4], const float v[3], float out[3]) {
    float t[3] = {2*(q[1]*v[2]-q[2]*v[1]), 2*(q[2]*v[0]-q[0]*v[2]), 2*(q[0]*v[1]-q[1]*v[0])};
    float r[3] = {v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1],
                  v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2],
                  v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0]};
    for (int i = 0; i < 3; ++i) out[i] = r[i];
}

static inline int vr_ik_quat_shortest_arc(const float from[3], const float to[3], float out[4]) {
    float a[3], b[3];
    if (!vr_ik_vec_normalize(from, a) || !vr_ik_vec_normalize(to, b)) return 0;
    double c[3] = {(double)a[1]*b[2]-(double)a[2]*b[1],
                   (double)a[2]*b[0]-(double)a[0]*b[2],
                   (double)a[0]*b[1]-(double)a[1]*b[0]};
    double n = hypot(hypot(c[0], c[1]), c[2]);
    double dot = (double)a[0]*b[0]+(double)a[1]*b[1]+(double)a[2]*b[2];
    float r[4] = {0, 0, 0, 1};
    if (n > 1e-12) {
        double half = 0.5*atan2(n, dot), s = sin(half)/n;
        for (int i = 0; i < 3; ++i) r[i] = (float)(c[i]*s);
        r[3] = (float)cos(half);
    } else if (dot < 0) {
        // Opposite vectors: choose the least aligned cardinal axis.
        int k = fabsf(a[0]) < fabsf(a[1]) ? 0 : 1;
        if (fabsf(a[2]) < fabsf(a[k])) k = 2;
        float axis[3] = {0,0,0}; axis[k] = 1;
        float cross[3] = {a[1]*axis[2]-a[2]*axis[1], a[2]*axis[0]-a[0]*axis[2], a[0]*axis[1]-a[1]*axis[0]};
        if (!vr_ik_vec_normalize(cross, r)) return 0;
        r[3] = 0;
    }
    return vr_ik_quat_normalize(r, out);
}

// Preserve the two original segment lengths. Targets outside the reachable
// annulus are clamped. A singular pole falls back to the original elbow plane,
// then a deterministic perpendicular axis. Failure leaves both outputs intact.
static inline int vr_ik_solve_two_bone(const float upper[3], const float elbow[3],
    const float wrist[3], const float target[3], const float pole[3],
    float out_elbow[3], float out_wrist[3]) {
    double u[3], f[3], dir[3], bend[3];
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(upper[i]) || !isfinite(elbow[i]) || !isfinite(wrist[i]) ||
            !isfinite(target[i]) || !isfinite(pole[i])) return 0;
        u[i] = (double)elbow[i]-upper[i];
        f[i] = (double)wrist[i]-elbow[i];
        dir[i] = (double)target[i]-upper[i];
    }
    double a = hypot(hypot(u[0],u[1]),u[2]), b = hypot(hypot(f[0],f[1]),f[2]);
    if (a < 1e-8 || b < 1e-8) return 0;
    double d = hypot(hypot(dir[0],dir[1]),dir[2]);
    if (d < 1e-12) {
        for (int i = 0; i < 3; ++i) dir[i] = u[i]/a;
    } else {
        for (int i = 0; i < 3; ++i) dir[i] /= d;
    }
    double reach = fmax(fabs(a-b), fmin(a+b, d));
    double bn = 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        int k = fabs(dir[0]) < fabs(dir[1]) ? 0 : 1;
        if (fabs(dir[2]) < fabs(dir[k])) k = 2;
        for (int i = 0; i < 3; ++i)
            bend[i] = attempt == 0 ? (double)pole[i]-upper[i] : attempt == 1 ? u[i] : (i == k ? 1.0 : 0.0);
        double dot = bend[0]*dir[0]+bend[1]*dir[1]+bend[2]*dir[2];
        for (int i = 0; i < 3; ++i) bend[i] -= dot*dir[i];
        bn = hypot(hypot(bend[0],bend[1]),bend[2]);
        if (bn > 1e-8) break;
    }
    if (bn <= 1e-8) return 0;
    // Equal lengths can fold all the way to the shoulder without division by zero.
    double along = reach > 1e-12 ? (a*a-b*b+reach*reach)/(2*reach) : 0;
    along = fmax(-a, fmin(a, along));
    double height = sqrt(fmax(0, a*a-along*along));
    float e[3], w[3];
    for (int i = 0; i < 3; ++i) {
        double ev = upper[i]+along*dir[i]+height*bend[i]/bn;
        double wv = upper[i]+reach*dir[i];
        if (!isfinite(ev) || !isfinite(wv) || fabs(ev)>FLT_MAX || fabs(wv)>FLT_MAX) return 0;
        e[i] = (float)ev; w[i] = (float)wv;
    }
    for (int i = 0; i < 3; ++i) { out_elbow[i] = e[i]; out_wrist[i] = w[i]; }
    return 1;
}
