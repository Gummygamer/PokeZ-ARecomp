// PCVR: the player character's arms follow the hands (runtime/vr_body.h). Pokémon Z-A 2.0.0 only.
// The character's skeleton: the game object's components are a vector at object + 0x148; the one with the vtable
// 0x7003c0d668 holds the model (component + 0x128), whose skeleton is at model + 0x68 (vtable 0x7003c07ca0):
//   +030 its parts (16 bytes each: the part, and at +10 its first joint's number - the body, then hair and clothes;
//        a part's joint names are a hash map, part + 0x50 the first node: next, hash, the name (hash, text, length), its
//        number in the part at + 0x30)
//   +070 u16 per joint: its place in the arrays below      +0b0 u16 per place: the parent's place
//   +050 per place (0x28 bytes): the words to OR into the "world matrix is stale" bits (+0d0) when it changes - itself
//        and everything below it
//   +190 per place (0x30 bytes): the joint in its parent - scale, rotation (x y z w), translation
//   +1b0 per place: the joint's matrix in the model (column-major), right while its stale bit is clear
//   +280 the model's matrix in the world
// (the game's own setters: f_700188bc90 the rotation, f_7000017a60 the matrix, made fresh.)
// (debug) LARECOMP_VR_SKEL_LOG=<frame>: at that frame every joint's name, parent and place in the log ("[vr-skel]").
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../hle.h"
#include "../platform.h"
#include "../rt.h"
#include "../vr_body.h"
#include "../vr_ik.h"

static bool guest(uint64_t a) { return a >= GUEST_LO + 0x3190000 && a < GUEST_HI - 0x1000 && !(a & 7); }
#define G64(a) (*(const uint64_t *)(uintptr_t)(a))

typedef struct {
    uint64_t m;           // the skeleton
    uint32_t n;           // joints
    const uint16_t *place, *parent;
    float *local;         // n x 12
    float *model;         // n x 16
} Skel;

static bool skeleton(uint64_t player, Skel *s) {
    uint64_t b = G64(player + 0x148), e = G64(player + 0x150);
    if (!guest(b) || e <= b || e - b > 0x2000) return false;
    for (uint64_t a = b; a < e; a += 8) {
        uint64_t c = G64(a);
        if (!guest(c) || G64(c) != 0x7003c0d668ull) continue;
        uint64_t model = G64(c + 0x128);
        if (!guest(model)) return false;
        uint64_t m = G64(model + 0x68);
        if (!guest(m) || G64(m) != 0x7003c07ca0ull) return false;
        uint64_t lb = G64(m + 0x190), le = G64(m + 0x198), rb = G64(m + 0x70), re = G64(m + 0x78), mb = G64(m + 0x1b0), pb = G64(m + 0xb0);
        if (!guest(lb) || le <= lb || (le - lb) % 0x30 || le - lb > 0x30 * 1024 || !guest(rb) || re - rb != (le - lb) / 0x30 * 2 || !guest(mb) ||
            !guest(pb)) return false;
        *s = (Skel){.m = m, .n = (uint32_t)((le - lb) / 0x30), .place = (const uint16_t *)(uintptr_t)rb, .parent = (const uint16_t *)(uintptr_t)pb,
                    .local = (float *)(uintptr_t)lb, .model = (float *)(uintptr_t)mb};
        return true;
    }
    return false;
}

// A joint's number by its name (-1: none). name NULL: every name into the log.
static int joint_named(const Skel *s, const char *name) {
    uint64_t pb = G64(s->m + 0x30), pe = G64(s->m + 0x38);
    if (!guest(pb) || pe <= pb || pe - pb > 16 * 64) return -1;
    size_t len = name ? strlen(name) : 0;
    for (uint64_t p = pb; p < pe; p += 16) {
        uint64_t part = G64(p), node;
        uint32_t first = *(const uint16_t *)(uintptr_t)(p + 10);
        if (!guest(part)) continue;
        int guard = 0;
        for (node = G64(part + 0x50); guest(node) && guard < 2048; node = G64(node), guard++) {
            uint64_t text = G64(node + 0x18), tl = G64(node + 0x20), k = G64(node + 0x30) + first;
            if (text < GUEST_LO || text >= GUEST_HI || tl > 80 || k >= s->n) continue;
            if (!name) {
                uint32_t at = s->place[k], up = s->parent[at];
                const float *l = s->local + at * 12, *m = s->model + at * 16;
                LOG("[vr-skel] %3u place %3u parent's %3u %-28.*s r (%.3f %.3f %.3f %.3f) t (%.3f %.3f %.3f) in model (%.3f %.3f %.3f) x (%.2f %.2f %.2f) y (%.2f %.2f %.2f)\n",
                    (unsigned)k, at, up, (int)tl, (const char *)(uintptr_t)text, l[4], l[5], l[6], l[7], l[8], l[9], l[10], m[12], m[13], m[14],
                    m[0], m[1], m[2], m[4], m[5], m[6]);
            } else if (tl == len && !memcmp((const void *)(uintptr_t)text, name, len)) return (int)k;
        }
    }
    return -1;
}

// ---- the arms
// The joints, by the game's names ("left_" / "right_" before each): the arm's bones run along their own +x (the upper
// arm 0.24 m, the forearm 0.21), the elbow and every finger joint bend about their own +y (the elbow by a negative
// angle: the forearm goes towards the upper arm's +z). The hand's +z is the thumb's side; its +y is the back of the
// left hand and the palm of the right one (the right side's joints are the left's mirrored).
enum { J_SHOULDER, J_ARM1, J_ARM2, J_HAND, J_ARM2_ROLL, J_HAND_ROLL, J_FINGER, J_COUNT = J_FINGER + 15 };
static const char *const joint_names[J_COUNT] = {"shoulder", "arm_01", "arm_02", "hand", "arm_02_roll", "hand_roll",
    "thumb_01", "thumb_02", "thumb_03", "index_01", "index_02", "index_03", "middle_01", "middle_02", "middle_03",
    "ring_01", "ring_02", "ring_03", "pinky_01", "pinky_02", "pinky_03"};
// A finger's first joint with the hand open (the left one's; the right one's: x and z negated), before its bend
static const float finger_base[5][4] = {{0.028f, -0.261f, -0.122f, 0.957f}, {-0.706f, 0.0f, -0.085f, 0.703f}, {-0.741f, 0.0f, -0.157f, 0.653f},
                                        {-0.067f, 0.0f, -0.043f, 0.997f}, {-0.067f, 0.0f, -0.071f, 0.995f}};

static struct { uint64_t m; uint32_t n; int j[2][J_COUNT]; } names;  // (the joints' numbers, found once per skeleton)
static float weight[2];  // (how much of each arm is the hand's now: in over a few frames)

static void mat_quat(const float x[3], const float y[3], const float z[3], float q[4]) {  // (the rotation whose columns are x y z)
    float t = x[0] + y[1] + z[2];
    if (t > 0) { float r = sqrtf(t + 1) * 2; q[3] = r / 4, q[0] = (y[2] - z[1]) / r, q[1] = (z[0] - x[2]) / r, q[2] = (x[1] - y[0]) / r; }
    else if (x[0] > y[1] && x[0] > z[2]) { float r = sqrtf(1 + x[0] - y[1] - z[2]) * 2; q[3] = (y[2] - z[1]) / r, q[0] = r / 4, q[1] = (y[0] + x[1]) / r, q[2] = (z[0] + x[2]) / r; }
    else if (y[1] > z[2]) { float r = sqrtf(1 + y[1] - x[0] - z[2]) * 2; q[3] = (z[0] - x[2]) / r, q[0] = (y[0] + x[1]) / r, q[1] = r / 4, q[2] = (z[1] + y[2]) / r; }
    else { float r = sqrtf(1 + z[2] - x[0] - y[1]) * 2; q[3] = (x[1] - y[0]) / r, q[0] = (z[0] + x[2]) / r, q[1] = (z[1] + y[2]) / r, q[2] = r / 4; }
    vr_ik_quat_normalize(q, q);
}
static void cross(const float a[3], const float b[3], float o[3]) {
    float r[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    memcpy(o, r, sizeof r);
}
static float dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// A joint in the model, from the joints above it as they are now (the game's own matrices are made when asked for:
// they may be the last frame's here). Scales are left out: none on the way to an arm.
static void in_model(const Skel *s, uint32_t place, float q[4], float pos[3]) {
    uint32_t chain[48];
    int n = 0;
    for (uint32_t p = place; p < s->n && n < 48; p = s->parent[p]) chain[n++] = p;
    q[0] = q[1] = q[2] = 0, q[3] = 1, pos[0] = pos[1] = pos[2] = 0;
    while (n--) {
        const float *l = s->local + chain[n] * 12;
        float t[3];
        vr_ik_quat_rotate(q, l + 8, t);
        for (int i = 0; i < 3; i++) pos[i] += t[i];
        vr_ik_quat_mul(q, l + 4, q);
    }
    vr_ik_quat_normalize(q, q);
}

// A joint's rotation in its parent becomes q, by the weight w; its matrix and those below it are stale from here
static void turn(const Skel *s, int joint, const float q[4], float w) {
    if (joint < 0) return;
    uint32_t p = s->place[joint];
    float *l = s->local + p * 12 + 4, r[4];
    float sign = l[0] * q[0] + l[1] * q[1] + l[2] * q[2] + l[3] * q[3] < 0 ? -1.0f : 1.0f;
    for (int i = 0; i < 4; i++) r[i] = l[i] + (sign * q[i] - l[i]) * w;
    if (!vr_ik_quat_normalize(r, r)) return;
    memcpy(l, r, sizeof r);
    uint64_t sb = G64(s->m + 0xd0), se = G64(s->m + 0xd8), masks = G64(s->m + 0x50);
    if (!guest(sb & ~7ull) || se <= sb || se - sb > 256 || !guest(masks)) return;
    uint64_t mask = G64(masks + (uint64_t)p * 0x28);
    if (mask < GUEST_LO || mask >= GUEST_HI) return;
    for (uint64_t i = 0; i < (se - sb) / 4; i++) ((uint32_t *)(uintptr_t)sb)[i] |= ((const uint32_t *)(uintptr_t)mask)[i];
}

static void roty(float a, float q[4]) { q[0] = 0, q[1] = sinf(a / 2), q[2] = 0, q[3] = cosf(a / 2); }

// (tests on the window) LARECOMP_VR_FAKE_HANDS=<lx>,<ly>,<lz>,<rx>,<ry>,<rz>[,<grip>]: hands that stay where they are,
// metres from the head (x right, y up, z back), flat, fingers ahead
static bool fake_hand(int h, const VrPose *head, VrPose *out, float *grip) {
    static float v[7];
    static int have = -1;
    if (have < 0) {
        const char *e = ENV("LARECOMP_VR_FAKE_HANDS");
        have = e && sscanf(e, "%f,%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) >= 6;
    }
    if (!have) return false;
    *out = (VrPose){.pos = {head->pos[0] + v[h * 3], head->pos[1] + v[h * 3 + 1], head->pos[2] + v[h * 3 + 2]},
                    .rot = {0.70710678f, 0, 0, 0.70710678f}, .valid = true};
    *grip = v[6];
    return true;
}

static float bend(const VrPose *a, const VrPose *b) {  // (the angle between two of the hand's tracked bones: their -z)
    static const float back[3] = {0, 0, -1};
    float u[3], v[3];
    if (!a->valid || !b->valid) return 0.2f;
    vr_ik_quat_rotate(a->rot, back, u), vr_ik_quat_rotate(b->rot, back, v);
    float c = dot(u, v);
    return acosf(c > 1 ? 1 : c < -1 ? -1 : c);
}

// LARECOMP_VR_ARMS=0: the arms stay the animation's. LARECOMP_VR_WRIST=<m>: how far behind the middle of the palm (a
// controller's grip) the wrist is (0.07).
void vr_body_targets(uint64_t player, const VrInput *input, const float eye[3], const float room[3][3], float scale) {
    Skel s;
    if (!skeleton(player, &s)) return;
    static long at = -2;
    static int on = -1;
    static float wrist_back;
    if (at == -2) {
        at = ENV("LARECOMP_VR_SKEL_LOG") ? atol(ENV("LARECOMP_VR_SKEL_LOG")) : -1;
        on = !ENV("LARECOMP_VR_ARMS") || atoi(ENV("LARECOMP_VR_ARMS"));
        wrist_back = ENV("LARECOMP_VR_WRIST") ? (float)atof(ENV("LARECOMP_VR_WRIST")) : 0.07f;
    }
    extern uint64_t renderer_present_count(void);
    if (at >= 0 && (long)renderer_present_count() >= at) {
        at = -1;
        LOG("[vr-skel] skeleton %llx: %u joints\n", (unsigned long long)s.m, s.n);
        joint_named(&s, NULL);
    }
    if (!on) return;
    if (names.m != s.m || names.n != s.n) {
        for (int h = 0; h < 2; h++)
            for (int k = 0; k < J_COUNT; k++) {
                char nm[48];
                snprintf(nm, sizeof nm, "%s_%s", h ? "right" : "left", joint_names[k]);
                names.j[h][k] = joint_named(&s, nm);
            }
        names.m = s.m, names.n = s.n;
        LOG("[vr] arms: the character's skeleton has %u joints, the left arm's are %d %d %d %d, the right arm's %d %d %d %d\n", s.n,
            names.j[0][J_SHOULDER], names.j[0][J_ARM1], names.j[0][J_ARM2], names.j[0][J_HAND], names.j[1][J_SHOULDER], names.j[1][J_ARM1],
            names.j[1][J_ARM2], names.j[1][J_HAND]);
    }
    const float *wm = (const float *)(uintptr_t)(s.m + 0x280);  // the model in the world: a turn and a place
    for (int h = 0; h < 2; h++) {
        const int *j = names.j[h];
        VrPose hand = input->hand[h];
        float grip = input->grip[h], trigger = input->trigger[h];
        bool tracked = input->hand_tracked[h];
        if (!hand.valid && fake_hand(h, &input->head, &hand, &grip)) tracked = false, trigger = grip;
        bool have = hand.valid && j[J_ARM1] >= 0 && j[J_ARM2] >= 0 && j[J_HAND] >= 0 && j[J_SHOULDER] >= 0;
        weight[h] = have ? (weight[h] > 0.8f ? 1.0f : weight[h] + 0.2f) : 0;
        if (!have) continue;  // (the animation's arm again at once: there is no hand to blend from)
        float w = weight[h];
        // the hand's axes in the play space: its fingers, the back of the hand
        static const float grip_f[3] = {0, -1, 0}, joint_f[3] = {0, 0, -1}, joint_b[3] = {0, 1, 0};
        float grip_b[3] = {h ? 1.0f : -1.0f, 0, 0}, f[3], b[3], pos[3];
        vr_ik_quat_rotate(hand.rot, tracked ? joint_f : grip_f, f);
        vr_ik_quat_rotate(hand.rot, tracked ? joint_b : grip_b, b);
        for (int i = 0; i < 3; i++) pos[i] = hand.pos[i] - input->head.pos[i] - (tracked ? 0 : f[i] * wrist_back);
        // into the world (the head is the camera's eye), then into the model
        float target[3], mf[3], mb[3];
        for (int i = 0; i < 3; i++) {
            float wp = 0, wf = 0, wb = 0;
            for (int k = 0; k < 3; k++) wp += room[i][k] * pos[k] * scale, wf += room[i][k] * f[k], wb += room[i][k] * b[k];
            target[i] = eye[i] + wp - wm[12 + i], mf[i] = wf, mb[i] = wb;
        }
        float t[3] = {dot(target, wm), dot(target, wm + 4), dot(target, wm + 8)};
        float fx[3] = {dot(mf, wm), dot(mf, wm + 4), dot(mf, wm + 8)}, by[3] = {dot(mb, wm), dot(mb, wm + 4), dot(mb, wm + 8)};
        // the arm as the animation has it, and the shoulder that stays
        float qs[4], ps[3], q1[4], p1[3], q2[4], p2[3], q3[4], p3[3];
        in_model(&s, s.place[j[J_SHOULDER]], qs, ps);
        in_model(&s, s.place[j[J_ARM1]], q1, p1);
        in_model(&s, s.place[j[J_ARM2]], q2, p2);
        in_model(&s, s.place[j[J_HAND]], q3, p3);
        // the elbow: below the line from the shoulder to the hand, out to the side and a little back
        float pole[3] = {(p1[0] + t[0]) / 2 + (h ? -0.25f : 0.25f), (p1[1] + t[1]) / 2 - 0.5f, (p1[2] + t[2]) / 2 - 0.2f}, e[3], wr[3];
        if (!vr_ik_solve_two_bone(p1, p2, p3, t, pole, e, wr)) continue;
        float x1[3], x2[3], y[3], z1[3], z2[3];
        for (int i = 0; i < 3; i++) x1[i] = e[i] - p1[i], x2[i] = wr[i] - e[i];
        if (!vr_ik_vec_normalize(x1, x1) || !vr_ik_vec_normalize(x2, x2)) continue;
        // the elbow's axis: the forearm lies on the upper arm's +z side (a straight arm: away from the elbow's pole)
        float d = dot(x2, x1);
        for (int i = 0; i < 3; i++) z1[i] = x2[i] - d * x1[i];
        if (!vr_ik_vec_normalize(z1, z1)) {
            float pd[3] = {p1[0] - pole[0], p1[1] - pole[1], p1[2] - pole[2]}, k = dot(pd, x1);
            for (int i = 0; i < 3; i++) z1[i] = pd[i] - k * x1[i];
            if (!vr_ik_vec_normalize(z1, z1)) continue;
        }
        cross(z1, x1, y), cross(x2, y, z2);
        float m1[4], m2[4], m3[4], l[4], inv[4];
        mat_quat(x1, y, z1, m1), mat_quat(x2, y, z2, m2);
        // the hand: its fingers along +x, +y the back of the left hand and the palm of the right one
        float hy[3], hz[3];
        if (!vr_ik_vec_normalize(fx, fx)) continue;
        d = dot(by, fx);
        for (int i = 0; i < 3; i++) hy[i] = (by[i] - d * fx[i]) * (h ? -1.0f : 1.0f);
        if (!vr_ik_vec_normalize(hy, hy)) continue;
        cross(fx, hy, hz);
        mat_quat(fx, hy, hz, m3);
        vr_ik_quat_conjugate(qs, inv), vr_ik_quat_mul(inv, m1, l), turn(&s, j[J_ARM1], l, w);
        vr_ik_quat_conjugate(m1, inv), vr_ik_quat_mul(inv, m2, l), turn(&s, j[J_ARM2], l, w);
        vr_ik_quat_conjugate(m2, inv), vr_ik_quat_mul(inv, m3, l), turn(&s, j[J_HAND], l, w);
        {   // the forearm's skin turns with the hand: the hand's turn about the forearm's own axis, half of it midway
            float tw[4] = {l[0], 0, 0, l[3]}, half[4];
            if (!vr_ik_quat_normalize(tw, tw)) tw[0] = 0, tw[3] = 1;
            if (tw[3] < 0) tw[0] = -tw[0], tw[3] = -tw[3];
            half[0] = tw[0], half[1] = half[2] = 0, half[3] = tw[3] + 1;
            vr_ik_quat_normalize(half, half);
            turn(&s, j[J_HAND_ROLL], tw, w), turn(&s, j[J_ARM2_ROLL], half, w);
        }
        // the fingers: as the headset sees them, or closed by the controller's grip and trigger
        for (int fi = 0; fi < 5; fi++) {
            float a[3];
            if (tracked) {
                const VrPose *jt = input->joint[h] + (fi ? 6 + (fi - 1) * 5 : 2);  // (the thumb has no bone inside the palm)
                a[0] = fi ? bend(jt, jt + 1) - 0.1f : 0.1f, a[1] = bend(jt + (fi ? 1 : 0), jt + (fi ? 2 : 1)), a[2] = bend(jt + (fi ? 2 : 1), jt + (fi ? 3 : 2));
            } else {
                float c = fi == 0 ? 0.6f * (grip > trigger ? grip : trigger) : fi == 1 ? trigger : grip;
                a[0] = 0.15f + c * (fi ? 1.1f : 0.3f), a[1] = 0.2f + c * 1.3f, a[2] = 0.1f + c * 0.9f;
            }
            float q[4], base[4] = {finger_base[fi][0] * (h ? -1 : 1), finger_base[fi][1], finger_base[fi][2] * (h ? -1 : 1), finger_base[fi][3]};
            vr_ik_quat_normalize(base, base);
            roty(a[0], q), vr_ik_quat_mul(base, q, q), turn(&s, j[J_FINGER + fi * 3], q, w);
            roty(a[1], q), turn(&s, j[J_FINGER + fi * 3 + 1], q, w);
            roty(a[2], q), turn(&s, j[J_FINGER + fi * 3 + 2], q, w);
        }
    }
}
