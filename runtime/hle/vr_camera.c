// PCVR: the first-person camera (runtime/vr.h). Pokémon Z-A 2.0.0 only: the generator's patch (tools/za_patches.py
// ZA_CAMERA) calls rt_za_camera from the renderer's "set this camera's parameters" call, before the parameters are
// stored - on a game thread, once per camera whose component changed, each frame. The parameters (0xd0 bytes):
//   +00 the viewport (0 0 1 1)          +10 ?, near, far, the vertical field of view (radians)
//   +20 the aspect, an orthographic box  +40 the camera's place in the world (metres, y up)
//   +50 the view matrix (the world -> the camera, which looks down its -z; column-major, as all of the game's)
//   +90 the projection (reversed depth, no far plane)
// The gameplay camera is the one with handle 1 (the others are the menus' and previews' own, at the origin). With a
// headset on, its parameters are written again from the head: the eye is at the player character's (the character's
// place in the world plus LARECOMP_VR_EYE_HEIGHT, 1.45 m, and LARECOMP_VR_EYE_FORWARD, 0.13 m - "ahead" in the room is where the game's own camera looks
// along the ground, so the right stick still turns the view, and the head's turns and steps are added to that), the
// field of view is the headset's (xr.c vr_set_fov: one picture wide enough for both eyes). What was used for each
// frame is kept (vr_render_view), so xr.c can tell the headset where the picture it shows was drawn from - the
// headset's compositor then holds the world still between the game's frames.
// The character: the player's object through the game's handle table (table at 0x7003ebd1a0: 0x20-byte slots, the
// object at slot + 0x20, 0x28 past its header; the field manager's handle at 0x700612b780, the player's at manager +
// 0x90), its world matrix at object + 0x40. Nothing of it is written.
// LARECOMP_VR_FP=0: the screen in front of the player as before (theater). =1 without a headset: the same camera from
// a head that stays where it is (LARECOMP_VR_FAKE_HEAD=<yaw>,<pitch> degrees, <x>,<y>,<z> m), for tests on the window.
// LARECOMP_LOG_CAMERA=<frame>: the first 40 parameter blocks from that frame on, as the game set them ("[vr-camera]").
#include <math.h>
#include <stdatomic.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../hle.h"
#include "../platform.h"
#include "../vr.h"
#include "../vr_body.h"

static pthread_mutex_t camera_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned camera_logs;
static float fov_tan[2] = {1.19f, 1.11f};  // (half the field of view, its tangent: across and up; until the headset says)
static VrRenderView ring[8];               // what the last frames were drawn with (ring[seq % 8])
static uint64_t ring_seq;
// The player's place in the play space (vr_recenter): the head's position there is the character's eyes, and its yaw
// (sine, cosine) is ahead. The headset's own origin is wherever its software was last recentred - often not where the
// player sits now: taken from it, the camera stood behind the character's head.
static struct { float pos[3], s, c; bool have, want; } origin = {.c = 1, .want = true};
// The play space -> the world of the last first-person camera (vr_world_rotation): the thrown ball's velocity
static struct { float rot[3][3], scale; uint64_t ns; } world_of_room;

void vr_recenter(void) {
    pthread_mutex_lock(&camera_lock);
    origin.want = true;
    pthread_mutex_unlock(&camera_lock);
}

static bool raw_forward(const VrPose *h, float f[2]) {  // (0 0 -1 turned by the head, laid on the floor)
    float x = h->rot[0], y = h->rot[1], z = h->rot[2], w = h->rot[3];
    f[0] = -2 * (x * z + w * y), f[1] = -(1 - 2 * (x * x + y * y));
    float n = hypotf(f[0], f[1]);
    if (!(n > 0.2f)) return false;
    f[0] /= n, f[1] /= n;
    return true;
}

bool vr_room_forward(const VrPose *head, float f[2]) {
    float r[2];
    if (!head->valid || !raw_forward(head, r)) return false;
    pthread_mutex_lock(&camera_lock);
    float s = origin.s, c = origin.c;
    pthread_mutex_unlock(&camera_lock);
    f[0] = c * r[0] + s * r[1], f[1] = -s * r[0] + c * r[1];  // (turned back by the origin's yaw)
    return true;
}

void vr_set_fov(float tan_h, float tan_v) {
    if (!(tan_h > 0.1f && tan_h < 10) || !(tan_v > 0.1f && tan_v < 10)) return;
    pthread_mutex_lock(&camera_lock);
    fov_tan[0] = tan_h, fov_tan[1] = tan_v;
    pthread_mutex_unlock(&camera_lock);
}

bool vr_render_view(int back, VrRenderView *out) {
    pthread_mutex_lock(&camera_lock);
    bool have = back >= 0 && back < 8 && ring_seq > (uint64_t)back;
    if (have) *out = ring[(ring_seq - 1 - (uint64_t)back) % 8];
    pthread_mutex_unlock(&camera_lock);
    return have && plat_now_ns() - out->ns < 500000000ull;  // (a menu, a loading screen: no camera set for a while)
}

// The frame whose scene was drawn with this view matrix (the renderer read it back from the frame's own camera
// constants, exec.c vk_scene_view): the head it was drawn from, exactly - which frame of the game's a picture belongs
// to is not a fixed number of frames back (the game thread runs ahead of the picture by one frame or two, and a
// guess one frame off made the world shake with every turn of the head).
bool vr_render_view_of(const float view[16], VrRenderView *out) {
    bool have = false;
    pthread_mutex_lock(&camera_lock);
    for (uint64_t k = 0; k < 8 && k < ring_seq && !have; k++) {
        const VrRenderView *rv = &ring[(ring_seq - 1 - k) % 8];
        float d = 0;
        for (int i = 0; i < 16; i++) d = fmaxf(d, fabsf(rv->view[i] - view[i]) / (1 + fabsf(view[i])));
        if (d < 1e-5f) *out = *rv, have = true;
    }
    pthread_mutex_unlock(&camera_lock);
    return have;
}

bool vr_world_rotation(float rot[3][3], float *scale) {
    pthread_mutex_lock(&camera_lock);
    bool have = world_of_room.ns && plat_now_ns() - world_of_room.ns < 500000000ull;
    if (have) memcpy(rot, world_of_room.rot, sizeof world_of_room.rot), *scale = world_of_room.scale;
    pthread_mutex_unlock(&camera_lock);
    return have;
}

static atomic_bool session_on;  // Written by the presenter, read by game and render threads.
void vr_session_made(void) { atomic_store(&session_on, true); }
void vr_session_lost(void) { atomic_store(&session_on, false); }

static int fp_mode(void) {  // 0 off, 1 with a headset, 2 forced (a head that stays where it is)
    const char *e = ENV("LARECOMP_VR_FP");
    if (e && !atoi(e)) return 0;
    if (vr_wanted()) return atomic_load(&session_on) ? 1 : 0;
    return e ? 2 : 0;
}

// How far ahead of the head's middle the eyes are, along the way the head looks: LARECOMP_VR_EYE_FORWARD (0.13 m). The
// camera's place, and with it where the head is from the camera (vr_head_cut).
static float eye_forward(void) {
    static float f = -1;
    if (f < 0) {
        f = ENV("LARECOMP_VR_EYE_FORWARD") ? (float)atof(ENV("LARECOMP_VR_EYE_FORWARD")) : 0.13f;
        if (!(f >= 0 && f < 1)) f = 0.13f;
    }
    return f;
}

// The player character's head is cut out of its own camera's picture (vr.h): LARECOMP_VR_HEAD_CUT=<metres>, the
// radius around the head's middle (0.18; 0: the head stays, as it does without VR). Decided once, from what was asked
// for - not from the session, which comes after the first shaders: without a headset it cuts what is around a point
// behind the game's own camera, which is nothing.
uint16_t vr_head_cut(void) {
    static int v = -1;
    if (v < 0) {
        const char *e = ENV("LARECOMP_VR_HEAD_CUT"), *fp = ENV("LARECOMP_VR_FP");
        float m = e ? (float)atof(e) : 0.18f;
        bool on = fp ? atoi(fp) != 0 : vr_wanted();
        v = on && m > 0 && m < 2.5f ? ((int)(eye_forward() * 100 + 0.5f) << 8) | (int)(m * 100 + 0.5f) : 0;
    }
    return (uint16_t)v;
}

// (nvn/output_scale.c) First person is on for this run: the game is to draw a picture of the headset's shape - *shape:
// its width over its height, against 16:9 - LARECOMP_VR_RES x 1080 lines high (1.5: 1620). A 16:9 picture stretched
// over a view as tall as it is wide, then shrunk to the window, was a blur in the headset.
bool vr_render_shape(double *shape, double *lines) {
    if (!fp_mode()) return false;
    static double res = -1;
    if (res < 0) {
        res = ENV("LARECOMP_VR_RES") ? atof(ENV("LARECOMP_VR_RES")) : 1.5;
        if (!(res >= 0.5 && res <= 4)) res = 1.5;
    }
    pthread_mutex_lock(&camera_lock);
    double a = fov_tan[0] / fov_tan[1];
    pthread_mutex_unlock(&camera_lock);
    *shape = floor(a / (16.0 / 9.0) * 50 + 0.5) / 50;  // (in steps: a field of view that moves by a hair reallocates nothing)
    *lines = res;
    return true;
}

// The player character's world matrix (column-major), or false. *object: the character's game object.
static bool player_matrix(float m[16], uint64_t *object) {
    if (g_game_is_la) return false;
    uint64_t table = *(const uint64_t *)(uintptr_t)0x7003ebd1a0ull, mh = *(const uint64_t *)(uintptr_t)0x700612b780ull;
    if (table < 0x7000000000ull || !mh) return false;
    const uint64_t *s = (const uint64_t *)(uintptr_t)(table + (mh & 0xFFFFF) * 0x20);
    if (s[2] != mh || s[4] < 0x7000000000ull) return false;  // (slot + 0x10: the handle it holds now, + 0x20: its object)
    uint64_t manager = s[4] - 0x28, ph = *(const uint64_t *)(uintptr_t)(manager + 0x90);
    if (!ph) return false;
    s = (const uint64_t *)(uintptr_t)(table + (ph & 0xFFFFF) * 0x20);
    if (s[2] != ph || s[4] < 0x7000000000ull) return false;
    *object = s[4] - 0x28;
    memcpy(m, (const void *)(uintptr_t)(s[4] - 0x28 + 0x40), 64);
    for (int i = 0; i < 16; i++) if (!isfinite(m[i])) return false;
    return fabsf(m[15] - 1) < 1e-3f;
}

static void quat_rows(const float q[4], float r[3][3]) {  // r[row][col]: turns a vector of the head's into the room's
    float x = q[0], y = q[1], z = q[2], w = q[3];
    r[0][0] = 1 - 2 * (y * y + z * z), r[0][1] = 2 * (x * y - w * z), r[0][2] = 2 * (x * z + w * y);
    r[1][0] = 2 * (x * y + w * z), r[1][1] = 1 - 2 * (x * x + z * z), r[1][2] = 2 * (y * z - w * x);
    r[2][0] = 2 * (x * z - w * y), r[2][1] = 2 * (y * z + w * x), r[2][2] = 1 - 2 * (x * x + y * y);
}

static bool fake_head(VrPose *h) {
    static float v[7];
    static int init;
    if (!init) {
        const char *e = ENV("LARECOMP_VR_FAKE_HEAD");
        if (e) sscanf(e, "%f,%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]);
        init = 1;
    }
    bool first = !origin.have;  // (until the recentring has taken "ahead" from a head that looks ahead: the yaw comes after)
    // (a sixth and seventh value: the head sways that many degrees to each side, once in that many seconds - a head that
    // is never quite still, as a real one)
    float sway = v[5] && v[6] > 0 ? v[5] * sinf((float)(plat_now_ns() % 1000000000000ull) * 1e-9f * 6.2831853f / v[6]) : 0;
    float yaw = first ? 0 : v[0] + sway, cy = cosf(yaw * 0.0087266f), sy = sinf(yaw * 0.0087266f), cp = cosf(v[1] * 0.0087266f), sp = sinf(v[1] * 0.0087266f);
    *h = (VrPose){.pos = {first ? 0 : v[2], first ? 0 : v[3], first ? 0 : v[4]}, .rot = {cy * sp, sy * cp, -sy * sp, cy * cp}, .valid = true};  // (yaw, then pitch)
    return true;
}

void rt_za_camera(uint64_t manager, uint64_t handle, uint64_t parameters) {
    float *p = (float *)(uintptr_t)parameters;
    const char *debug = ENV("LARECOMP_LOG_CAMERA");
    if (debug && atoi(debug)) {
        extern uint64_t renderer_present_count(void);
        pthread_mutex_lock(&camera_lock);
        if (renderer_present_count() >= (unsigned)atoi(debug) && camera_logs++ < 40) {
            LOG("[vr-camera] manager %llx handle %llx parameters %llx\n", (unsigned long long)manager, (unsigned long long)handle,
                (unsigned long long)parameters);
            for (unsigned i = 0; i < 52; i += 4) LOG("[vr-camera] +%02x %.6f %.6f %.6f %.6f\n", i * 4, p[i], p[i + 1], p[i + 2], p[i + 3]);
        }
        pthread_mutex_unlock(&camera_lock);
    }
    if (handle != 1) return;
    int mode = fp_mode();
    if (!mode) return;
    float *view = p + 20, *proj = p + 36;
    if (proj[11] != -1 || proj[15] != 0) return;  // (not a perspective: a map, a menu's flat camera)
    VrInput in = {0};
    VrPose head;
    if (vr_tracking_snapshot(&in)) head = in.head;
    else if (mode != 2 || !fake_head(&head)) return;
    float pm[16];
    uint64_t player;
    if (!player_matrix(pm, &player)) return;
    // ahead in the room (-z) is where the game's camera looks, along the ground: its yaw (the camera's -z axis in the
    // world is the view matrix's third row, negated)
    float fx = -view[2], fz = -view[10], fn = hypotf(fx, fz);
    if (!(fn > 1e-4f)) return;  // (looking straight down: the game's own view for that frame)
    static float eye_h = -1, eye_f, scale;
    if (eye_h < 0) {
        eye_f = eye_forward();
        eye_h = ENV("LARECOMP_VR_EYE_HEIGHT") ? (float)atof(ENV("LARECOMP_VR_EYE_HEIGHT")) : 1.45f;
        scale = ENV("LARECOMP_VR_WORLD_SCALE") ? (float)atof(ENV("LARECOMP_VR_WORLD_SCALE")) : 1.0f;
        if (!(scale > 0.01f)) scale = 1;
    }
    float s = -fx / fn, c = -fz / fn;  // (the yaw's sine and cosine: x' = c x + s z, z' = -s x + c z)
    float hr[3][3], r[3][3], hp[3];
    quat_rows(head.rot, hr);
    {   // the head in the recentred room: from the player's place, turned back by the yaw they faced then
        pthread_mutex_lock(&camera_lock);
        float f[2];
        if ((origin.want || !origin.have) && raw_forward(&head, f)) {
            memcpy(origin.pos, head.pos, sizeof origin.pos);
            origin.s = f[0], origin.c = -f[1];  // (the yaw that turns 0 0 -1 to f: x = -sin, z = -cos, and its inverse below)
            origin.have = true, origin.want = false;
            LOG("[vr] recentred: the player is at (%.2f %.2f %.2f) in the play space\n", head.pos[0], head.pos[1], head.pos[2]);
        }
        float os = origin.s, oc = origin.c, d[3] = {head.pos[0] - origin.pos[0], head.pos[1] - origin.pos[1], head.pos[2] - origin.pos[2]};
        pthread_mutex_unlock(&camera_lock);
        // (f = (-sin y0, -cos y0) -> origin.s = -sin y0, origin.c = cos y0; turning by -y0: x' = cos y0 x - sin y0 z,
        // z' = sin y0 x + cos y0 z)
        hp[0] = oc * d[0] + os * d[2], hp[1] = d[1], hp[2] = -os * d[0] + oc * d[2];
        float t[3][3];
        for (int j = 0; j < 3; j++) {
            t[0][j] = oc * hr[0][j] + os * hr[2][j];
            t[1][j] = hr[1][j];
            t[2][j] = -os * hr[0][j] + oc * hr[2][j];
        }
        memcpy(hr, t, sizeof hr);
    }
    for (int j = 0; j < 3; j++) {
        r[0][j] = c * hr[0][j] + s * hr[2][j];
        r[1][j] = hr[1][j];
        r[2][j] = -s * hr[0][j] + c * hr[2][j];
    }
    float hx = hp[0] * scale, hy = hp[1] * scale, hz = hp[2] * scale;
    // A continuous neck-to-eye offset in head space. Normalizing its horizontal
    // projection flips direction at the poles and the old threshold introduced
    // a positional jump while looking down toward the body.
    float eye[3] = {
        pm[12] - r[0][2] * eye_f + c * hx + s * hz,
        pm[13] + eye_h - r[1][2] * eye_f + hy,
        pm[14] - r[2][2] * eye_f - s * hx + c * hz
    };
    if (ENV("LARECOMP_LOG_VR_CAMERA_HEIGHT")) {
        static uint64_t last_log;
        uint64_t now = plat_now_ns();
        pthread_mutex_lock(&camera_lock);
        if (now - last_log >= 1000000000ull) {
            last_log = now;
            LOG("[vr-height] player %llx root (%.3f %.3f %.3f) eye (%.3f %.3f %.3f) height %.3f room-dy %.3f focus %.2f %.2f\n",
                (unsigned long long)player, pm[12], pm[13], pm[14], eye[0], eye[1], eye[2], eye[1] - pm[13], hp[1],
                in.trigger[0], in.trigger[1]);
        }
        pthread_mutex_unlock(&camera_lock);
    }
    // The same rigid transform as the rendered head, including recenter and stick yaw.
    // Publish targets only; the animation hook owns all skeleton writes.
    float raw[3][3], room[3][3];
    quat_rows(head.rot, raw);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        room[i][j] = 0;
        for (int k = 0; k < 3; k++) room[i][j] += r[i][k] * raw[j][k];
    }
    pthread_mutex_lock(&camera_lock);
    memcpy(world_of_room.rot, room, sizeof room);
    world_of_room.scale = scale, world_of_room.ns = plat_now_ns();
    pthread_mutex_unlock(&camera_lock);
    in.head = head;  // (a test's head that stays where it is: no hands with it)
    vr_body_targets(player, &in, eye, room, scale);
    // the view: the transpose of the head's turn in the world, and the eye brought to the origin
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) view[j * 4 + i] = r[j][i];
        view[i * 4 + 3] = 0;
        view[12 + i] = -(r[0][i] * eye[0] + r[1][i] * eye[1] + r[2][i] * eye[2]);
    }
    view[15] = 1;
    pthread_mutex_lock(&camera_lock);
    float th = fov_tan[0], tv = fov_tan[1];
    VrRenderView *rv = &ring[ring_seq++ % 8];
    *rv = (VrRenderView){.head = head, .tan_h = th, .tan_v = tv, .near = proj[14], .zeps = proj[10], .ns = plat_now_ns()};
    memcpy(rv->view, view, sizeof rv->view);
    pthread_mutex_unlock(&camera_lock);
    proj[0] = 1 / th, proj[5] = 1 / tv;
    p[7] = 2 * atanf(tv), p[8] = th / tv;
    p[16] = eye[0], p[17] = eye[1], p[18] = eye[2];
}

RT_ON_RESET(vr_camera_reset) { camera_logs = 0; }
