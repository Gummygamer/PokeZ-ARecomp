// PCVR: the headset's controllers as the game's pad, and the throw found in the hand's motion (runtime/vr.h).
// runtime/nvn/vulkan/xr.c reads the controllers once per frame shown in the headset (vr_input_update, on the thread
// that presents); the game reads its pad from its own threads (input.c fill_state -> vr_pad_state): one lock for both.
//
// The pad (Touch-style controllers: a stick, two face buttons, a trigger and a grip per hand, Menu on the left):
//   sticks = the sticks (and their clicks), A B X Y = the same, Menu = +, left trigger = ZL, right grip = R,
//   right trigger = ZR (see the throw below). The left grip is a shift key, for what has no button of its own: with it
//   held the left stick is the d-pad, X is L, Y is - and B is + (controllers without a Menu button).
//
// The throw (LARECOMP_VR_THROW=hold, the default): the right trigger held is ZR held - the game's ball in hand, its
// aim on screen - and the ball goes when the hand throws: its speed went over LARECOMP_VR_THROW_SPEED (m/s, 2.0),
// away from the player, and has just passed its peak (where a real ball leaves the hand). ZR is let go there, with
// the trigger still held. Letting go of the trigger without a swing lets go of ZR too, as on a controller.
// =tap: for a game that throws on the press - the trigger does nothing, the swing is a short ZR press.
// =0: the trigger is ZR and nothing else. LARECOMP_VR_THROW_HAND=left: the other hand (the triggers swap).
// Each throw is kept (vr_last_throw: where the hand was, its velocity, the head) for a ball that flies the way it was
// thrown, which needs the game's own throw code and is not done here.
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "../hle.h"
#include "../input_bind.h"
#include "../platform.h"
#include "../settings.h"
#include "../vr.h"
#include "../vr_math.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t pad_b;         // the pad the controllers hold now: nn::hid button bits (BIND_*)
static int32_t pad_st[4];      // lx ly rx ry
static uint64_t updated_ns;    // (vr_active: the last vr_input_update)
static VrThrow last_throw;
static VrInput tracking;
static uint64_t throw_ns;      // (plat_now_ns at it: VrThrow's t is the headset's clock)
static struct { float amp, secs; bool set; } pulse[2];
static float rumble_amp;
static bool rumble_dirty[2];

bool vr_wanted(void) {
    static int want = -1;
    if (want < 0) {
        const char *e = getenv("LARECOMP_VR");
        want = e && *e ? atoi(e) != 0 : g_settings.vr;
    }
    return want;
}

bool vr_active(void) {
    pthread_mutex_lock(&lock);
    uint64_t at = updated_ns;
    pthread_mutex_unlock(&lock);
    return at && plat_now_ns() - at < 500000000ull;
}

// ---- the throw
enum { THROW_OFF, THROW_HOLD, THROW_TAP };
static int throw_mode(void) {
    const char *e = ENV("LARECOMP_VR_THROW");
    return !e || !strcmp(e, "hold") ? THROW_HOLD : !strcmp(e, "tap") ? THROW_TAP : atoi(e) ? THROW_HOLD : THROW_OFF;
}
static int throw_hand(void) {
    const char *e = ENV("LARECOMP_VR_THROW_HAND");
    return e && !strcmp(e, "left") ? VR_LEFT : VR_RIGHT;
}
static float throw_speed(void) {
    const char *e = ENV("LARECOMP_VR_THROW_SPEED");
    float v = e ? (float)atof(e) : 2.0f;
    return v > 0.2f ? v : 0.2f;
}

// The head's forward direction on the floor plane (0 0 -1 turned by its rotation, y dropped); false: looking straight
// up or down, or no head pose
static bool head_forward(const VrPose *h, float f[3]) {
    if (!h->valid) return false;
    float x = h->rot[0], y = h->rot[1], z = h->rot[2], w = h->rot[3];
    f[0] = -2 * (x * z + w * y), f[1] = 0, f[2] = -(1 - 2 * (x * x + y * y));
    float n = sqrtf(f[0] * f[0] + f[2] * f[2]);
    if (n < 0.2f) return false;
    f[0] /= n, f[2] /= n;
    return true;
}

// One frame of the throwing hand. Returns whether ZR is held for the game now.
static bool throw_step(const VrInput *in) {
    static bool held, thrown;
    static double t_hold, t_peak, t_tap;
    static float peak;
    static VrThrow at_peak;
    int mode = throw_mode(), h = throw_hand();
    float trig = in->trigger[h];
    if (mode == THROW_OFF) return trig > 0.5f;
    if (!in->head.valid || !in->hand[h].valid) {
        held = thrown = false;
        peak = 0;
        return mode == THROW_HOLD && trig > 0.5f;
    }
    bool was = held;
    held = held ? trig > 0.3f : trig > 0.6f;  // (hysteresis: a trigger resting half pressed doesn't flutter)
    if (held && !was) t_hold = in->t, peak = 0, thrown = false;
    const VrPose *p = &in->hand[h];
    float v = p->valid ? sqrtf(p->vel[0] * p->vel[0] + p->vel[1] * p->vel[1] + p->vel[2] * p->vel[2]) : 0;
    float f[3];
    // away from the player: the arm coming back for the swing is as fast as the swing itself
    if (v > 0 && head_forward(&in->head, f) && p->vel[0] * f[0] + p->vel[2] * f[2] < 0.3f * v) v = 0;
    float thr = throw_speed();
    bool fire = false;
    if (held && !thrown) {
        if (v > peak) {
            peak = v, t_peak = in->t;
            at_peak = (VrThrow){.t = in->t, .hand = h, .speed = v, .head = in->head};
            memcpy(at_peak.pos, p->pos, sizeof at_peak.pos);
            memcpy(at_peak.vel, p->vel, sizeof at_peak.vel);
        }
        if (peak >= thr && v < 0.8f * peak && in->t - t_hold > 0.12) fire = true;
        else if (peak < thr && v < 0.5f * peak) peak = v;  // (a slower move that ended: the next one starts from here)
    }
    // the trigger let go in the swing, before its speed fell: the same throw
    if (!held && was && !thrown && peak >= thr && in->t - t_peak < 0.15) fire = true;
    if (fire) {
        thrown = true;
        t_tap = in->t;
        at_peak.seq = last_throw.seq + 1;
        last_throw = at_peak;
        throw_ns = plat_now_ns();
        pulse[h].amp = 0.8f, pulse[h].secs = 0.08f, pulse[h].set = true;
        LOG("[vr] throw %llu: %.1f m/s (%.2f %.2f %.2f) from (%.2f %.2f %.2f)\n", (unsigned long long)at_peak.seq, at_peak.speed,
            at_peak.vel[0], at_peak.vel[1], at_peak.vel[2], at_peak.pos[0], at_peak.pos[1], at_peak.pos[2]);
    }
    if (mode == THROW_TAP) return thrown && in->t - t_tap < 0.15;
    return held && !thrown;
}

static int32_t axis(float v) {
    if (!isfinite(v)) return 0;
    float a = fabsf(v);
    if (a < 0.12f) return 0;  // (a resting stick)
    a = (a - 0.12f) / 0.88f;
    if (a > 1) a = 1;
    return (int32_t)((v < 0 ? -a : a) * 32767.0f);
}

static void clean_pose(VrPose *p) {
    float norm = 0;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(p->pos[i])) p->valid = false;
        if (!isfinite(p->vel[i])) p->vel[i] = 0;
    }
    for (int i = 0; i < 4; i++) norm += p->rot[i] * p->rot[i];
    if (!isfinite(norm) || norm < 1e-8f) p->valid = false;
    if (!p->valid) { *p = (VrPose){.rot = {0, 0, 0, 1}}; return; }
    float scale = 1 / sqrtf(norm);
    for (int i = 0; i < 4; i++) p->rot[i] *= scale;
}

void vr_input_update(const VrInput *input) {
    VrInput cleaned = *input;
    clean_pose(&cleaned.head);
    for (int h = 0; h < 2; h++) {
        clean_pose(&cleaned.hand[h]);
        for (int j = 0; j < VR_HAND_JOINT_COUNT; j++) clean_pose(&cleaned.joint[h][j]);
        if (!cleaned.hand[h].valid) cleaned.hand_tracked[h] = false;
    }
    const VrInput *in = &cleaned;
    uint64_t b = 0;
    int32_t st[4] = {axis(in->stick[VR_LEFT][0]), axis(in->stick[VR_LEFT][1]), axis(in->stick[VR_RIGHT][0]), axis(in->stick[VR_RIGHT][1])};
    {   // First person (vr_camera.c): the left stick pushes where the player looks, not where the game's own camera
        // points - the stick turned by the head's yaw in the room (ahead in the room is that camera's ahead)
        VrRenderView rv;
        float f[2];
        if (vr_render_view(0, &rv) && vr_room_forward(&in->head, f)) {
            float x = (float)st[0], y = (float)st[1];
            st[0] = (int32_t)(-x * f[1] + y * f[0]);
            st[1] = (int32_t)(-x * f[0] - y * f[1]);
        }
        // both sticks held in for half a second: here is where the player stands and faces (vr_recenter)
        static double t_both;
        static bool done;
        bool both = (in->buttons & VRB_LSTICK) && (in->buttons & VRB_RSTICK);
        if (!both) t_both = in->t, done = false;
        else if (!done && in->t - t_both > 0.5) vr_recenter(), done = true;
    }
    int th = throw_hand(), other = th ^ 1;
    bool shift = in->grip[VR_LEFT] > 0.6f;
    if (in->buttons & VRB_A) b |= 1ull << BIND_A;
    if (in->buttons & VRB_B) b |= 1ull << (shift ? BIND_PLUS : BIND_B);  // (+ for controllers without a Menu button)
    if (in->buttons & VRB_X) b |= 1ull << (shift ? BIND_L : BIND_X);
    if (in->buttons & VRB_Y) b |= 1ull << (shift ? BIND_MINUS : BIND_Y);
    if (in->buttons & VRB_MENU) b |= 1ull << BIND_PLUS;
    if (in->buttons & VRB_LSTICK) b |= 1ull << BIND_LSTICK;
    if (in->buttons & VRB_RSTICK) b |= 1ull << BIND_RSTICK;
    if (in->grip[VR_RIGHT] > 0.6f) b |= 1ull << BIND_R;
    if (in->trigger[other] > 0.5f) b |= 1ull << BIND_ZL;
    if (shift) {  // the left stick as the d-pad
        float x = in->stick[VR_LEFT][0], y = in->stick[VR_LEFT][1];
        if (x < -0.5f) b |= 1ull << BIND_LEFT;
        if (x > 0.5f) b |= 1ull << BIND_RIGHT;
        if (y > 0.5f) b |= 1ull << BIND_UP;
        if (y < -0.5f) b |= 1ull << BIND_DOWN;
        st[0] = st[1] = 0;
    }
    pthread_mutex_lock(&lock);
    tracking = *in;
    if (throw_step(in)) b |= 1ull << BIND_ZR;
    pad_b = b;
    memcpy(pad_st, st, sizeof pad_st);
    updated_ns = plat_now_ns();
    pthread_mutex_unlock(&lock);
}

bool vr_tracking_snapshot(VrInput *out) {
    pthread_mutex_lock(&lock);
    bool live = updated_ns && plat_now_ns() - updated_ns < 500000000ull;
    *out = live ? tracking : (VrInput){0};
    pthread_mutex_unlock(&lock);
    return live && out->head.valid;
}

uint64_t vr_pad_state(int32_t st[4]) {
    pthread_mutex_lock(&lock);
    bool live = updated_ns && plat_now_ns() - updated_ns < 500000000ull;  // (the headset taken off: nothing stays held)
    uint64_t b = live ? pad_b : 0;
    if (live) for (int i = 0; i < 4; i++) st[i] += pad_st[i];
    pthread_mutex_unlock(&lock);
    return b;
}

VrThrow vr_last_throw(void) {
    pthread_mutex_lock(&lock);
    VrThrow t = last_throw;
    pthread_mutex_unlock(&lock);
    return t;
}

// ---- the thrown ball (Pokémon Z-A; the generator's patch: tools/za_patches.py ZA_BALL)
// The game is about to give a ball's body its velocities (the action node SetBallVelocity): lin and ang are guest
// addresses of a vec4 each, the world's x y z (metres a second: to be confirmed in play, LARECOMP_VR_BALL_SCALE
// multiplies the hand's). Within a second of a throw found in the hand's motion, the first of these calls gets the
// hand's velocity instead of the game's: the play space's axes laid on the scene camera's - ahead of the player is
// where the camera looks along the ground, right is its right, up is up (the camera from the last frame drawn,
// vulkan/exec.c vk_scene_view_proj). Later calls of the same throw are the game's own (a node that runs every frame
// steers the ball back by its LerpRate). LARECOMP_VR_BALL=0: never; =all: every call in that second.
// LARECOMP_LOG_BALL=1: each call's velocities in the log, VR or not (which throws go through this node, and how fast).
// Called on a game thread, any of them.
void rt_za_ball_velocity(uint64_t body, uint64_t lin, uint64_t ang) {
    float *l = (float *)(uintptr_t)lin, *a = (float *)(uintptr_t)ang;
    if (ENV("LARECOMP_LOG_BALL")) {
        static _Atomic int n;
        if (n++ < 2000)
            LOG("[ball] body %llx linear (%.2f %.2f %.2f) angular (%.2f %.2f %.2f)\n", (unsigned long long)body, l[0], l[1], l[2],
                a[0], a[1], a[2]);
    }
    const char *mode = ENV("LARECOMP_VR_BALL");
    if (mode && !strcmp(mode, "0")) return;
    static uint64_t used;  // (the throw whose ball already has the hand's velocity)
    pthread_mutex_lock(&lock);
    VrThrow t = last_throw;
    bool live = updated_ns && plat_now_ns() - updated_ns < 500000000ull;
    bool take = live && t.seq && plat_now_ns() - throw_ns < 1000000000ull && (t.seq != used || (mode && !strcmp(mode, "all")));
    pthread_mutex_unlock(&lock);
    if (!take) return;
    float vp[16];
#ifndef __APPLE__
    extern bool vk_scene_view_proj(float out[16]);
    if (!vk_scene_view_proj(vp)) return;
#else
    return;  // (no VR on the Metal backend)
#endif
    const char *se = ENV("LARECOMP_VR_BALL_SCALE");
    float k = se ? (float)atof(se) : 1.0f;
    float w[3];
    if (!vr_world_velocity(vp, t.vel, k, w)) return;
    // Claim only after conversion succeeds; two game threads cannot consume the same throw.
    pthread_mutex_lock(&lock);
    bool all = mode && !strcmp(mode, "all");
    if (used == t.seq && !all) { pthread_mutex_unlock(&lock); return; }
    used = t.seq;
    pthread_mutex_unlock(&lock);
    LOG("[vr] ball of throw %llu: the game's velocity (%.2f %.2f %.2f) -> the hand's (%.2f %.2f %.2f)\n", (unsigned long long)t.seq,
        l[0], l[1], l[2], w[0], w[1], w[2]);
    l[0] = w[0], l[1] = w[1], l[2] = w[2];
}

// ---- vibration
void vr_rumble(uint16_t lo, uint16_t hi) {
    float a = (lo > hi ? lo : hi) / 65535.0f;
    pthread_mutex_lock(&lock);
    if (a != rumble_amp) rumble_amp = a, rumble_dirty[0] = rumble_dirty[1] = true;
    pthread_mutex_unlock(&lock);
}

void vr_haptic_pulse(int hand, float amplitude, float seconds) {
    if (hand < 0 || hand > 1) return;
    pthread_mutex_lock(&lock);
    pulse[hand].amp = amplitude, pulse[hand].secs = seconds, pulse[hand].set = true;
    pthread_mutex_unlock(&lock);
}

// The game's rumble has no length (a value until the next one): given 0.1 s at each frame while it lasts, so a stalled
// game can't leave a controller buzzing
bool vr_haptic_take(int hand, float *amplitude, float *seconds) {
    bool have = false;
    pthread_mutex_lock(&lock);
    if (pulse[hand].set) {
        *amplitude = pulse[hand].amp, *seconds = pulse[hand].secs, pulse[hand].set = false, have = true;
    } else if (rumble_amp > 0 || rumble_dirty[hand]) {
        *amplitude = rumble_amp, *seconds = 0.1f, rumble_dirty[hand] = false, have = true;
    }
    pthread_mutex_unlock(&lock);
    return have;
}
