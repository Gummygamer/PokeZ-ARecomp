// PCVR (OpenXR): the headset's side of the runtime. runtime/nvn/vulkan/xr.c holds the OpenXR session (its frames,
// the screen in the headset, the controllers' poses and buttons); runtime/hle/vr_input.c turns what the controllers
// do into the game's pad (runtime/hle/input.c fill_state) and finds the throws.
// Off unless larecomp.ini has "vr = 1" (F1 > Video > VR headset) or LARECOMP_VR=1; a build without the OpenXR loader
// (Makefile VR) has the same functions and never starts a session.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// A pose in the play space (OpenXR's LOCAL space: metres, +x right, +y up, -z ahead of where the headset was at the
// start or at the last recentre), with its velocity (m/s) and its angular velocity (rad/s, about the play space's
// axes) there.
typedef struct { float pos[3], rot[4] /* x y z w */, vel[3], avel[3]; bool valid; } VrPose;

enum { VR_LEFT = 0, VR_RIGHT = 1 };
// XR_EXT_hand_tracking joint order: palm, wrist, then thumb/index/middle/ring/little.
enum { VR_HAND_JOINT_COUNT = 26, VR_HAND_WRIST = 1 };
enum { VRB_A = 1 << 0, VRB_B = 1 << 1, VRB_X = 1 << 2, VRB_Y = 1 << 3, VRB_MENU = 1 << 4, VRB_LSTICK = 1 << 5, VRB_RSTICK = 1 << 6 };

// The controllers at one XR frame (xr.c, once per frame shown in the headset)
typedef struct {
    double t;              // seconds (the frame's display time)
    VrPose head, hand[2];  // (the hands: their grip poses)
    float trigger[2], grip[2], stick[2][2];  // 0-1; sticks -1..1, x right / y up
    uint32_t buttons;      // VRB_*
    VrPose joint[2][VR_HAND_JOINT_COUNT]; // LOCAL-space joints; each has its own validity
    bool hand_tracked[2];  // hand[] uses the tracked wrist instead of a controller grip
    float pinch[2], fist[2];  // (bare hands, xr.c) thumb on index finger, a closed hand: 0-1, the trigger and grip then
} VrInput;

// A throw found in the hand's motion (vr_input.c): what the hand did at the moment the ball left it. seq counts them.
// vel / avel: the release's velocity and spin in the play space, each the mean of the hand's samples around the peak
// of the swing (one frame's velocity is noisy: the ball went a few degrees off where the arm went).
typedef struct { uint64_t seq; double t; int hand; float pos[3], vel[3], avel[3], speed; VrPose head; } VrThrow;

bool vr_wanted(void);  // asked for (the setting / LARECOMP_VR): decided once, before the GPU device is made
bool vr_active(void);  // a session is running and its controllers were read in the last half second

// The first-person camera (hle/vr_camera.c: Pokémon Z-A 2.0.0): what one frame of the game was drawn with - the head's
// pose in the play space and half the field of view (tangents: across, up; the same to both sides).
// near / zeps: the projection's depth terms (a point `dist` ahead of the camera has the depth near / dist - zeps).
// view: the view matrix the game was given (column-major) - what tells this frame from the others (vr_render_view_of).
typedef struct { VrPose head; float tan_h, tan_v, near, zeps, view[16]; uint64_t ns; } VrRenderView;
bool vr_render_view_of(const float view[16], VrRenderView *out);
// (nvn/output_scale.c) first person is on for this run: the picture's shape against 16:9, and its height in 1080s
bool vr_render_shape(double *shape, double *lines);
// First person: the player character's own head is cut out of the picture its camera draws - the radius around the
// head's middle, cm, in the low byte, and how far behind the camera that middle is, cm, in the high one (0: not at
// all). The same all through a run: it is part of every vertex shader's variant (nvn/maxwell/shader.h ShaderKey.vr_cut).
uint16_t vr_head_cut(void);
void vr_session_made(void);  // (xr.c) the headset's session exists: first person is on for this run
void vr_session_lost(void);  // Session ended: restore ordinary camera and render shape.
// (xr.c -> vr_camera.c) the field of view the game is to draw with: wide enough for both of the headset's eyes
void vr_set_fov(float tan_h, float tan_v);
// (vr_camera.c -> xr.c) the view of the frame drawn `back` frames before the newest (0-7); false: the game's camera is
// not the head's now (off, a menu, another game version) - the picture goes on the screen in front of the player
bool vr_render_view(int back, VrRenderView *out);
// Where the player stands and faces in the play space is taken anew (the next camera: there is the character's eyes,
// that way is the game camera's ahead). Done by itself at the first frame; vr_input.c asks when both sticks are held in.
void vr_recenter(void);
// The head's ahead along the floor, in that recentred room (ahead = 0, -1); false: looking straight up or down
bool vr_room_forward(const VrPose *head, float f[2]);
// First person: the turn from the play space to the game's world that the last camera was made with (row-major: world =
// rot * play space; the recentring's yaw and the game camera's, so it does not change with the head) and the world's
// metres per play-space metre (LARECOMP_VR_WORLD_SCALE). What turns a hand's velocity into a ball's. False: no first
// person camera in the last half second (theater, a menu).
bool vr_world_rotation(float rot[3][3], float *scale);

// (xr.c -> vr_input.c) the controllers' state of this frame
void vr_input_update(const VrInput *in);
// A coherent head/hand sample, or false after tracking has gone stale.
bool vr_tracking_snapshot(VrInput *out);
// (input.c fill_state) what the controllers hold as the game's pad: nn::hid button bits, and lx ly rx ry (+-32767)
// added to st. Nothing while no session runs.
uint64_t vr_pad_state(int32_t st[4]);
// The last throw (seq 0: none yet)
VrThrow vr_last_throw(void);

// Vibration: the game's rumble (input.c input_rumble_poll, 0-65535 per motor) and one-off pulses (vr_input.c). xr.c
// reads them at its frames (vr_haptic_take: the amplitude 0-1 and length for a hand, false: nothing new).
void vr_rumble(uint16_t lo, uint16_t hi);
void vr_haptic_pulse(int hand, float amplitude, float seconds);
bool vr_haptic_take(int hand, float *amplitude, float *seconds);

#ifdef __cplusplus
}
#endif

