#include "../runtime/vr.h"
#include "../runtime/input_bind.h"
#include "../runtime/settings.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

Settings g_settings;
static uint64_t now = 1000000000ull;
uint64_t plat_now_ns(void) { return now; }
bool vk_scene_view_proj(float out[16]) { (void)out; return false; }
bool vr_render_view(int back, VrRenderView *out) { (void)back; (void)out; return false; }
bool vr_room_forward(const VrPose *head, float out[2]) { (void)head; (void)out; return false; }
void vr_recenter(void) {}
bool vr_world_rotation(float out[3][3], float *scale) { (void)out; (void)scale; return false; }

int main(void) {
    VrInput out, in = {.t = 1, .head = {.valid = true, .rot = {0,0,0,1}},
                      .buttons = VRB_A, .stick = {{0.8f,0},{0,0}}};
    memset(&out, 0xff, sizeof out);
    assert(!vr_tracking_snapshot(&out));
    assert(!out.head.valid && !out.buttons);
    in.head.pos[0] = 2;
    in.hand[VR_RIGHT].valid = true;
    in.hand[VR_RIGHT].rot[3] = 1;
    in.hand[VR_RIGHT].pos[1] = 1.4f;
    vr_input_update(&in);
    assert(vr_tracking_snapshot(&out));
    assert(out.head.pos[0] == 2 && out.hand[VR_RIGHT].pos[1] == 1.4f);
    in.head.pos[0] = 99; // The snapshot is owned by the input system, not the caller.
    assert(vr_tracking_snapshot(&out) && out.head.pos[0] == 2);
    int32_t axes[4] = {0};
    assert(vr_active());
    assert(vr_pad_state(axes) && axes[0] > 0);
    now += 500000000ull;
    assert(!vr_active());
    assert(!vr_tracking_snapshot(&out));
    assert(!out.head.valid && !out.hand[VR_RIGHT].valid && !out.buttons);
    memset(axes, 0, sizeof axes);
    assert(!vr_pad_state(axes) && !axes[0]);
    in.joint[VR_LEFT][VR_HAND_WRIST] = (VrPose){.valid = true, .rot = {0,0,0,2}, .pos = {1,2,3}};
    in.joint[VR_LEFT][2] = (VrPose){.valid = true, .rot = {0,0,0,1}, .pos = {NAN,0,0}};
    in.hand[VR_RIGHT].rot[0] = NAN;
    in.hand_tracked[VR_RIGHT] = true;
    vr_input_update(&in);
    assert(vr_tracking_snapshot(&out));
    assert(out.joint[VR_LEFT][VR_HAND_WRIST].valid && out.joint[VR_LEFT][VR_HAND_WRIST].rot[3] == 1);
    assert(!out.joint[VR_LEFT][2].valid && out.joint[VR_LEFT][2].pos[0] == 0);
    assert(!out.hand[VR_RIGHT].valid && !out.hand_tracked[VR_RIGHT]);
    vr_input_update(&(VrInput){0});
    assert(!vr_tracking_snapshot(&out));
    assert(!out.joint[VR_LEFT][VR_HAND_WRIST].valid && !vr_pad_state(axes));
    in.head.valid = false;
    vr_input_update(&in);
    assert(!vr_tracking_snapshot(&out));
    // Battle chords must reach the guest unchanged while ZL is held.
    const uint32_t face[] = {VRB_A, VRB_B, VRB_X, VRB_Y};
    const int bind[] = {BIND_A, BIND_B, BIND_X, BIND_Y};
    for (int i = 0; i < 4; i++) {
        in = (VrInput){.buttons = face[i], .trigger = {1, 0}};
        vr_input_update(&in);
        memset(axes, 0, sizeof axes);
        assert(vr_pad_state(axes) == ((1ull << BIND_ZL) | (1ull << bind[i])));
    }
    // The left grip deliberately shifts X/Y/B, even during focus: document the trap.
    in = (VrInput){.buttons = VRB_X | VRB_Y | VRB_B, .trigger = {1, 0}, .grip = {1, 0}};
    vr_input_update(&in);
    assert(vr_pad_state(axes) == ((1ull << BIND_ZL) | (1ull << BIND_L) |
                                (1ull << BIND_MINUS) | (1ull << BIND_PLUS)));
    puts("VR input snapshot/expiry and battle chord tests passed");
    return 0;
}
