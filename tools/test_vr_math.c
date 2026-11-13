#include "../runtime/vr_math.h"
#include <assert.h>
#include <stdio.h>

static void near(float a, float b) { assert(fabsf(a - b) < 0.0001f); }

int main(void) {
    // Camera looking down -Z, translated far from the origin. Translation must not steer throws.
    float vp[16] = {2,0,0,0, 0,3,0,0, 0,0,0.001f,-1, 1500,-20,0.1f,900};
    float v[3] = {1,2,-3}, out[3];
    assert(vr_world_velocity(vp,v,1,out));
    near(out[0],1); near(out[1],2); near(out[2],-3);
    vp[12] = -8000; vp[13] = 7000; vp[14] = 60; vp[15] = 200;
    assert(vr_world_velocity(vp,v,1,out));
    near(out[0],1); near(out[1],2); near(out[2],-3);
    // Camera yaw 90 degrees: right is +Z, forward is +X.
    vp[0] = 0; vp[8] = 2; vp[3] = 1; vp[11] = 0;
    assert(vr_world_velocity(vp,v,2,out));
    near(out[0],6); near(out[1],4); near(out[2],2);
    // Captured Z-A camera, pitched downward. Up remains world up.
    vp[0] = -1.237651f; vp[8] = 0.925556f;
    vp[3] = 0.593635f; vp[7] = -0.132163f; vp[11] = 0.793808f;
    float forward[3] = {0,0,-1};
    assert(vr_world_velocity(vp,forward,1,out));
    near(out[0],0.598888f); near(out[1],0); near(out[2],0.800832f);
    assert(!vr_world_velocity(vp,v,NAN,out));
    assert(!vr_world_velocity(vp,v,0,out));
    v[0] = INFINITY;
    assert(!vr_world_velocity(vp,v,1,out));
    v[0] = 1; vp[3] = vp[11] = 0;
    assert(!vr_world_velocity(vp,v,1,out));
    // The play space turned into the world: a quarter turn about y (ahead in the room, -z, is -x in the world), the
    // world twice the room's size. Up stays up and a throw's pitch is kept.
    {
        float rot[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}, up_ahead[3] = {0, 3, -4}, w[3];
        assert(vr_rot_apply(rot, up_ahead, 2, w));
        near(w[0], -8); near(w[1], 6); near(w[2], 0);
        float bad[3] = {NAN, 0, 0};
        assert(!vr_rot_apply(rot, bad, 1, w));
    }
    // The node's lerp taken back out: cur + (p - cur) k = lerped -> cur
    {
        float cur[3] = {5, -2, 7}, p[3] = {0, 1, 20}, lerped[3], back[3];
        float k = 0.25f;
        for (int i = 0; i < 3; i++) lerped[i] = cur[i] + (p[i] - cur[i]) * k;
        assert(vr_ball_unlerp(lerped, p, k, back));
        near(back[0], 5); near(back[1], -2); near(back[2], 7);
        assert(vr_ball_unlerp(p, p, 0, back));  // (no lerp: what it sets is the body's own)
        near(back[2], 20);
        assert(!vr_ball_unlerp(lerped, p, 1, back));  // (set outright: nothing of the body's left)
        assert(!vr_ball_unlerp(lerped, p, NAN, back));
        assert(!vr_ball_unlerp(lerped, p, -0.5f, back));
    }
    // The release's mean of the hand's samples
    {
        float s[3][3] = {{1, 2, 3}, {3, 2, 1}, {2, 2, 2}}, m[3];
        assert(vr_mean3((const float (*)[3])s, 3, m));
        near(m[0], 2); near(m[1], 2); near(m[2], 2);
        assert(!vr_mean3((const float (*)[3])s, 0, m));
    }
    puts("VR matrix/throw tests passed");
    return 0;
}
