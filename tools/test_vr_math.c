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
    puts("VR matrix/throw tests passed");
    return 0;
}
