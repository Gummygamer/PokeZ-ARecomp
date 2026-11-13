#include "../runtime/vr_ik.h"
#include <assert.h>
#include <stdio.h>

static void near(float a, float b) { assert(fabsf(a-b) < 0.0001f); }
static float distance(const float a[3], const float b[3]) {
    return hypotf(hypotf(a[0]-b[0],a[1]-b[1]),a[2]-b[2]);
}
static void lengths(const float u[3], const float e[3], const float w[3], float a, float b) {
    near(distance(u,e),a); near(distance(e,w),b);
}
int main(void) {
    float u[3]={3,4,5}, e[3]={4,4,5}, w[3]={4,5,5};
    float target[3]={3,5,6}, pole[3]={5,4,5}, oe[3], ow[3];
    assert(vr_ik_solve_two_bone(u,e,w,target,pole,oe,ow));
    lengths(u,oe,ow,1,1);
    for (int i=0;i<3;++i) near(ow[i],target[i]);
    assert(oe[0]>u[0]);
    // Beyond reach: preserve direction and lengths at full extension.
    target[0]=3; target[1]=10; target[2]=13;
    assert(vr_ik_solve_two_bone(u,e,w,target,pole,oe,ow));
    lengths(u,oe,ow,1,1); near(ow[0],3); near(ow[1],5.2f); near(ow[2],6.6f);
    // Coincident target and shoulder, equal length fold.
    assert(vr_ik_solve_two_bone(u,e,w,u,pole,oe,ow));
    lengths(u,oe,ow,1,1); near(distance(u,ow),0);
    // Unequal lengths cannot fold closer than their difference.
    w[1]=6;
    assert(vr_ik_solve_two_bone(u,e,w,u,pole,oe,ow));
    lengths(u,oe,ow,1,2); near(distance(u,ow),1);
    // Pole and original arm both collinear: choose a finite fallback plane.
    float zero[3]={0,0,0}, straight_e[3]={1,0,0}, straight_w[3]={2,0,0}, t[3]={1,0,0};
    assert(vr_ik_solve_two_bone(zero,straight_e,straight_w,t,t,oe,ow));
    lengths(zero,oe,ow,1,1); near(ow[0],1);
    // A range of target directions and distances preserves segment lengths.
    for (int i=0;i<200;++i) {
        t[0]=3*sinf(i*0.37f); t[1]=2*cosf(i*0.71f); t[2]=sinf(i*0.19f);
        assert(vr_ik_solve_two_bone(zero,straight_e,straight_w,t,pole,oe,ow));
        lengths(zero,oe,ow,1,1);
    }
    oe[0]=123; ow[0]=456;
    assert(!vr_ik_solve_two_bone(u,u,w,target,pole,oe,ow));
    near(oe[0],123); near(ow[0],456);
    target[0]=NAN;
    assert(!vr_ik_solve_two_bone(u,e,w,target,pole,oe,ow));
    target[0]=INFINITY;
    assert(!vr_ik_solve_two_bone(u,e,w,target,pole,oe,ow));
    float x[3]={1,0,0}, y[3]={0,1,0}, minus_x[3]={-1,0,0}, q[4], inv[4], prod[4], v[3];
    assert(vr_ik_quat_shortest_arc(x,y,q));
    vr_ik_quat_rotate(q,x,v); near(v[0],0); near(v[1],1); near(v[2],0);
    vr_ik_quat_conjugate(q,inv); vr_ik_quat_mul(inv,q,prod);
    near(prod[0],0); near(prod[1],0); near(prod[2],0); near(prod[3],1);
    vr_ik_quat_mul(q,q,q); vr_ik_quat_rotate(q,x,v); near(v[0],-1); near(v[1],0);
    assert(vr_ik_quat_shortest_arc(x,minus_x,q));
    vr_ik_quat_rotate(q,x,v); near(v[0],-1); near(v[1],0); near(v[2],0);
    assert(vr_ik_quat_shortest_arc(x,x,q)); near(q[3],1);
    assert(!vr_ik_quat_shortest_arc(zero,x,q));
    float bad[4]={NAN,0,0,1}, empty[4]={0,0,0,0}, large[4]={FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
    assert(!vr_ik_quat_normalize(bad,q)); assert(!vr_ik_quat_normalize(empty,q));
    assert(vr_ik_quat_normalize(large,large)); for(int i=0;i<4;++i) near(large[i],0.5f);
    puts("VR arm IK and quaternion tests passed");
    return 0;
}
