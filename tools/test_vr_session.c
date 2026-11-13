#include "../runtime/vr.h"
#include <assert.h>
#include <stdio.h>

bool g_game_is_la;
bool vr_wanted(void) { return true; }
bool vr_tracking_snapshot(VrInput *out) { (void)out; return false; }
uint64_t plat_now_ns(void) { return 1000000000ull; }
uint64_t renderer_present_count(void) { return 0; }

int main(void) {
    double shape = 1, lines = 1;
    assert(!vr_render_shape(&shape, &lines));
    vr_session_made();
    assert(vr_render_shape(&shape, &lines));
    assert(shape > 0 && lines > 0);
    vr_session_lost();
    assert(!vr_render_shape(&shape, &lines));
    vr_session_lost();
    assert(!vr_render_shape(&shape, &lines));
    vr_session_made();
    assert(vr_render_shape(&shape, &lines));
    puts("VR session fallback tests passed");
    return 0;
}
