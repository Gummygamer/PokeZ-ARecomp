// Player skeleton integration, separate from the OpenXR-facing input interface.
#pragma once
#include "vr.h"

// Publish from the camera; animation consumes a copy on its own thread. room is a
// row-major rotation from OpenXR LOCAL to game world, eye is the rendered eye.
void vr_body_targets(uint64_t player, const VrInput *input, const float eye[3],
                     const float room[3][3], float scale);
