// The OpenXR session (xr.c; runtime/vr.h has the rest of PCVR): what device.c asks of it, in the order it does.
// Every function is a no-op (false / 0 / NULL) in a build without OpenXR and while VR is off.
#pragma once
#include "vk.h"

// Before the Vulkan instance: the OpenXR instance and its headset (false: VR stays off for this run, the reason logged)
bool xr_startup(void);
// The Vulkan extensions the OpenXR runtime needs enabled, added to out[n..max) unless already there; returns the new n
uint32_t xr_instance_extensions(const char **out, uint32_t n, uint32_t max);
uint32_t xr_device_extensions(const char **out, uint32_t n, uint32_t max);
// The GPU the headset is on (the device must be made on it), or NULL
VkPhysicalDevice xr_physical_device(void);
// After the device: the session, its spaces, the controllers' actions
void xr_session_start(void);
bool xr_on(void);  // a session exists (present_mode: the window must not wait for the monitor)

// A frame for the headset, at the end of a window frame (device.c vk_frame_end, the thread that presents). Waits for
// the recorder and for the headset's frame timing. Returns what the headset is to be given:
//   0: nothing now (no session, the headset off).
//   1: the screen in front of the player - *screen: the image the window's picture (window_format, extent) is to be
//      copied into, in VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL and to be left in it; *blit: by vkCmdBlitImage instead
//      of vkCmdCopyImage (the headset has no format of the window's byte order).
//   2: first person (hle/vr_camera.c: the game's camera is the head's) - *eyes: an image per eye to draw (exec.c
//      vk_xr_eye), to be put into and left in COLOR_ATTACHMENT_OPTIMAL.
#include "../../vr.h"
typedef struct {
    VkImage image[2];
    VkImageView view[2];
    VkFormat format;   // (an sRGB one)
    uint32_t w, h;
    float fov[2][4];   // each eye's field of view, tangents: left, right, up, down (left and down negative)
    float eye[2][3];   // where each eye is from the middle between them, metres (the head's axes)
    VrRenderView rv;   // what the game drew this frame's picture with
} XrEyes;
int xr_frame_begin(VkFormat window_format, VkExtent2D extent, VkImage *screen, bool *blit, XrEyes *eyes);
// After the submission that holds the copy / the eyes' pictures (queued behind it on the recorder)
void xr_frame_end(void);
