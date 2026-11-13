// PCVR: the OpenXR session (runtime/vr.h, xr.h). What the headset shows for now is the window's picture - the start
// screen, the game, the F1 menu over it - on a screen floating in front of the player: a quad layer, which the
// headset's own compositor places for each eye at its own refresh rate, so the screen stays still in space whatever
// the game's frame rate (30 / 60 fps, a loading freeze). The game's world around the player (a view per eye from the
// game's camera, moved with the head) is not done yet: exec.c knows the scene camera (mv_note_draw, the lighting pass)
// and would draw the frame once per eye from there.
//
// Vulkan: XR_KHR_vulkan_enable - the device stays device.c's, made on the GPU the headset names with the extensions
// its runtime asks for. OpenXR uses the queue inside xrBeginFrame / xrEndFrame / xrAcquireSwapchainImage /
// xrReleaseSwapchainImage, and the queue is the recorder's (record.c): the frame begins after rec_drain (the recorder
// idle), and ends in a packet of the recorder's (rec_call), behind the submission that copied the picture.
//
// A frame of the headset's is made per window frame (device.c vk_frame_end): none while the window is minimised.
// LARECOMP_VR_SCREEN=<width>,<distance>,<height>: the screen's width, how far ahead and how far above eye level it
// is, metres (3.2,2.5,0).
#include <SDL3/SDL.h>
#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "../../rt.h"
#include "../../vr.h"
#include "vk.h"
#include "record.h"
#include "xr.h"

#ifdef LARECOMP_VR
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

static struct {
    XrInstance inst;
    XrSystemId sys;
    XrSession session;
    XrSpace local, view, hand[2];
    bool running, lost;
    bool hand_tracking;
    XrHandTrackerEXT tracker[2];
    char inst_ext[2048], dev_ext[2048];
    // the screen's swapchain: the window's size, an sRGB format of the window's byte order
    XrSwapchain sc;
    uint32_t sc_w, sc_h, sc_n;
    int64_t sc_fmt;
    bool sc_blit;
    VkFormat sc_for;  // (the window format it was made for)
    VkImage sc_img[16];
    XrActionSet set;
    XrAction pose, trigger, grip, stick, click, face1, face2, menu, haptic;
    XrPath hand_path[2];
    // first person: an image per eye, the size the headset asks for, and where its eyes are (xrLocateViews)
    XrSwapchain esc[2];
    uint32_t e_w, e_h, e_n[2];
    int64_t e_fmt;
    bool e_failed;
    VkImage e_img[2][16];
    VkImageView e_view[2][16];
    XrPosef eye_pose[2];
    XrFovf eye_fov[2];
    bool have_views;
    int mode;          // (this frame: 1 the screen, 2 the eyes)
    VrRenderView rv;
    // the frame open now (between xr_frame_begin and end_run)
    XrTime time;
    atomic_bool begun;
    bool image;
} xr;
static struct { float pos[3]; XrTime t; bool have; } prev_pose[3];

// (an extension's functions are not the loader's exports: from the instance, by name)
#define XR_KHR_FUNCS(X) X(xrGetVulkanInstanceExtensionsKHR) X(xrGetVulkanDeviceExtensionsKHR) X(xrGetVulkanGraphicsDeviceKHR)     X(xrGetVulkanGraphicsRequirementsKHR)
#define XR_DEFINE_FUNC(f) static PFN_##f p_##f;
XR_KHR_FUNCS(XR_DEFINE_FUNC)
XR_DEFINE_FUNC(xrCreateHandTrackerEXT)
XR_DEFINE_FUNC(xrDestroyHandTrackerEXT)
XR_DEFINE_FUNC(xrLocateHandJointsEXT)
#undef XR_DEFINE_FUNC

static const char *xr_err(XrResult r) {
    static _Thread_local char s[XR_MAX_RESULT_STRING_SIZE];
    if (!xr.inst || XR_FAILED(xrResultToString(xr.inst, r, s))) snprintf(s, sizeof s, "%d", (int)r);
    return s;
}
#define XR_TRY(call, ret) do { XrResult r_ = (call); if (XR_FAILED(r_)) { LOG("[vr] %s: %s\n", #call, xr_err(r_)); return ret; } } while (0)

// "a b c" -> its words added to out (the string is cut in place and kept: out points into it)
static uint32_t add_words(char *s, const char **out, uint32_t n, uint32_t max) {
    for (char *w = strtok(s, " "); w; w = strtok(NULL, " ")) {
        bool have = false;
        for (uint32_t i = 0; i < n; i++) have |= !strcmp(out[i], w);
        if (!have && n < max) out[n++] = w;
    }
    return n;
}

bool xr_startup(void) {
    if (!vr_wanted()) return false;
    const char *exts[2] = {XR_KHR_VULKAN_ENABLE_EXTENSION_NAME};
    uint32_t ext_count = 0;
    if (XR_SUCCEEDED(xrEnumerateInstanceExtensionProperties(NULL, 0, &ext_count, NULL)) && ext_count) {
        XrExtensionProperties *props = calloc(ext_count, sizeof *props);
        if (props) {
            for (uint32_t i = 0; i < ext_count; i++) props[i].type = XR_TYPE_EXTENSION_PROPERTIES;
            if (XR_SUCCEEDED(xrEnumerateInstanceExtensionProperties(NULL, ext_count, &ext_count, props)))
                for (uint32_t i = 0; i < ext_count; i++)
                    if (!strcmp(props[i].extensionName, XR_EXT_HAND_TRACKING_EXTENSION_NAME)) xr.hand_tracking = true;
            free(props);
        }
    }
    if (xr.hand_tracking) exts[1] = XR_EXT_HAND_TRACKING_EXTENSION_NAME;
    XrInstanceCreateInfo ci = {.type = XR_TYPE_INSTANCE_CREATE_INFO,
                               .applicationInfo = {.applicationName = "ZA Recomp", .applicationVersion = 1, .engineName = "LARecomp",
                                                   .apiVersion = XR_MAKE_VERSION(1, 0, 0)},
                               .enabledExtensionCount = xr.hand_tracking ? 2 : 1, .enabledExtensionNames = exts};
    XrResult r = xrCreateInstance(&ci, &xr.inst);
    if (XR_FAILED(r)) {
        xr.inst = XR_NULL_HANDLE;
        LOG("[vr] no OpenXR runtime with Vulkan (%d): is the headset's PC app (Meta Link, SteamVR...) installed and set as "
            "the OpenXR runtime? Playing on the window only\n", (int)r);
        return false;
    }
    XrInstanceProperties ip = {.type = XR_TYPE_INSTANCE_PROPERTIES};
    if (XR_SUCCEEDED(xrGetInstanceProperties(xr.inst, &ip)))
        LOG("[vr] OpenXR runtime: %s %u.%u.%u\n", ip.runtimeName, XR_VERSION_MAJOR(ip.runtimeVersion),
            XR_VERSION_MINOR(ip.runtimeVersion), XR_VERSION_PATCH(ip.runtimeVersion));
    XrSystemGetInfo si = {.type = XR_TYPE_SYSTEM_GET_INFO, .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    r = xrGetSystem(xr.inst, &si, &xr.sys);
    if (XR_FAILED(r)) {
        LOG("[vr] no headset (%s): connect it (Quest: start Link / Air Link first) and start again. Playing on the window only\n",
            xr_err(r));
        xrDestroyInstance(xr.inst);
        xr.inst = XR_NULL_HANDLE;
        return false;
    }
    if (xr.hand_tracking) {
        XrSystemHandTrackingPropertiesEXT hp = {.type = XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
        XrSystemProperties sp = {.type = XR_TYPE_SYSTEM_PROPERTIES, .next = &hp};
        xr.hand_tracking = XR_SUCCEEDED(xrGetSystemProperties(xr.inst, xr.sys, &sp)) && hp.supportsHandTracking;
#define LOAD_HAND(f) if (xr.hand_tracking) xr.hand_tracking = XR_SUCCEEDED(xrGetInstanceProcAddr(xr.inst, #f, (PFN_xrVoidFunction *)&p_##f)) && p_##f;
        LOAD_HAND(xrCreateHandTrackerEXT)
        LOAD_HAND(xrDestroyHandTrackerEXT)
        LOAD_HAND(xrLocateHandJointsEXT)
#undef LOAD_HAND
    }
    XrGraphicsRequirementsVulkanKHR gr = {.type = XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR};
    uint32_t n = 0;
    r = XR_SUCCESS;
#define XR_LOAD_FUNC(f) if (XR_SUCCEEDED(r)) r = xrGetInstanceProcAddr(xr.inst, #f, (PFN_xrVoidFunction *)&p_##f);
    XR_KHR_FUNCS(XR_LOAD_FUNC)
#undef XR_LOAD_FUNC
    if (XR_FAILED(r) || XR_FAILED(r = p_xrGetVulkanGraphicsRequirementsKHR(xr.inst, xr.sys, &gr)) ||
        XR_FAILED(r = p_xrGetVulkanInstanceExtensionsKHR(xr.inst, xr.sys, sizeof xr.inst_ext, &n, xr.inst_ext))) {
        LOG("[vr] the OpenXR runtime's Vulkan requirements: %s. Playing on the window only\n", xr_err(r));
        xrDestroyInstance(xr.inst);
        xr.inst = XR_NULL_HANDLE;
        return false;
    }
    return true;
}

uint32_t xr_instance_extensions(const char **out, uint32_t n, uint32_t max) {
    return xr.inst ? add_words(xr.inst_ext, out, n, max) : n;
}

VkPhysicalDevice xr_physical_device(void) {
    VkPhysicalDevice pd = VK_NULL_HANDLE;
    if (!xr.inst) return pd;
    XR_TRY(p_xrGetVulkanGraphicsDeviceKHR(xr.inst, xr.sys, g_vk.inst, &pd), VK_NULL_HANDLE);
    return pd;
}

uint32_t xr_device_extensions(const char **out, uint32_t n, uint32_t max) {
    if (!xr.inst) return n;
    uint32_t len = 0;
    XR_TRY(p_xrGetVulkanDeviceExtensionsKHR(xr.inst, xr.sys, sizeof xr.dev_ext, &len, xr.dev_ext), n);
    return add_words(xr.dev_ext, out, n, max);
}

// ---- the controllers' actions
static XrPath path(const char *s) {
    XrPath p = XR_NULL_PATH;
    xrStringToPath(xr.inst, s, &p);
    return p;
}
static XrAction action(const char *name, XrActionType type, bool hands) {
    XrActionCreateInfo ai = {.type = XR_TYPE_ACTION_CREATE_INFO, .actionType = type, .countSubactionPaths = hands ? 2 : 0,
                             .subactionPaths = hands ? xr.hand_path : NULL};
    snprintf(ai.actionName, sizeof ai.actionName, "%s", name);
    snprintf(ai.localizedActionName, sizeof ai.localizedActionName, "%s", name);
    XrAction a = XR_NULL_HANDLE;
    XrResult r = xrCreateAction(xr.set, &ai, &a);
    if (XR_FAILED(r)) LOG("[vr] action %s: %s\n", name, xr_err(r));
    return a;
}
typedef struct { XrAction *a; const char *p; } Bind;
static void suggest(const char *profile, const Bind *b, uint32_t n) {
    XrActionSuggestedBinding sb[32];
    uint32_t k = 0;
    for (uint32_t i = 0; i < n && k < 32; i++)
        if (*b[i].a) sb[k++] = (XrActionSuggestedBinding){*b[i].a, path(b[i].p)};
    XrInteractionProfileSuggestedBinding s = {.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING, .interactionProfile = path(profile),
                                              .countSuggestedBindings = k, .suggestedBindings = sb};
    XrResult r = xrSuggestInteractionProfileBindings(xr.inst, &s);
    if (XR_FAILED(r)) LOG("[vr] bindings for %s: %s\n", profile, xr_err(r));
}

static bool actions_create(void) {
    xr.hand_path[VR_LEFT] = path("/user/hand/left");
    xr.hand_path[VR_RIGHT] = path("/user/hand/right");
    XrActionSetCreateInfo si = {.type = XR_TYPE_ACTION_SET_CREATE_INFO, .actionSetName = "game", .localizedActionSetName = "Game"};
    XR_TRY(xrCreateActionSet(xr.inst, &si, &xr.set), false);
    xr.pose = action("hand_pose", XR_ACTION_TYPE_POSE_INPUT, true);
    xr.trigger = action("trigger", XR_ACTION_TYPE_FLOAT_INPUT, true);
    xr.grip = action("grip", XR_ACTION_TYPE_FLOAT_INPUT, true);
    xr.stick = action("stick", XR_ACTION_TYPE_VECTOR2F_INPUT, true);
    xr.click = action("stick_click", XR_ACTION_TYPE_BOOLEAN_INPUT, true);
    xr.face1 = action("lower_button", XR_ACTION_TYPE_BOOLEAN_INPUT, true);  // (A / X)
    xr.face2 = action("upper_button", XR_ACTION_TYPE_BOOLEAN_INPUT, true);  // (B / Y)
    xr.menu = action("menu", XR_ACTION_TYPE_BOOLEAN_INPUT, false);
    xr.haptic = action("vibration", XR_ACTION_TYPE_VIBRATION_OUTPUT, true);
#define BOTH(a, p) {&xr.a, "/user/hand/left" p}, {&xr.a, "/user/hand/right" p}
    const Bind touch[] = {BOTH(pose, "/input/grip/pose"), BOTH(trigger, "/input/trigger/value"), BOTH(grip, "/input/squeeze/value"),
                          BOTH(stick, "/input/thumbstick"), BOTH(click, "/input/thumbstick/click"), BOTH(haptic, "/output/haptic"),
                          {&xr.face1, "/user/hand/left/input/x/click"}, {&xr.face2, "/user/hand/left/input/y/click"},
                          {&xr.face1, "/user/hand/right/input/a/click"}, {&xr.face2, "/user/hand/right/input/b/click"},
                          {&xr.menu, "/user/hand/left/input/menu/click"}};
    suggest("/interaction_profiles/oculus/touch_controller", touch, sizeof touch / sizeof touch[0]);
    // (Index: no Menu button - + is the left grip with B, vr_input.c)
    const Bind index[] = {BOTH(pose, "/input/grip/pose"), BOTH(trigger, "/input/trigger/value"), BOTH(grip, "/input/squeeze/value"),
                          BOTH(stick, "/input/thumbstick"), BOTH(click, "/input/thumbstick/click"), BOTH(haptic, "/output/haptic"),
                          BOTH(face1, "/input/a/click"), BOTH(face2, "/input/b/click")};
    suggest("/interaction_profiles/valve/index_controller", index, sizeof index / sizeof index[0]);
    const Bind simple[] = {BOTH(pose, "/input/grip/pose"), BOTH(trigger, "/input/select/click"), BOTH(haptic, "/output/haptic"),
                           {&xr.menu, "/user/hand/left/input/menu/click"}};
    suggest("/interaction_profiles/khr/simple_controller", simple, sizeof simple / sizeof simple[0]);
#undef BOTH
    XrSessionActionSetsAttachInfo at = {.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO, .countActionSets = 1, .actionSets = &xr.set};
    XR_TRY(xrAttachSessionActionSets(xr.session, &at), false);
    for (int h = 0; h < 2; h++) {
        XrActionSpaceCreateInfo sp = {.type = XR_TYPE_ACTION_SPACE_CREATE_INFO, .action = xr.pose, .subactionPath = xr.hand_path[h],
                                      .poseInActionSpace = {.orientation = {0, 0, 0, 1}}};
        XR_TRY(xrCreateActionSpace(xr.session, &sp, &xr.hand[h]), false);
    }
    return true;
}

void xr_session_start(void) {
    if (!xr.inst) return;
    XrGraphicsBindingVulkanKHR gb = {.type = XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR, .instance = g_vk.inst, .physicalDevice = g_vk.phys,
                                     .device = g_vk.dev, .queueFamilyIndex = g_vk.qfam, .queueIndex = 0};
    XrSessionCreateInfo ci = {.type = XR_TYPE_SESSION_CREATE_INFO, .next = &gb, .systemId = xr.sys};
    XrResult r = xrCreateSession(xr.inst, &ci, &xr.session);
    if (XR_FAILED(r)) {
        LOG("[vr] xrCreateSession: %s. Playing on the window only\n", xr_err(r));
        xr.session = XR_NULL_HANDLE;
        return;
    }
    XrReferenceSpaceCreateInfo rs = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO, .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL,
                                     .poseInReferenceSpace = {.orientation = {0, 0, 0, 1}}};
    if (XR_FAILED(r = xrCreateReferenceSpace(xr.session, &rs, &xr.local))) LOG("[vr] the play space: %s\n", xr_err(r));
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    if (XR_FAILED(r = xrCreateReferenceSpace(xr.session, &rs, &xr.view))) LOG("[vr] the head's space: %s\n", xr_err(r));
    if (!xr.local || !actions_create()) {
        LOG("[vr] the session could not be set up. Playing on the window only\n");
        xrDestroySession(xr.session);
        xr.session = XR_NULL_HANDLE;
        return;
    }
    if (xr.hand_tracking) {
        for (int h = 0; h < 2; h++) {
            XrHandTrackerCreateInfoEXT hi = {.type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
                .hand = h == VR_LEFT ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT, .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT};
            r = p_xrCreateHandTrackerEXT(xr.session, &hi, &xr.tracker[h]);
            if (XR_FAILED(r)) {
                xr.tracker[h] = XR_NULL_HANDLE;
                LOG("[vr] hand tracker %d unavailable: %s\n", h, xr_err(r));
            }
        }
    }
    vr_session_made();
    LOG("[vr] session made: the picture goes to the headset once it is worn\n");
}

bool xr_on(void) { return xr.session != XR_NULL_HANDLE && !xr.lost; }

// ---- session state
static void clear_input(void) {
    memset(prev_pose, 0, sizeof prev_pose);
    VrInput in = {.t = (double)xr.time * 1e-9};
    vr_input_update(&in);
}

static void destroy_hand_trackers(void) {
    for (int h = 0; h < 2; h++) {
        if (xr.tracker[h]) p_xrDestroyHandTrackerEXT(xr.tracker[h]);
        xr.tracker[h] = XR_NULL_HANDLE;
    }
}

static void poll_events(void) {
    XrEventDataBuffer ev = {.type = XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(xr.inst, &ev) == XR_SUCCESS) {
        if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            XrSessionState s = ((const XrEventDataSessionStateChanged *)&ev)->state;
            if (s == XR_SESSION_STATE_READY) {
                XrSessionBeginInfo bi = {.type = XR_TYPE_SESSION_BEGIN_INFO, .primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
                XrResult r = xrBeginSession(xr.session, &bi);
                xr.running = XR_SUCCEEDED(r);
                LOG("[vr] headset on: %s\n", xr.running ? "session running" : xr_err(r));
            } else if (s == XR_SESSION_STATE_STOPPING) {
                clear_input();
                xrEndSession(xr.session);
                xr.running = false;
                LOG("[vr] headset off: session stopped\n");
            } else if (s == XR_SESSION_STATE_EXITING || s == XR_SESSION_STATE_LOSS_PENDING) {
                clear_input();
                destroy_hand_trackers();
                xr.running = false, xr.lost = true;
                vr_session_lost();
                LOG("[vr] the headset's session ended (%s): playing on the window only\n", s == XR_SESSION_STATE_EXITING ? "closed there" : "lost");
            }
        } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            clear_input();
            destroy_hand_trackers();
            xr.running = false, xr.lost = true;
            vr_session_lost();
            LOG("[vr] the OpenXR runtime is going away: playing on the window only\n");
        }
        ev = (XrEventDataBuffer){.type = XR_TYPE_EVENT_DATA_BUFFER};
    }
}

// ---- the controllers, once per frame
static void locate(XrSpace space, int slot, XrTime t, VrPose *out) {
    XrSpaceVelocity v = {.type = XR_TYPE_SPACE_VELOCITY};
    XrSpaceLocation l = {.type = XR_TYPE_SPACE_LOCATION, .next = &v};
    memset(out, 0, sizeof *out);
    out->rot[3] = 1;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    if (!space || XR_FAILED(xrLocateSpace(space, xr.local, t, &l)) || (l.locationFlags & need) != need) { prev_pose[slot].have = false; return; }
    out->pos[0] = l.pose.position.x, out->pos[1] = l.pose.position.y, out->pos[2] = l.pose.position.z;
    out->rot[0] = l.pose.orientation.x, out->rot[1] = l.pose.orientation.y, out->rot[2] = l.pose.orientation.z, out->rot[3] = l.pose.orientation.w;
    if (v.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) {
        out->vel[0] = v.linearVelocity.x, out->vel[1] = v.linearVelocity.y, out->vel[2] = v.linearVelocity.z;
    } else if (prev_pose[slot].have && t > prev_pose[slot].t && t - prev_pose[slot].t < 100000000ll) {  // no velocity across a tracking gap
        float dt = (float)((t - prev_pose[slot].t) * 1e-9);
        for (int i = 0; i < 3; i++) out->vel[i] = (out->pos[i] - prev_pose[slot].pos[i]) / dt;
    }
    if (v.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)  // (the ball's spin: vr_input.c; none from the runtime, none)
        out->avel[0] = v.angularVelocity.x, out->avel[1] = v.angularVelocity.y, out->avel[2] = v.angularVelocity.z;
    memcpy(prev_pose[slot].pos, out->pos, sizeof prev_pose[slot].pos);
    prev_pose[slot].t = t, prev_pose[slot].have = true;
    out->valid = true;
}
static float get_float(XrAction a, int hand) {
    XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO, .action = a, .subactionPath = xr.hand_path[hand]};
    XrActionStateFloat s = {.type = XR_TYPE_ACTION_STATE_FLOAT};
    return a && XR_SUCCEEDED(xrGetActionStateFloat(xr.session, &gi, &s)) && s.isActive ? s.currentState : 0;
}
static bool get_bool(XrAction a, int hand) {
    XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO, .action = a, .subactionPath = hand < 0 ? XR_NULL_PATH : xr.hand_path[hand]};
    XrActionStateBoolean s = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
    return a && XR_SUCCEEDED(xrGetActionStateBoolean(xr.session, &gi, &s)) && s.isActive && s.currentState;
}

static void locate_joints(int h, XrTime t, VrInput *in) {
    if (!xr.tracker[h]) return;
    _Static_assert(VR_HAND_JOINT_COUNT == XR_HAND_JOINT_COUNT_EXT, "hand joint count");
    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT] = {0};
    XrHandJointVelocityEXT velocities[XR_HAND_JOINT_COUNT_EXT] = {0};
    XrHandJointVelocitiesEXT vs = {.type = XR_TYPE_HAND_JOINT_VELOCITIES_EXT,
        .jointCount = XR_HAND_JOINT_COUNT_EXT, .jointVelocities = velocities};
    XrHandJointLocationsEXT ls = {.type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT, .next = &vs,
        .jointCount = XR_HAND_JOINT_COUNT_EXT, .jointLocations = joints};
    XrHandJointsLocateInfoEXT li = {.type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT, .baseSpace = xr.local, .time = t};
    if (XR_FAILED(p_xrLocateHandJointsEXT(xr.tracker[h], &li, &ls)) || !ls.isActive) return;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT
        | XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    for (int j = 0; j < VR_HAND_JOINT_COUNT; j++) {
        if ((joints[j].locationFlags & need) != need) continue;
        VrPose *p = &in->joint[h][j];
        XrPosef *pose = &joints[j].pose;
        *p = (VrPose){.pos = {pose->position.x, pose->position.y, pose->position.z},
            .rot = {pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w}, .valid = true};
        if (velocities[j].velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) {
            p->vel[0] = velocities[j].linearVelocity.x;
            p->vel[1] = velocities[j].linearVelocity.y;
            p->vel[2] = velocities[j].linearVelocity.z;
        }
        if (velocities[j].velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) {
            p->avel[0] = velocities[j].angularVelocity.x;
            p->avel[1] = velocities[j].angularVelocity.y;
            p->avel[2] = velocities[j].angularVelocity.z;
        }
    }
    if (!in->hand[h].valid && in->joint[h][VR_HAND_WRIST].valid) {
        in->hand[h] = in->joint[h][VR_HAND_WRIST];
        in->hand_tracked[h] = true;
        // bare hands (no controller): a pinch is the trigger (the thumb's tip on the index finger's: 5 cm apart is
        // nothing, 1.5 cm all of it), a fist the grip (the middle finger's tip by the palm: 9 cm to 4 cm)
        const VrPose *pj = in->joint[h];
        float d1 = 1, d2 = 1;
        if (pj[XR_HAND_JOINT_THUMB_TIP_EXT].valid && pj[XR_HAND_JOINT_INDEX_TIP_EXT].valid)
            d1 = hypotf(hypotf(pj[5].pos[0] - pj[10].pos[0], pj[5].pos[1] - pj[10].pos[1]), pj[5].pos[2] - pj[10].pos[2]);
        if (pj[XR_HAND_JOINT_MIDDLE_TIP_EXT].valid && pj[XR_HAND_JOINT_PALM_EXT].valid)
            d2 = hypotf(hypotf(pj[15].pos[0] - pj[0].pos[0], pj[15].pos[1] - pj[0].pos[1]), pj[15].pos[2] - pj[0].pos[2]);
        float pinch = (0.05f - d1) / 0.035f, fist = (0.09f - d2) / 0.05f;
        in->pinch[h] = pinch < 0 ? 0 : pinch > 1 ? 1 : pinch;
        in->fist[h] = fist < 0 ? 0 : fist > 1 ? 1 : fist;
    }
}

static void controllers(XrTime t) {
    XrActiveActionSet as = {xr.set, XR_NULL_PATH};
    XrActionsSyncInfo si = {.type = XR_TYPE_ACTIONS_SYNC_INFO, .countActiveActionSets = 1, .activeActionSets = &as};
    if (xrSyncActions(xr.session, &si) != XR_SUCCESS) { clear_input(); return; }
    VrInput in = {.t = (double)t * 1e-9};
    locate(xr.view, 2, t, &in.head);
    for (int h = 0; h < 2; h++) {
        XrActionStateGetInfo pi = {.type = XR_TYPE_ACTION_STATE_GET_INFO, .action = xr.pose, .subactionPath = xr.hand_path[h]};
        XrActionStatePose ps = {.type = XR_TYPE_ACTION_STATE_POSE};
        bool active = xr.pose && XR_SUCCEEDED(xrGetActionStatePose(xr.session, &pi, &ps)) && ps.isActive;
        locate(active ? xr.hand[h] : XR_NULL_HANDLE, h, t, &in.hand[h]);
        locate_joints(h, t, &in);
        in.trigger[h] = in.hand_tracked[h] ? in.pinch[h] : get_float(xr.trigger, h);
        in.grip[h] = in.hand_tracked[h] ? in.fist[h] : get_float(xr.grip, h);
        XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO, .action = xr.stick, .subactionPath = xr.hand_path[h]};
        XrActionStateVector2f s = {.type = XR_TYPE_ACTION_STATE_VECTOR2F};
        if (xr.stick && XR_SUCCEEDED(xrGetActionStateVector2f(xr.session, &gi, &s)) && s.isActive)
            in.stick[h][0] = s.currentState.x, in.stick[h][1] = s.currentState.y;
    }
    if (get_bool(xr.face1, VR_RIGHT)) in.buttons |= VRB_A;
    if (get_bool(xr.face2, VR_RIGHT)) in.buttons |= VRB_B;
    if (get_bool(xr.face1, VR_LEFT)) in.buttons |= VRB_X;
    if (get_bool(xr.face2, VR_LEFT)) in.buttons |= VRB_Y;
    if (get_bool(xr.menu, -1)) in.buttons |= VRB_MENU;
    if (get_bool(xr.click, VR_LEFT)) in.buttons |= VRB_LSTICK;
    if (get_bool(xr.click, VR_RIGHT)) in.buttons |= VRB_RSTICK;
    vr_input_update(&in);
    if (getenv("LARECOMP_LOG_VR_POSES")) {
        static XrTime last_log;
        if (t - last_log >= 1000000000ll) {
            last_log = t;
            LOG("[vr] poses head %d (%.3f %.3f %.3f), left %d (%.3f %.3f %.3f), right %d (%.3f %.3f %.3f)\n",
                in.head.valid, in.head.pos[0], in.head.pos[1], in.head.pos[2],
                in.hand[0].valid, in.hand[0].pos[0], in.hand[0].pos[1], in.hand[0].pos[2],
                in.hand[1].valid, in.hand[1].pos[0], in.hand[1].pos[1], in.hand[1].pos[2]);
        }
    }
    for (int h = 0; h < 2; h++) {
        float amp, secs;
        if (!xr.haptic || !vr_haptic_take(h, &amp, &secs)) continue;
        XrHapticActionInfo hi = {.type = XR_TYPE_HAPTIC_ACTION_INFO, .action = xr.haptic, .subactionPath = xr.hand_path[h]};
        if (amp > 0) {
            XrHapticVibration hv = {.type = XR_TYPE_HAPTIC_VIBRATION, .duration = (XrDuration)(secs * 1e9f),
                                    .frequency = XR_FREQUENCY_UNSPECIFIED, .amplitude = amp > 1 ? 1 : amp};
            xrApplyHapticFeedback(xr.session, &hi, (const XrHapticBaseHeader *)&hv);
        } else xrStopHapticFeedback(xr.session, &hi);
    }
}

// ---- the screen's swapchain
// The window's picture is sRGB-encoded bytes in a UNORM image (device.c create_swapchain); the headset's compositor
// takes a UNORM image for linear light, so its image is the sRGB format of the same byte order and the bytes are copied
// as they are. A runtime without that format: a blit into the other order's (the colours right, the picture too
// bright: encoded twice), said in the log.
static int64_t pick_format(VkFormat window, bool *blit) {
    int64_t f[64];
    uint32_t n = 0;
    if (XR_FAILED(xrEnumerateSwapchainFormats(xr.session, 64, &n, f)) || !n) return 0;
    bool bgra = window == VK_FORMAT_B8G8R8A8_UNORM || window == VK_FORMAT_B8G8R8A8_SRGB;
    int64_t same = bgra ? VK_FORMAT_B8G8R8A8_SRGB : VK_FORMAT_R8G8B8A8_SRGB, other = bgra ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_B8G8R8A8_SRGB;
    *blit = false;
    for (uint32_t i = 0; i < n; i++) if (f[i] == same) return same;
    *blit = true;
    for (uint32_t i = 0; i < n; i++) if (f[i] == other) return other;
    return f[0];
}

static bool swapchain_for(VkFormat window, VkExtent2D e) {
    if (xr.sc && xr.sc_for == window && xr.sc_w == e.width && xr.sc_h == e.height) return true;
    if (xr.sc) {  // (the window changed size: the GPU done with the old images first)
        vk_wait_seq(vk_pending_seq() - 1);
        xrDestroySwapchain(xr.sc);
        xr.sc = XR_NULL_HANDLE;
    }
    bool blit = false;
    int64_t fmt = pick_format(window, &blit);
    if (!fmt) return false;
    XrSwapchainCreateInfo ci = {.type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                                .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT,
                                .format = fmt, .sampleCount = 1, .width = e.width, .height = e.height, .faceCount = 1, .arraySize = 1,
                                .mipCount = 1};
    XR_TRY(xrCreateSwapchain(xr.session, &ci, &xr.sc), false);
    XrSwapchainImageVulkanKHR img[16];
    for (int i = 0; i < 16; i++) img[i] = (XrSwapchainImageVulkanKHR){.type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR};
    uint32_t n = 0;
    XrResult r = xrEnumerateSwapchainImages(xr.sc, 16, &n, (XrSwapchainImageBaseHeader *)img);
    if (XR_FAILED(r) || !n) {
        LOG("[vr] the screen's images: %s\n", xr_err(r));
        xrDestroySwapchain(xr.sc);
        xr.sc = XR_NULL_HANDLE;
        return false;
    }
    for (uint32_t i = 0; i < n; i++) xr.sc_img[i] = img[i].image;
    xr.sc_n = n, xr.sc_w = e.width, xr.sc_h = e.height, xr.sc_fmt = fmt, xr.sc_blit = blit, xr.sc_for = window;
    LOG("[vr] screen %ux%u, format %lld, %u images%s\n", e.width, e.height, (long long)fmt, n,
        blit ? " (not the window's byte order: blitted, brighter than it should be)" : "");
    return true;
}

// ---- first person: the eyes' swapchains
static bool eyes_swapchains(void) {
    if (xr.esc[0] && xr.esc[1]) return true;
    if (xr.e_failed) return false;
    xr.e_failed = true;  // (until it all worked: not tried again each frame)
    XrViewConfigurationView cv[2] = {{.type = XR_TYPE_VIEW_CONFIGURATION_VIEW}, {.type = XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    uint32_t n = 0;
    XR_TRY(xrEnumerateViewConfigurationViews(xr.inst, xr.sys, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &n, cv), false);
    int64_t f[64], fmt = 0;
    uint32_t nf = 0;
    if (XR_FAILED(xrEnumerateSwapchainFormats(xr.session, 64, &nf, f))) return false;
    for (uint32_t i = 0; i < nf && !fmt; i++) if (f[i] == VK_FORMAT_R8G8B8A8_SRGB || f[i] == VK_FORMAT_B8G8R8A8_SRGB) fmt = f[i];
    if (!fmt || n != 2) { LOG("[vr] first person: the headset offers no sRGB image format - the screen in front of the player instead\n"); return false; }
    // LARECOMP_VR_EYE_SCALE: the eyes' images, times this on each side (1: what the headset asks for)
    float k = getenv("LARECOMP_VR_EYE_SCALE") ? (float)atof(getenv("LARECOMP_VR_EYE_SCALE")) : 1.0f;
    if (!(k >= 0.25f && k <= 2.0f)) k = 1;
    uint32_t w = (uint32_t)((float)cv[0].recommendedImageRectWidth * k) & ~1u, h = (uint32_t)((float)cv[0].recommendedImageRectHeight * k) & ~1u;
    if (w > cv[0].maxImageRectWidth) w = cv[0].maxImageRectWidth;
    if (h > cv[0].maxImageRectHeight) h = cv[0].maxImageRectHeight;
    for (int e = 0; e < 2; e++) {
        XrSwapchainCreateInfo ci = {.type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                                    .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT,
                                    .format = fmt, .sampleCount = 1, .width = w, .height = h, .faceCount = 1, .arraySize = 1, .mipCount = 1};
        XR_TRY(xrCreateSwapchain(xr.session, &ci, &xr.esc[e]), false);
        XrSwapchainImageVulkanKHR img[16];
        for (int i = 0; i < 16; i++) img[i] = (XrSwapchainImageVulkanKHR){.type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR};
        uint32_t ni = 0;
        XR_TRY(xrEnumerateSwapchainImages(xr.esc[e], 16, &ni, (XrSwapchainImageBaseHeader *)img), false);
        for (uint32_t i = 0; i < ni; i++) {
            xr.e_img[e][i] = img[i].image;
            VkImageViewCreateInfo vi = {.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO, .image = img[i].image, .viewType = VK_IMAGE_VIEW_TYPE_2D,
                                        .format = (VkFormat)fmt, .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}};
            if (vkCreateImageView(g_vk.dev, &vi, NULL, &xr.e_view[e][i]) != VK_SUCCESS) return false;
        }
        xr.e_n[e] = ni;
    }
    xr.e_w = w, xr.e_h = h, xr.e_fmt = fmt;
    xr.e_failed = false;
    LOG("[vr] first person: %ux%u for each eye, format %lld\n", w, h, (long long)fmt);
    return true;
}

// ---- frames
// The frame's end, with or without a picture: the recorder's thread (rec_call), or the presenting thread with the
// recorder idle
static void end_run(void *arg) {
    (void)arg;
    static float sw = -1, sd, sy;
    if (sw < 0) {
        sw = 3.2f, sd = 2.5f, sy = 0;
        const char *e = getenv("LARECOMP_VR_SCREEN");
        if (e) sscanf(e, "%f,%f,%f", &sw, &sd, &sy);
        if (!(sw > 0.1f)) sw = 3.2f;
        if (!(sd > 0.1f)) sd = 2.5f;
    }
    if (xr.image) {
        XrSwapchainImageReleaseInfo ri = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(xr.sc, &ri);
    }
    // First person (hle/vr_camera.c): the eyes' pictures (exec.c vk_xr_eye) go to the headset as a projection - where
    // each eye was when the game drew the frame (the head then, and the eye's place from it) and how wide it sees. The
    // compositor turns them to where the head is at each of its own frames, so the world holds still between the
    // game's. (The eyes look the way the head does: right for a headset whose screens are not turned outwards.)
    if (xr.mode == 2) {
        XrSwapchainImageReleaseInfo ri = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        XrCompositionLayerProjectionView pv[2];
        const float *q = xr.rv.head.rot, *hp = xr.rv.head.pos;
        for (int e = 0; e < 2; e++) {
            xrReleaseSwapchainImage(xr.esc[e], &ri);
            // the eye's place in the play space: the head's turn applied to its offset (v + 2 w (q x v) + 2 q x (q x v))
            float v[3] = {xr.eye_pose[e].position.x, xr.eye_pose[e].position.y, xr.eye_pose[e].position.z};
            float c1[3] = {q[1] * v[2] - q[2] * v[1], q[2] * v[0] - q[0] * v[2], q[0] * v[1] - q[1] * v[0]};
            float c2[3] = {q[1] * c1[2] - q[2] * c1[1], q[2] * c1[0] - q[0] * c1[2], q[0] * c1[1] - q[1] * c1[0]};
            pv[e] = (XrCompositionLayerProjectionView){
                .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
                .pose = {.orientation = {q[0], q[1], q[2], q[3]},
                         .position = {hp[0] + v[0] + 2 * (q[3] * c1[0] + c2[0]), hp[1] + v[1] + 2 * (q[3] * c1[1] + c2[1]),
                                      hp[2] + v[2] + 2 * (q[3] * c1[2] + c2[2])}},
                .fov = xr.eye_fov[e],
                .subImage = {.swapchain = xr.esc[e], .imageRect = {{0, 0}, {(int32_t)xr.e_w, (int32_t)xr.e_h}}}};
        }
        XrCompositionLayerProjection pl = {.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION, .space = xr.local, .viewCount = 2, .views = pv};
        const XrCompositionLayerBaseHeader *pls[1] = {(const XrCompositionLayerBaseHeader *)&pl};
        XrFrameEndInfo ei = {.type = XR_TYPE_FRAME_END_INFO, .displayTime = xr.time, .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
                             .layerCount = 1, .layers = pls};
        XrResult r = xrEndFrame(xr.session, &ei);
        static int nlog;
        if (XR_FAILED(r) && nlog++ < 10) LOG("[vr] xrEndFrame (first person): %s\n", xr_err(r));
        xr.mode = 0;
        atomic_store(&xr.begun, false);
        return;
    }
    XrCompositionLayerQuad q = {.type = XR_TYPE_COMPOSITION_LAYER_QUAD, .space = xr.local, .eyeVisibility = XR_EYE_VISIBILITY_BOTH,
                                .subImage = {.swapchain = xr.sc, .imageRect = {{0, 0}, {(int32_t)xr.sc_w, (int32_t)xr.sc_h}}},
                                .pose = {.orientation = {0, 0, 0, 1}, .position = {0, sy, -sd}},
                                .size = {sw, xr.sc_w ? sw * (float)xr.sc_h / (float)xr.sc_w : sw}};
    const XrCompositionLayerBaseHeader *layers[1] = {(const XrCompositionLayerBaseHeader *)&q};
    XrFrameEndInfo ei = {.type = XR_TYPE_FRAME_END_INFO, .displayTime = xr.time, .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
                         .layerCount = xr.image ? 1 : 0, .layers = layers};
    XrResult r = xrEndFrame(xr.session, &ei);
    static int nlog;
    if (XR_FAILED(r) && nlog++ < 10) LOG("[vr] xrEndFrame: %s\n", xr_err(r));
    xr.image = false;
    atomic_store(&xr.begun, false);
}

int xr_frame_begin(VkFormat window_format, VkExtent2D extent, VkImage *screen, bool *blit, XrEyes *eyes) {
    if (!xr_on()) return 0;
    rec_drain();  // (the last frame's end ran, and the queue is OpenXR's until this returns)
    poll_events();
    if (!xr.running || atomic_load(&xr.begun)) return 0;
    XrFrameState fs = {.type = XR_TYPE_FRAME_STATE};
    static int nlog;
    XrResult r = xrWaitFrame(xr.session, NULL, &fs);
    if (XR_SUCCEEDED(r)) r = xrBeginFrame(xr.session, NULL);
    if (XR_FAILED(r)) {
        if (nlog++ < 10) LOG("[vr] the headset's frame: %s\n", xr_err(r));
        return 0;
    }
    xr.time = fs.predictedDisplayTime;
    xr.image = false;
    atomic_store(&xr.begun, true);
    controllers(fs.predictedDisplayTime);
    {   // the field of view the game is to draw with (hle/vr_camera.c): the widest of both eyes' to each side, so the
        // one picture covers what either eye can see
        XrViewLocateInfo li = {.type = XR_TYPE_VIEW_LOCATE_INFO, .viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                               .displayTime = fs.predictedDisplayTime, .space = xr.view ? xr.view : xr.local};
        XrViewState vs = {.type = XR_TYPE_VIEW_STATE};
        XrView v[2] = {{.type = XR_TYPE_VIEW}, {.type = XR_TYPE_VIEW}};
        uint32_t nv = 0;
        if (XR_SUCCEEDED(xrLocateViews(xr.session, &li, &vs, 2, &nv, v)) && nv == 2) {
            for (int e = 0; e < 2; e++) xr.eye_pose[e] = v[e].pose, xr.eye_fov[e] = v[e].fov;
            xr.have_views = xr.view != XR_NULL_HANDLE;
            float th = 0, tv = 0;
            for (int e = 0; e < 2; e++) {
                th = fmaxf(th, fmaxf(tanf(-v[e].fov.angleLeft), tanf(v[e].fov.angleRight)));
                tv = fmaxf(tv, fmaxf(tanf(v[e].fov.angleUp), tanf(-v[e].fov.angleDown)));
            }
            vr_set_fov(th, tv);
        }
    }
    uint32_t i = 0;
    XrSwapchainImageWaitInfo wi = {.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .timeout = XR_INFINITE_DURATION};
    // First person: the game's camera was the head's for the frame shown now - found by the view matrix its scene was
    // drawn with (exec.c vk_scene_view), so the head is that frame's own, not a guess of how many frames the game
    // thread is ahead (LARECOMP_VR_POSE_LAG=<frames>: that guess instead, to compare). A frame with no scene, or a
    // scene from another camera (a menu, a cutscene's own): the screen in front of the player.
    static int lag = -2;
    if (lag == -2) lag = getenv("LARECOMP_VR_POSE_LAG") ? atoi(getenv("LARECOMP_VR_POSE_LAG")) & 7 : -1;
    xr.mode = 0;
    extern bool vk_scene_view(float out[16]);
    float sv[16];
    bool fp = lag >= 0 ? vr_render_view(lag, &xr.rv) : vk_scene_view(sv) && vr_render_view_of(sv, &xr.rv);
    if (fs.shouldRender && xr.have_views && fp && eyes_swapchains()) {
        uint32_t k[2] = {0, 0};
        int got = 0;
        for (int e = 0; e < 2; e++) {
            if (XR_FAILED(r = xrAcquireSwapchainImage(xr.esc[e], NULL, &k[e])) || k[e] >= xr.e_n[e]) break;
            got++;
            if (XR_FAILED(r = xrWaitSwapchainImage(xr.esc[e], &wi))) break;
        }
        if (got == 2 && XR_SUCCEEDED(r)) {
            static int said;
            if (!said++) LOG("[vr] first person: the game's camera is the head's (drawn %.0f x %.0f degrees wide)\n",
                             2 * atanf(xr.rv.tan_h) * 57.2958f, 2 * atanf(xr.rv.tan_v) * 57.2958f);
            for (int e = 0; e < 2; e++) {
                eyes->image[e] = xr.e_img[e][k[e]], eyes->view[e] = xr.e_view[e][k[e]];
                const XrFovf *f = &xr.eye_fov[e];
                eyes->fov[e][0] = tanf(f->angleLeft), eyes->fov[e][1] = tanf(f->angleRight), eyes->fov[e][2] = tanf(f->angleUp), eyes->fov[e][3] = tanf(f->angleDown);
                eyes->eye[e][0] = xr.eye_pose[e].position.x, eyes->eye[e][1] = xr.eye_pose[e].position.y, eyes->eye[e][2] = xr.eye_pose[e].position.z;
            }
            eyes->format = (VkFormat)xr.e_fmt, eyes->w = xr.e_w, eyes->h = xr.e_h, eyes->rv = xr.rv;
            xr.mode = 2;
            return 2;
        }
        // (an eye's image not to be had: what was acquired goes back, and the frame ends with no picture)
        XrSwapchainImageReleaseInfo ri = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        for (int e = 0; e < got; e++) xrReleaseSwapchainImage(xr.esc[e], &ri);
        if (nlog++ < 10) LOG("[vr] the eyes' images: %s\n", xr_err(r));
        end_run(NULL);
        return 0;
    }
    if (!fs.shouldRender || !swapchain_for(window_format, extent) ||
        XR_FAILED(r = xrAcquireSwapchainImage(xr.sc, NULL, &i)) || i >= xr.sc_n) {
        end_run(NULL);  // (a frame without a picture: the headset keeps the last one)
        return 0;
    }
    xr.image = true;  // (acquired: released at the end whatever happens)
    if (XR_FAILED(r = xrWaitSwapchainImage(xr.sc, &wi))) {
        if (nlog++ < 10) LOG("[vr] the screen's image: %s\n", xr_err(r));
        end_run(NULL);
        return 0;
    }
    *blit = xr.sc_blit;
    *screen = xr.sc_img[i];
    xr.mode = 1;
    return 1;
}

void xr_frame_end(void) {
    if (atomic_load(&xr.begun)) rec_call(end_run, NULL);
}

#else  // a build without OpenXR (Makefile VR)

bool xr_startup(void) {
    if (vr_wanted()) LOG("[vr] this build has no OpenXR (built without its loader): playing on the window only\n");
    return false;
}
uint32_t xr_instance_extensions(const char **out, uint32_t n, uint32_t max) { (void)out, (void)max; return n; }
uint32_t xr_device_extensions(const char **out, uint32_t n, uint32_t max) { (void)out, (void)max; return n; }
VkPhysicalDevice xr_physical_device(void) { return VK_NULL_HANDLE; }
void xr_session_start(void) {}
bool xr_on(void) { return false; }
int xr_frame_begin(VkFormat window_format, VkExtent2D extent, VkImage *screen, bool *blit, XrEyes *eyes) {
    (void)window_format, (void)extent, (void)screen, (void)blit, (void)eyes;
    return 0;
}
void xr_frame_end(void) {}

#endif

