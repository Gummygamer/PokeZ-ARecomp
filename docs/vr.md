# PC VR (OpenXR)

ZA Recomp can show the game in a PC VR headset and take the headset's controllers as the gamepad. Any headset with
an OpenXR runtime on the PC: Quest over Link / Air Link (Meta's runtime), SteamVR headsets, Virtual Desktop.

This is the first stage of a full VR version. What it does now, and what it does not do yet, is listed below.

## Turning it on

1. Connect the headset to the PC first (Quest: start Link or Air Link) and make sure its app is the active OpenXR
   runtime.
2. F1 > Video > **VR headset**: On, then start the game again. Or `vr = 1` in `larecomp.ini`, or `LARECOMP_VR=1`
   for one run (`LARECOMP_VR=0` turns it off whatever the setting says).
3. Keep the game's window open on the desktop (not minimised): the headset shows what the window shows.

If no headset or no OpenXR runtime is found, the log says why (`[vr] ...` in `logs/play.log`) and the game plays on
the window as usual.

## What you get

- **A screen in front of you.** The window's picture - the start screen, the game, the F1 menu - on a screen about
  3.2 m wide, 2.5 m ahead. The headset's compositor places it for each eye at the headset's own refresh rate, so it
  stays still in space at any game frame rate. Its resolution is the window's: make the window bigger or fullscreen
  for a sharper screen. `LARECOMP_VR_SCREEN=<width>,<distance>,<height>` (metres) moves and resizes it.
- **The controllers as the gamepad** (Touch-style layout):

  | Controller | Game |
  | --- | --- |
  | Sticks, stick clicks | Sticks, stick clicks |
  | A B X Y | A B X Y |
  | Menu (left) | + |
  | Left trigger | ZL |
  | Right trigger | ZR (see throwing) |
  | Right grip | R |
  | Left grip (held) | shift: left stick = d-pad, X = L, Y = -, B = + |

- **Throwing with your hand.** Hold the right trigger (the game readies a Poké Ball: ZR held), swing your arm as if
  throwing, and the ball goes at the moment a real one would leave your hand - the trigger can stay held. Releasing
  the trigger without a swing releases ZR as a gamepad would.
  - `LARECOMP_VR_THROW_SPEED=<m/s>`: how fast the hand must move (default 2.0).
  - `LARECOMP_VR_THROW_HAND=left`: throw with the left hand (the triggers swap).
  - `LARECOMP_VR_THROW=tap`: the swing is a short ZR press instead (the trigger alone does nothing); `=0`: no
    gesture, the trigger is plain ZR.
- **Vibration.** The game's rumble on both controllers, and a short pulse on the throw.
- **Your arms are the character's (game 2.0.0, first person).** The character's arms reach to where your hands are:
  shoulder to elbow to wrist solved each frame on the character's own skeleton, the hand turned as yours is, the
  forearm twisting with it. With controllers, the fingers close with the grip (middle, ring, little finger) and the
  trigger (index finger). With bare hands (the headset's hand tracking: Quest with hand tracking on in Link), each
  finger bends as yours does, a pinch is the trigger and a fist is the grip, so throwing works without controllers.
  When a hand is not seen, that arm goes back to the game's animation.
  - `LARECOMP_VR_ARMS=0`: the arms stay the game's animation.
  - `LARECOMP_VR_WRIST=<m>`: how far behind the controller's grip the wrist is (default 0.07).

## Not done yet

- **Independent scene rendering for each eye.** Game 2.0.0 now has a head-tracked first-person camera anchored
  to the player. The scene is still drawn once; `vr_eye.frag` synthesizes eye images using scene depth. This can
  distort nearby geometry, silhouettes and HUD elements. `LARECOMP_VR_STEREO=0` disables that depth warp for
  diagnosis while retaining head tracking; it produces a monoscopic scene, not true stereo rendering.
  Each eye sees a little background beside near things that the one picture does not have: the depth is smoothed
  first so that background stretches around them (without it, a faint copy of every character's outline showed
  beside it, a different side in each eye). `LARECOMP_VR_SMOOTH=<texels>` sets how far the smoothing reaches (96;
  `0` turns it off).
- **Player body.** The arms follow the hands (above), but the rest of the body is still the game's animation: the
  torso doesn't lean with the head, and walking animations still swing the shoulders under the tracked arms.
  Head-mesh suppression and reliable torso visibility are not implemented. The neck-to-eye offset follows head
  ides every call in that second, `=0`
  none). `LARECOMP_LOG_BALL=1` logs each call's velocities (`[ball]`), with or without a headset. Game versions
  1.0.0 and 2.0.2 need their own address for the patch.
- **The F1 menu and the start screen with the VR controllers.** Use the keyboard, mouse or a gamepad for those.
- **Other controllers' layouts** beyond Touch, Index and the generic OpenXR one, and remapping.

## For developers

- `runtime/vr.h`: the interface. `runtime/nvn/vulkan/xr.c`: the OpenXR session. `runtime/hle/vr_input.c`: the pad
  mapping and the throw detection (no OpenXR in it).
- `runtime/hle/vr_camera.c`: the 2.0.0 camera submission diagnostic. Its 0xd0-byte block contains lens settings
  at +0x00, world position at +0x40, view at +0x50 and projection at +0x90. Both matrices are column-major,
  as is the scene's view-projection. Throw directions use matrix rows indexed across the stored columns;
  the translation column must never be used as the forward direction.
- `vr_tracking_snapshot` returns a coherent head/controller sample and clears it after 500 ms without input.
  `LARECOMP_LOG_VR_POSES=1` logs pose validity and positions once per second while OpenXR input is focused.
- `tools/test_vr_math.c` checks throwing direction with translated, rotated and pitched cameras, and rejects
  invalid input. Compile with a C compiler and run the resulting executable.
- Built in when `pkg-config openxr` finds the loader (MSYS2: `mingw-w64-ucrt-x86_64-openxr-sdk`); `make VR=0`
  builds without. The release folder then needs `libopenxr_loader.dll` next to the exe (the `dist` target's DLL
  collection picks it up).
- OpenXR uses the Vulkan queue inside its frame calls, and the queue belongs to the recorder thread: a headset frame
  begins after `rec_drain()` and ends in a recorder packet (`rec_call`). That drain makes the render thread wait
  for the recorder once per frame.
