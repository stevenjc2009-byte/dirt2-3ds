/*---------------------------------------------------------------------------------
 * camera.c -- smoothed chase camera. See camera.h for the contract: this
 * file reads the vehicle's already-INTERPOLATED draw pose and has no
 * knowledge of physics steps, accumulators or PHYSICS_DT at all -- only
 * `dt`, the real elapsed render-frame seconds passed in by main.c.
 *
 * No #ifdef __3DS__ anywhere in this file: it is plain float/Vec3/Quat
 * math with no libctru/citro3d dependency, so it compiles identically on
 * the WSL host build and the 3DS build.
 *---------------------------------------------------------------------------------*/
#include "render/camera.h"
#include "core/vecmath.h"
#include <math.h>

void camera_init(Camera *cam, const CameraConfig *config) {
    (void)config;
    if (!cam) return;
    cam->position = vec3_zero();
    cam->target = vec3_zero();
    /* Body-space forward is +X (see core/rigidbody.h's axis convention),
     * so this is an arbitrary-but-sane bootstrap heading: there is no
     * chassis pose to read yet at init time. The first camera_update call
     * blends it toward the car's real heading at the configured smoothing
     * rate (or snaps to it immediately if position_smoothing == 1). */
    cam->smoothed_forward = vec3_make(1.0f, 0.0f, 0.0f);
}

void camera_update(Camera *cam, const CameraConfig *config,
                    Vec3 chassis_pos, Quat chassis_orient, f32 dt) {
    if (!cam || !config) return;

    /* Defensive clamp: a zero dt (first frame, or a paused/frozen frame)
     * must leave the smoothed state untouched, and a negative dt (a clock
     * hiccup on real hardware) must not be allowed to run the exponential
     * smoothing formula below backwards. t already evaluates to 0 for
     * dt == 0 (see the formula's comment), so this only guards dt < 0. */
    if (dt < 0.0f) dt = 0.0f;

    Vec3 world_up = vec3_make(0.0f, 1.0f, 0.0f);

    /* Body-space forward is +X (see core/rigidbody.h's axis convention:
     * "x forward, y up, z right"). This is the chassis's RAW, unsmoothed,
     * full 3D facing for this frame -- includes whatever pitch/roll the
     * chassis currently has (mid-jump, landed hard, on its roof, ...). */
    Vec3 raw_forward = vec3_rotate_by_quat(vec3_make(1.0f, 0.0f, 0.0f), chassis_orient);

    /* Flatten onto the horizontal plane before it ever reaches the follow
     * position: a chase cam that inherits chassis pitch/roll swings the
     * horizon and lunges the follow point up/down on every ramp, landing or
     * tumble -- exactly what "must not roll" and "damp pitch response too"
     * rule out. Flattening removes roll entirely (there is no roll axis left
     * once only the horizontal heading is used) and removes pitch's direct
     * effect on the follow position outright, which is the simplest way to
     * satisfy "damp pitch" without adding a second, separately-tuned pitch
     * smoothing constant that main.c's CameraConfig has no field for.
     * world_up is pinned to global +Y (never the chassis's own local up) for
     * the same reason -- the airborne auto-level in suspension.h is a
     * gameplay assist, not a guarantee the chassis is ever upright.
     *
     * vec3_normalize's documented contract (vecmath.h) is to return a zero
     * vector for a (near-)zero-length input rather than dividing by zero --
     * relied on directly below to detect the degenerate case where the car
     * is pointing (near) straight up or down and its heading has no
     * horizontal component to steer the camera by. When that happens, keep
     * the previous frame's smoothed heading rather than normalizing garbage
     * or snapping to an arbitrary axis: the camera simply stops following
     * yaw for as long as the car stays vertical, which reads as "the camera
     * held its ground" rather than a visible glitch. */
    Vec3 flat_forward = vec3_normalize(vec3_make(raw_forward.x, 0.0f, raw_forward.z));
    Vec3 raw_heading = (vec3_length_sq(flat_forward) > 0.0f) ? flat_forward : cam->smoothed_forward;

    /* Framerate-independent exponential smoothing ("lerp with dt"): the
     * fraction of the remaining distance to the ideal value closed THIS call
     * is  t = 1 - (1 - smoothing)^dt , so everything below converges at the
     * same real-world-time rate regardless of how often camera_update is
     * called -- calling it twice as often (double the render rate) takes
     * twice as many, proportionally smaller steps, not a different-shaped
     * curve. This is exactly why this function takes a real `dt` and never
     * PHYSICS_DT (see camera.h's header comment): a camera whose feel
     * changed with frame rate would make the exact same physics look twitchy
     * at 30 Hz and sluggish at 60 Hz.
     *   smoothing == 0 -> t == 0 always      (camera never moves)
     *   smoothing == 1 -> t == 1 for any dt>0 (camera snaps instantly)
     * The same `t` drives the heading blend below and the position/target
     * blend at the end of this function -- CameraConfig has one smoothing
     * knob (position_smoothing), not a separate one per quantity, so reusing
     * it is the only option that does not require a CameraConfig field main.c
     * (not this task's file) would need to be taught to initialise. */
    f32 smoothing = config->position_smoothing;
    if (smoothing < 0.0f) smoothing = 0.0f;
    if (smoothing > 1.0f) smoothing = 1.0f;
    f32 t = 1.0f - powf(1.0f - smoothing, dt);

    /* Smooth the HEADING itself, not just the resulting follow position --
     * this is what makes the camera lag a fast yaw change by sweeping around
     * the car over the next several frames instead of the follow point
     * cutting a straight line through where the car used to be. Lerp-then-
     * normalize is a deliberately cheap stand-in for a true slerp: no trig
     * (asin/acos/atan2) in this per-frame path, which matters on an ARM11
     * with VFPv2 and no NEON. Guarded the same way as the flattening step
     * above: if the two headings being blended happen to (near-)cancel,
     * keep the previous heading rather than normalizing garbage. */
    Vec3 blended_heading = vec3_normalize(vec3_lerp(cam->smoothed_forward, raw_heading, t));
    if (vec3_length_sq(blended_heading) > 0.0f) {
        cam->smoothed_forward = blended_heading;
    }

    Vec3 ideal_position = vec3_add(
        vec3_sub(chassis_pos, vec3_scale(cam->smoothed_forward, config->follow_distance)),
        vec3_scale(world_up, config->follow_height));
    Vec3 ideal_target = vec3_add(chassis_pos, vec3_scale(world_up, config->look_ahead_height));

    cam->position = vec3_lerp(cam->position, ideal_position, t);
    cam->target = vec3_lerp(cam->target, ideal_target, t);

    /* Degenerate case: camera position and look-at target coincide (e.g. the
     * very first frame, before any smoothing has run: camera_init leaves
     * both at the origin, and if the car also spawns at/near the origin with
     * dt == 0 on that first call, t == 0 leaves them exactly equal). A
     * zero-length eye-to-target vector makes Mtx_LookAt (renderer.c) not
     * merely a wrong direction but a NaN-producing degenerate matrix, so
     * this must never reach the renderer even for one frame. Nudge the
     * target off the position along the current heading -- a nudge this
     * small (1 cm) is invisible once real smoothing has anything to work
     * with, but guarantees a well-defined view direction on every frame. */
    if (vec3_length_sq(vec3_sub(cam->target, cam->position)) <= 0.0f) {
        cam->target = vec3_add(cam->target, vec3_scale(cam->smoothed_forward, 0.01f));
    }
}
