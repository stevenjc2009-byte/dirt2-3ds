/*---------------------------------------------------------------------------------
 * camera.h -- chase camera that follows the vehicle's INTERPOLATED draw pose.
 *
 * OWNER: render.
 *
 * Reads the vehicle's chassis pose already blended for this render frame
 * (see core/timestep.h's alpha and vec3_lerp/quat_slerp in core/vecmath.h)
 * -- camera.c does not know about physics steps, accumulators or PHYSICS_DT
 * at all, only about "where is the car this frame, for drawing."
 *
 * NO CITRO3D / C3D_Mtx HERE, DELIBERATELY.
 *   This module hands the renderer a smoothed eye point, look-at point and
 *   world-up convention (position/target below, world up is always +Y --
 *   see camera_update) -- exactly what a citro3d Mtx_LookAt call needs --
 *   rather than building a C3D_Mtx itself. Two reasons: (1) camera.c's own
 *   file header requires it to stay libctru/citro3d-free so it keeps
 *   compiling and testing on the WSL host build, and (2) render/renderer.c
 *   already owns both the view matrix (Mtx_LookAt from a Camera's
 *   position/target, world up (0,1,0)) and the projection matrix
 *   (Mtx_PerspTilt) -- see renderer.c's renderer_frame_begin and its header
 *   comment. That comment also flags, as a separate pre-existing gap, that
 *   renderer.c cannot currently reach CameraConfig's fov_degrees/near_clip/
 *   far_clip and keeps its own copy of those numbers instead; closing that
 *   gap needs a renderer.h signature change, which is out of camera.h/.c's
 *   ownership -- reported, not made, per this project's per-file ownership
 *   split.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RENDER_CAMERA_H
#define DIRT2_RENDER_CAMERA_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CameraConfig {
    f32 follow_distance;    /* metres behind the car                       */
    f32 follow_height;      /* metres above the car's chassis origin       */
    f32 look_ahead_height;  /* metres above the car the look-at point sits */
    f32 position_smoothing; /* 0..1 per-second blend factor toward the
                              * ideal follow position -- 0 means the camera
                              * never moves, 1 means it snaps instantly     */
    f32 fov_degrees;
    f32 near_clip;
    f32 far_clip;
} CameraConfig;

typedef struct Camera {
    Vec3 position;   /* current world-space camera position, smoothed      */
    Vec3 target;     /* current world-space look-at point, smoothed        */

    /* Internal state, not read by renderer.c: a unit, world-space, HORIZONTAL
     * (y == 0) heading vector -- the camera's own lagged idea of "which way
     * the car is facing," updated and consumed entirely inside camera.c (see
     * camera_update). Exists so a fast yaw change makes the camera sweep
     * around the car over time instead of the position/target points cutting
     * a straight line through where the car used to be, and so the chassis's
     * pitch (ramps, jump attitude) and roll never reach the follow position
     * at all -- see camera_update's header comment for why. */
    Vec3 smoothed_forward;
} Camera;

void camera_init(Camera *cam, const CameraConfig *config);

/* Advances the smoothed camera position/target toward the ideal chase pose
 * behind `chassis_pos`/`chassis_orient` (the vehicle's INTERPOLATED draw
 * pose for this render frame, not a raw physics-step pose) by `dt` real
 * seconds (render frame time, NOT PHYSICS_DT). */
void camera_update(Camera *cam, const CameraConfig *config,
                    Vec3 chassis_pos, Quat chassis_orient, f32 dt);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RENDER_CAMERA_H */
