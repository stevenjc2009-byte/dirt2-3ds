/*---------------------------------------------------------------------------------
 * renderer.h -- top-level citro3d setup and the per-frame draw entry point.
 *
 * OWNER: render. Owns the one-time citro3d/citro2d/gfx init and the
 * C3D_FrameBegin/FrameEnd pair; camera.h and debugdraw.h are called FROM
 * inside a renderer_frame_begin/end bracket, they do not manage the GPU
 * frame themselves.
 *
 * STEREO / DUAL-SCREEN NOTE (Old 3DS baseline, per this project's scope):
 * Phase 1 targets the top screen only, no stereoscopic 3D slider support
 * yet -- renderer_init sets up a single render target. New 3DS-specific
 * and stereo enhancements are explicitly later work, per this project's
 * "New 3DS enhancements layered on later" scope.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RENDER_RENDERER_H
#define DIRT2_RENDER_RENDERER_H

#include "core/types.h"
#include "render/camera.h"
#include "world/testground.h" /* Testground, for renderer_draw_ground below --
    main.c already includes this directly too (it owns picking/generating the
    world), so this adds no new file to main.c's include graph. */
#include "race/track.h" /* Track/TrackWaypoint/TRACK_MAX_WAYPOINTS, for
    renderer_draw_track below. Data-only dependency: renderer_draw_track reads
    track->waypoints/count directly and never calls track_init/track_query, so
    this header include carries no link-time dependency on race/track.c --
    whether or not race/track.c is in the Makefile's SOURCES list yet is
    irrelevant to whether THIS file builds. */

#ifdef __cplusplus
extern "C" {
#endif

/* Sets up citro3d, the top-screen render target, and citro2d (for
 * debugdraw.h's text). Called once at startup, after gfxInitDefault(). */
void renderer_init(void);

void renderer_shutdown(void);

/* Brackets one GPU frame: clears the target, prepares the projection/view
 * matrices from `cam`, and must be paired with exactly one
 * renderer_frame_end call. Everything drawn this frame (world geometry,
 * vehicle, debugdraw.h calls) happens between these two calls. */
void renderer_frame_begin(const Camera *cam);
void renderer_frame_end(void);

/* ADDITIVE (v0.1 "it drives" slice): the actual gameplay geometry, drawn as
 * flat/vertex-coloured triangles -- no textures yet. Both functions must be
 * called between renderer_frame_begin/end (same bracket rule as debugdraw.h's
 * draw calls) and may be called in either order relative to each other and
 * to any debugdraw_* call inside that same bracket. Each rebuilds and draws
 * its own geometry fresh every call -- see renderer.c's header comment for
 * why there is no persistent world mesh. */

/* Rebuilds and draws the portion of `tg`'s heightfield within
 * RENDERER_GROUND_HALF_EXTENT metres (renderer.c) of `focus` in world X/Z --
 * typically the camera's current position or the chassis's interpolated
 * draw position; either is a reasonable choice of `focus` and this function
 * does not care which. Deliberately NOT a persistent/cached world mesh (see
 * this project's "do not allocate a heightfield mesh for the whole world"
 * hard constraint) -- nothing needs invalidating as the car moves. `tg` is
 * read-only. */
void renderer_draw_ground(const Testground *tg, Vec3 focus);

/* Draws the car: one shaded box for the chassis body plus one shaded box per
 * wheel (WheelIndex-ordered, core/types.h, VEHICLE_WHEEL_COUNT of them) --
 * v0.1 scope is "a box for the car", not real car art.
 *   chassis_pos, chassis_orient : world-space chassis pose to draw THIS
 *     render frame -- the INTERPOLATED draw pose (see camera.h's header
 *     comment), i.e. the same pose already passed to camera_update, never a
 *     raw physics-step pose.
 *   wheel_local_offsets[VEHICLE_WHEEL_COUNT] : body-space offset from the
 *     chassis origin to each wheel's visual centre, WheelIndex-ordered. Every
 *     wheel box is drawn at
 *     `chassis_pos + vec3_rotate_by_quat(wheel_local_offsets[i], chassis_orient)`,
 *     with the SAME orientation as the chassis -- no per-wheel steer or spin
 *     animation in v0.1; that is cosmetic follow-up work once the car is
 *     actually visible and driving. A natural source for these offsets is
 *     each wheel's SuspensionConfig.mount_point_body
 *     (vehicle/vehicle_params.h), but this function does no suspension math
 *     itself and does not know or care where the numbers came from. */
void renderer_draw_vehicle(Vec3 chassis_pos, Quat chassis_orient,
                            const Vec3 wheel_local_offsets[VEHICLE_WHEEL_COUNT]);

/* Rebuilds and draws the racing surface as a flat ribbon around `track`'s
 * closed loop -- one quad per segment (waypoints[i] to waypoints[(i+1)%count],
 * including the closing pair, since the track is a LOOP with no last
 * segment excluded), covering the FULL loop every call, same "no persistent
 * world mesh, rebuild from scratch every draw" policy as
 * renderer_draw_ground (see this header's note above renderer_draw_ground and
 * renderer.c's file header for why).
 *
 * SIGNATURE NOTE: the brief for this function asked for
 * `renderer_draw_track(const Track *track)`, but track.h is explicit that a
 * waypoint's Y is never meaningful ("the world's own height query is the
 * only source of truth for ground height") -- so the ribbon's vertices must
 * come from a real height query, not from track->waypoints[i].center.y. `tg`
 * is that query source, added as a second parameter (matching
 * renderer_draw_ground's own `tg` parameter) rather than inventing a
 * height-query callback type; both are read-only.
 *
 * `track` is read-only; this function does not call track_query and does not
 * need or touch any caller-owned cached-segment state (that is lap.h's
 * concern, not rendering's). */
void renderer_draw_track(const Testground *tg, const Track *track);

/* ADDITIVE: exposes the top-screen render target so debugdraw.c can hand it
 * to citro2d's C2D_SceneBegin for its screen-space text pass -- renderer.c
 * still owns the target's lifetime (creation in renderer_init, deletion in
 * renderer_shutdown), this is a read-only accessor. Guarded by __3DS__ (and
 * pulls in citro3d.h) so this header stays includable, with no declaration
 * at all, from any host-build translation unit that never defines __3DS__ --
 * see this file's own header comment on the host/3DS split. */
#ifdef __3DS__
#include <citro3d.h>
C3D_RenderTarget *renderer_get_target(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RENDER_RENDERER_H */
