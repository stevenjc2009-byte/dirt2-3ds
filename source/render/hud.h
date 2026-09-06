/*---------------------------------------------------------------------------------
 * hud.h -- the BOTTOM screen: speed, revs, inputs, per-wheel state, frame cost.
 *
 * Deliberately separate from debugdraw.h, which owns the TOP screen's
 * world-space overlay. Two different screens, two different jobs: debugdraw
 * exists to make physics visible in world space while driving, this exists to
 * put numbers somewhere they can be read without cluttering the view out of
 * the windscreen. Both use citro2d, neither owns the other's render target.
 *
 * WHY A STRUCT RATHER THAN A PILE OF ARGUMENTS. hud_draw takes one HudStats by
 * pointer because the caller (main.c's frame loop) already holds every one of
 * these numbers, and threading a dozen floats through a call signature is how
 * one of them quietly ends up being the wrong one -- the struct's field names
 * are checked by the compiler, argument order is not. It also means this
 * module has ZERO dependency on Vehicle, Testground or anything else in the
 * simulation: it renders numbers it is handed and knows nothing about where
 * they came from, so it cannot be broken by a physics refactor.
 *
 * Every citro2d call in hud.c is behind #ifdef __3DS__, same as debugdraw.c,
 * so this still compiles on the WSL host build as deliberate no-ops.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RENDER_HUD_H
#define DIRT2_RENDER_HUD_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Per-wheel readout. One of these per wheel, in WheelIndex order. */
typedef struct HudWheel {
    f32  compression;      /* metres, as suspension.h reports it              */
    f32  max_travel;       /* metres, so the bar can be drawn as a fraction   */
    f32  normal_load;      /* newtons                                         */
    bool grounded;
} HudWheel;

/* Everything the bottom screen shows in one frame. Filled fresh by the caller
 * every frame; hud.c keeps no copy and no state derived from it. */
typedef struct HudStats {
    f32 speed_ms;          /* chassis linear speed, metres/second             */
    f32 engine_rpm;
    f32 max_rpm;           /* rev limiter, for scaling the rev bar            */
    f32 idle_rpm;

    f32 throttle;          /* 0..1, POST-ramp (what the physics actually got) */
    f32 brake;             /* 0..1, post-ramp                                 */
    f32 steer;             /* -1 full left .. +1 full right                   */
    bool handbrake;

    Vec3 position;         /* world space, metres                             */

    f32 frame_ms;          /* wall-clock milliseconds for the last frame      */
    f32 fps;               /* smoothed; hud.c does not smooth it itself       */
    uint32_t physics_steps; /* steps run this frame                           */

    HudWheel wheels[4];    /* VEHICLE_WHEEL_COUNT, in WheelIndex order        */

    /* Race state, straight off race/lap.h's LapState. Copied rather than
     * pointed at for the same reason as everything else here: this module
     * must not include race/lap.h, or the bottom screen stops being drawable
     * without a track. A build with no track fills these with zeroes and
     * LAP_NO_TIME and the panel renders correctly.
     *
     * The two lap times use lap.h's LAP_NO_TIME sentinel (-1.0f) for "not set
     * yet", which is why they are signed and why hud.c tests for negative
     * rather than for zero -- a genuine lap time of 0.0s is impossible, but a
     * lap time is a float and testing floats for equality to a sentinel is
     * how a sentinel gets missed. */
    int lap_count;
    f32 current_lap_time;  /* seconds, the open lap                           */
    f32 last_lap_time;     /* seconds, negative == no lap finished yet        */
    f32 best_lap_time;     /* seconds, negative == no lap finished yet        */
    f32 lap_progress;      /* 0..1 around the loop, for the progress bar      */
    f32 lateral_offset;    /* signed metres from the centreline               */
    f32 track_half_width;  /* metres; |lateral_offset| beyond this is off-course */
    bool on_track;         /* caller's verdict, so hud.c states no policy     */
} HudStats;

/* Creates the bottom-screen render target and this module's text buffer.
 * MUST be called after renderer_init(), which is what calls C3D_Init and
 * C2D_Init -- creating a render target before C3D_Init is a crash, and
 * building text before C2D_Init is a silent no-render. */
void hud_init(void);
void hud_shutdown(void);

/* Draws the whole bottom screen. MUST be called INSIDE renderer_frame_begin/
 * renderer_frame_end -- it is part of the same GPU frame, it just draws to a
 * different target, and it switches the draw target itself (C3D_FrameDrawOn).
 *
 * Call it AFTER everything that draws to the top screen. It leaves the bottom
 * screen as the active target, so anything that draws to the top afterwards
 * would land on the wrong screen. */
void hud_draw(const HudStats *stats);

/* ADDITIVE: exposes the BOTTOM-screen render target, exactly as
 * renderer.h's renderer_get_target exposes the top one and for the same
 * reason -- ui/pausemenu.c needs to hand it to C2D_SceneBegin so the pause
 * screen can take over the bottom screen while the game is paused, WITHOUT
 * creating a second render target for a screen that already has one.
 *
 * hud_init/hud_shutdown still own the lifetime; this is read-only, and
 * returns NULL if the target was never successfully created. Guarded by
 * __3DS__ (and pulls in citro3d.h) so this header stays includable, with no
 * declaration at all, from a host-build translation unit -- same pattern as
 * renderer.h. */
#ifdef __3DS__
#include <citro3d.h>
C3D_RenderTarget *hud_get_target(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RENDER_HUD_H */
