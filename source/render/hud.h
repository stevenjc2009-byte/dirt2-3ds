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

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RENDER_HUD_H */
