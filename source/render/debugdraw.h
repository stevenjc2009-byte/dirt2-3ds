/*---------------------------------------------------------------------------------
 * debugdraw.h -- immediate-mode wireframe/line drawing and on-screen text,
 * for making Phase 1 suspension/tyre behaviour VISIBLE and verifiable on
 * real hardware, per this project's "prove it, don't reason about it"
 * verification discipline.
 *
 * OWNER: render. Built on citro3d for lines/wireframes and citro2d for text
 * (frame counters, per-wheel compression/slip readouts) -- renderer.c owns
 * the actual C3D_FrameBegin/End and C2D_SceneBegin calls; debugdraw.c just
 * queues primitives between them.
 *
 * This is explicitly a DEBUG tool, not gameplay rendering -- renderer.h
 * owns the real car/world rendering. Keeping debugdraw entirely separate
 * means it can be compiled out or a call to debugdraw_frame_begin/end can
 * become a no-op without touching renderer.c at all.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RENDER_DEBUGDRAW_H
#define DIRT2_RENDER_DEBUGDRAW_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADDITIVE NOTE: every `uint32_t rgba` parameter below packs as
 * 0xRRGGBBAA (red in the highest byte, alpha in the lowest) -- e.g.
 * 0xFF0000FF is opaque red. debugdraw.c converts internally wherever an
 * underlying API wants a different byte order (citro2d's C2D_Color32 packs
 * the opposite way, alpha-highest); callers of this header never need to
 * know that, they just always use 0xRRGGBBAA. */

void debugdraw_init(void);
void debugdraw_shutdown(void);

/* Call once per render frame before any debugdraw_* draw call, and
 * debugdraw_frame_end once after -- brackets this frame's queued
 * primitives. */
void debugdraw_frame_begin(void);
void debugdraw_frame_end(void);

/* A single world-space line segment, one color for the whole segment. Used
 * for suspension raycasts (rest length + current compression), wheel
 * contact normals, and velocity vectors -- the load-bearing Phase 1 debug
 * views for "is the suspension actually compressing/rebounding
 * correctly." */
void debugdraw_line(Vec3 from, Vec3 to, uint32_t rgba);

/* Axis-aligned wire box in world space, used for a simple chassis
 * bounding-box overlay before real car art exists. */
void debugdraw_wire_box(Vec3 center, Vec3 half_extents, uint32_t rgba);

/* A short wire cross/sphere-ish marker at a single world-space point --
 * used for wheel contact points. */
void debugdraw_point(Vec3 position, f32 size, uint32_t rgba);

/* Screen-space text at pixel coordinates (top-left origin), for numeric
 * readouts (per-wheel compression, slip ratio/angle, speed, frame time) --
 * the numbers that turn "looks about right" into something actually
 * checked against expected values on real hardware.
 *
 * uint32_t rgba (not libctru's u32) so this header has zero dependency on
 * 3ds.h -- render/ modules that need to compile in the host physics build
 * (they don't today, but debugdraw.h costs nothing to keep clean) never
 * pull in libctru types through here. */
void debugdraw_text(f32 screen_x, f32 screen_y, uint32_t rgba, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RENDER_DEBUGDRAW_H */
