/*---------------------------------------------------------------------------------
 * timestep.h -- fixed-timestep accumulator with render interpolation.
 *
 * OWNER: core.
 *
 * WHY FIXED TIMESTEP.
 *   The physics in vehicle.h (suspension -> tyre -> integration, see that
 *   header's canonical order) must be deterministic and stable regardless of
 *   the 3DS's actual frame rate, which on real hardware varies with GPU load
 *   and is never exactly 60 Hz. Running physics directly off a variable
 *   render dt makes spring/damper stability depend on frame time, which is
 *   exactly the kind of bug that only shows up on real hardware and not in
 *   an emulator running full-speed. Fixed-step + accumulator decouples the
 *   two: physics always sees the same dt, the render loop just asks "how
 *   many physics steps do I owe" each frame and interpolates the leftover
 *   fraction for a smooth draw pose.
 *
 * PHYSICS_HZ IS A COMPILE-TIME CONSTANT, OVERRIDABLE.
 *   Default target is 120 Hz. The test harness (tests/test_physics.c) must
 *   be able to run the SAME physics code at 1/60 and 1/100 as well, because
 *   suspension/tyre stability has to be checked across step sizes, not just
 *   at the shipping rate. Override on the command line, e.g.:
 *
 *     make EXTRA_CFLAGS=-DPHYSICS_HZ=60
 *     gcc ... -DPHYSICS_HZ=100 ...   (Makefile.host, per test binary)
 *
 *   Do NOT read the physics rate from a runtime variable -- it is a
 *   compile-time constant on purpose, so PHYSICS_DT (1.0f / PHYSICS_HZ) is
 *   itself a compile-time constant and every module that hardcodes
 *   assumptions about step size (there should be none, but if one creeps in)
 *   fails to compile instead of failing silently at runtime.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_CORE_TIMESTEP_H
#define DIRT2_CORE_TIMESTEP_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PHYSICS_HZ
#define PHYSICS_HZ 120
#endif

/* Seconds per physics step. Compile-time constant -- see header note above. */
#define PHYSICS_DT (1.0f / (f32)PHYSICS_HZ)

/* Accumulator state for the fixed-step / variable-render decoupling.
 * One instance lives for the lifetime of the app (main.c owns it). */
typedef struct Timestep {
    f32 accumulator;        /* seconds of unsimulated time banked up      */
    f32 alpha;               /* 0..1 fraction into the next physics step,
                              * for render interpolation (see
                              * timestep_get_alpha) */
    uint32_t steps_this_frame; /* how many physics steps the last call to
                                * timestep_advance produced -- diagnostic /
                                * test-observable, not consumed by physics */
} Timestep;

void timestep_init(Timestep *ts);

/* Feeds `frame_dt` (real elapsed seconds since the last call, e.g. from
 * osGetTime()) into the accumulator. Clamps internally against a maximum
 * frame_dt (a spike -- e.g. a debugger breakpoint or a slow romfs read) so
 * the accumulator cannot demand an unbounded number of catch-up steps in one
 * frame ("spiral of death"); the exact clamp constant is an implementation
 * decision left to timestep.c, documented there.
 *
 * Returns the number of whole PHYSICS_DT-sized steps now owed. The caller
 * (main.c's frame loop) must call vehicleStep() (see vehicle.h) exactly that
 * many times, each with dt == PHYSICS_DT, before drawing. */
uint32_t timestep_advance(Timestep *ts, f32 frame_dt);

/* Fraction (0..1) of the way into the NEXT physics step, valid only after
 * timestep_advance has been called for the current frame. The renderer
 * blends the previous and current physics poses by this value
 * (vec3_lerp / quat_slerp, see vecmath.h) to draw a smooth position between
 * two 120 Hz physics samples on a 60 Hz (or uncapped) screen. */
f32 timestep_get_alpha(const Timestep *ts);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_CORE_TIMESTEP_H */
