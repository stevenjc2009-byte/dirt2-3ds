/*---------------------------------------------------------------------------------
 * timestep.c -- fixed-step accumulator with render interpolation alpha. See
 * timestep.h for the fixed-step/accumulator contract.
 *
 * MAX-STEPS CAP.
 *   MAX_STEPS_PER_FRAME below is 8. At the shipping rate (120 Hz) that is
 *   ~66.7 ms of physics the accumulator is allowed to bank and drain in a
 *   single timestep_advance() call -- generous enough to absorb an ordinary
 *   frame hitch (a couple of dropped 60 Hz frames) without visibly stepping
 *   the sim, but nowhere near enough to let a multi-second stall (a romfs
 *   load, a debugger breakpoint) demand hundreds of catch-up steps in one
 *   frame ("spiral of death": more steps -> longer frame -> even more steps
 *   owed next time). Any unsimulated time beyond the cap is discarded, not
 *   carried forward, so a stall costs simulated time (the car doesn't move
 *   during the stall) rather than a permanently growing backlog.
 *---------------------------------------------------------------------------------*/
#include "core/timestep.h"

#define MAX_STEPS_PER_FRAME 8u

void timestep_init(Timestep *ts) {
    if (!ts) return;
    ts->accumulator = 0.0f;
    ts->alpha = 0.0f;
    ts->steps_this_frame = 0;
}

uint32_t timestep_advance(Timestep *ts, f32 frame_dt) {
    if (!ts) return 0;

    /* Clamp the incoming frame_dt itself so a single huge spike cannot bank
     * more than MAX_STEPS_PER_FRAME steps' worth of time in the accumulator
     * to begin with. Negative frame_dt (should never happen, but a caller
     * could pass a bad clock delta) is floored to zero rather than draining
     * the accumulator backwards. */
    f32 max_frame_dt = (f32)MAX_STEPS_PER_FRAME * PHYSICS_DT;
    if (frame_dt > max_frame_dt) {
        frame_dt = max_frame_dt;
    } else if (frame_dt < 0.0f) {
        frame_dt = 0.0f;
    }

    ts->accumulator += frame_dt;

    uint32_t steps = 0;
    while (ts->accumulator >= PHYSICS_DT && steps < MAX_STEPS_PER_FRAME) {
        ts->accumulator -= PHYSICS_DT;
        steps++;
    }

    /* Defensive: if the accumulator still holds a whole step after hitting
     * the cap (only reachable if leftover time from a prior frame plus this
     * frame's clamped delta still exceeds the cap), drop the excess instead
     * of carrying it into next frame's accumulator -- carrying it forward
     * would just re-trigger the cap again next frame and the sim would never
     * catch back up to real time smoothly. */
    if (ts->accumulator >= PHYSICS_DT) {
        ts->accumulator = 0.0f;
    }

    ts->steps_this_frame = steps;
    /* alpha = accumulator / PHYSICS_DT, done as a multiply by the
     * compile-time constant PHYSICS_HZ (== 1/PHYSICS_DT) rather than a
     * runtime divide. accumulator is always in [0, PHYSICS_DT) here, so
     * alpha lands in [0, 1) as the header contract requires. */
    ts->alpha = ts->accumulator * (f32)PHYSICS_HZ;

    return steps;
}

f32 timestep_get_alpha(const Timestep *ts) {
    if (!ts) return 0.0f;
    return ts->alpha;
}
