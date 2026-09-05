/*---------------------------------------------------------------------------------
 * lap.c -- see lap.h for the module contract and the full design rationale
 * behind the far-side gate.
 *
 * FAULT INJECTION (see tests/test_race.c's header for the full scheme,
 * matching tests/test_physics.c's established convention). Building with
 * -DDIRT2_INJECT_FAULT=<N> deliberately breaks ONE specific piece of real
 * logic below so a specific check in tests/test_race.c is proven able to
 * fail on real code, not just an absent implementation:
 *   1 -- always treat the far side as reached (removes the gate entirely)
 *   2 -- inverts the forward/backward wrap classification
 *   3 -- best_lap_time always overwritten, "improvement" never checked
 *   4 -- time accumulation ignores dt (counts steps instead)
 *   6 -- the priming guard is skipped, so step 0 evaluates a delta against
 *        an unseeded prev_progress
 * (5 lives in track.c -- see that file.) Default (no -D) is 0, the normal
 * build.
 *---------------------------------------------------------------------------------*/
#include "race/lap.h"

#ifndef DIRT2_INJECT_FAULT
#define DIRT2_INJECT_FAULT 0
#endif

/* The loop's midpoint -- as far from the start/finish line (progress 0/1)
 * as the geometry allows. See lap.h's file header for what this gates. */
#define LAP_FAR_SIDE_PROGRESS 0.5f

/* True iff the closed interval [min(prev,curr), max(prev,curr)] contains
 * the far-side progress value -- i.e. this step's motion passed through
 * the loop's midpoint, in EITHER direction. Only meaningful for a step
 * lap_update has already classified as NOT a wrap (see there); a wrapping
 * step's prev/curr are on opposite sides of the 0/1 seam, not a short arc
 * through 0.5. */
static bool crosses_far_side(f32 prev_progress, f32 progress) {
    f32 lo = (prev_progress < progress) ? prev_progress : progress;
    f32 hi = (prev_progress < progress) ? progress : prev_progress;
    return lo <= LAP_FAR_SIDE_PROGRESS && hi >= LAP_FAR_SIDE_PROGRESS;
}

void lap_init(LapState *state, const Track *track) {
    state->track = track;
    state->cached_segment = TRACK_UNKNOWN_SEGMENT;
    state->primed = false;
    state->prev_progress = 0.0f;
    state->far_side_reached = false;
    state->lap_count = 0;
    state->current_lap_time = 0.0f;
    state->last_lap_time = LAP_NO_TIME;
    state->best_lap_time = LAP_NO_TIME;
    state->total_time = 0.0f;
}

void lap_notify_reset(LapState *state) {
    state->primed = false;
    state->far_side_reached = false;
    /* Force the next track_query to do a full scan instead of trusting a
     * windowed search from wherever the car used to be -- see track.h's
     * track_query comment and this header's lap_notify_reset comment. */
    state->cached_segment = TRACK_UNKNOWN_SEGMENT;
}

bool lap_update(LapState *state, Vec3 car_world_pos, f32 dt) {
    TrackQueryResult q;
    f32 prev, curr, raw_delta;
    bool completed_lap = false;

    track_query(state->track, car_world_pos, &state->cached_segment, &q);

    state->total_time +=
#if DIRT2_INJECT_FAULT == 4
        1.0f; /* deliberately broken: counts steps, not seconds */
#else
        dt;
#endif

#if DIRT2_INJECT_FAULT == 6
    /* Deliberately broken: skip the priming guard, so the very first
     * sample evaluates a delta against an unseeded prev_progress (0.0)
     * instead of just recording its own position. */
#else
    if (!state->primed) {
        state->prev_progress = q.progress;
        state->primed = true;
        state->current_lap_time += dt;
        if (q.progress >= LAP_FAR_SIDE_PROGRESS) {
            /* Starting mid-lap already past the midpoint -- see lap.h's
             * lap_init comment on why this legitimately primes the gate
             * rather than requiring a lap that started on the far side to
             * loop all the way back around to it again. */
            state->far_side_reached = true;
        }
        return false;
    }
#endif

    prev = state->prev_progress;
    curr = q.progress;
    raw_delta = curr - prev;

#if DIRT2_INJECT_FAULT == 2
    /* Deliberately broken: swap the forward/backward classification. */
    { f32 tmp = raw_delta; raw_delta = -tmp; }
#endif

    if (raw_delta < -0.5f) {
        /* Forward wrap: progress fell off the top of [0,1) and reappeared
         * near 0 -- the car crossed the start/finish line moving forward.
         * The ONLY branch that can ever increment lap_count, and only
         * when it does per the far-side gate (see file header). */
#if DIRT2_INJECT_FAULT == 1
        bool gate_open = true; /* deliberately broken: gate removed */
#else
        bool gate_open = state->far_side_reached;
#endif
        if (gate_open) {
            /* Split this step's dt at the crossing so a lap completing
             * mid-step attributes the right fraction of THIS step's time
             * to the lap that just finished vs the one that just started
             * -- see lap.h's lap_update comment. `to_line` is the
             * (unwrapped) lap-progress distance from `prev` to the line;
             * `travelled` is the total unwrapped distance covered this
             * step (to the line, plus from the line to `curr`). */
            f32 to_line = 1.0f - prev;
            f32 travelled = to_line + curr;
            f32 frac_before = (travelled > 1e-6f) ? (to_line / travelled) : 1.0f;
            f32 finished_lap_time = state->current_lap_time + dt * frac_before;

            state->last_lap_time = finished_lap_time;
#if DIRT2_INJECT_FAULT == 3
            /* Deliberately broken: overwrite unconditionally, ignoring
             * whether this lap actually improved on the existing best. */
            state->best_lap_time = finished_lap_time;
#else
            if (state->best_lap_time == LAP_NO_TIME ||
                finished_lap_time < state->best_lap_time) {
                state->best_lap_time = finished_lap_time;
            }
#endif
            state->lap_count++;
            state->current_lap_time = dt * (1.0f - frac_before);
            completed_lap = true;
        } else {
            /* Forward wrap without having reached the far side (e.g. the
             * "out and back" case: drive out past the line, reverse back
             * over it, then forward over it again -- an odd number of
             * crossings without ever leaving the near side). Not a lap;
             * the attempt is still open. */
            state->current_lap_time += dt;
        }
        state->far_side_reached = false;
    } else if (raw_delta > 0.5f) {
        /* Backward wrap: crossed the line in reverse. Never a lap. Does
         * NOT touch far_side_reached -- see lap.h's scope note on why a
         * backward crossing does not un-latch a far side already
         * genuinely visited earlier in this attempt. */
        state->current_lap_time += dt;
    } else {
        /* No wrap -- ordinary motion (either direction) away from the
         * line. */
        state->current_lap_time += dt;
        if (crosses_far_side(prev, curr)) {
            state->far_side_reached = true;
        }
    }

    state->prev_progress = curr;
    return completed_lap;
}
