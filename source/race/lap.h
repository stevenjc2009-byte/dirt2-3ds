/*---------------------------------------------------------------------------------
 * lap.h -- lap timing and counting on top of track.h's progress query.
 *
 * OWNER: race. This module has no idea what a car IS -- it takes a
 * world-space position and a dt each physics step (see lap_update) and
 * reasons entirely in terms of track.h's TrackQueryResult.progress.
 *
 * THE HARD PART: A LAP MUST ONLY COUNT WHEN THE CAR HAS ACTUALLY GONE ALL
 * THE WAY ROUND.
 *   track_query's progress is a snapshot spatial quantity: nearest point on
 *   the loop, expressed as a 0..1 fraction, wrapping at the start/finish
 *   line. A crossing test ALONE ("did progress just jump from ~1 to ~0
 *   between two steps?") cannot tell a genuine lap apart from the car
 *   sitting on the line and drifting back and forth across it, or driving
 *   forward over the line and immediately reversing back over it -- both
 *   produce the exact same crossing signature as a real lap.
 *
 *   This module gates a forward crossing on a SEPARATE fact tracked since
 *   the last crossing (or since lap_init/lap_notify_reset): has progress
 *   been observed on the FAR SIDE of the loop (the 0.5 midpoint, as far
 *   from the start/finish line as the track geometry allows)? Only a
 *   forward crossing that happened after genuinely visiting the far side
 *   counts as a completed lap. Reversing across the line, or a short
 *   out-and-back that never gets anywhere near the midpoint, never sets
 *   that flag, so neither can ever complete a lap -- see lap.c's
 *   lap_update for the exact state machine, and tests/test_race.c for the
 *   naive (crossing-test-only) counter this design exists to not be.
 *
 *   SCOPE NOTE: the far-side flag is a one-way latch per lap ATTEMPT --
 *   once set, only a forward crossing (counted or not) clears it, not a
 *   backward one. A car that genuinely drives most of the way around, then
 *   reverses ALL the way back past the near side again, then creeps
 *   forward across the line, would still be credited a lap without
 *   "re-earning" the far side on that final forward approach. That
 *   scenario is not one of this module's required checks (see
 *   tests/test_race.c) and is called out here rather than silently -- a
 *   future revision that wants to close it would track NET forward
 *   distance since the last crossing instead of a one-way flag.
 *
 * TIMING IS DT-DRIVEN, NEVER WALL-CLOCK.
 *   current_lap_time/last_lap_time/best_lap_time/total_time are all built
 *   by summing the `dt` passed to lap_update, matching every other
 *   subsystem in this project (see core/timestep.h) -- a render frame
 *   hitch cannot alter a recorded time because nothing here ever reads a
 *   clock.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RACE_LAP_H
#define DIRT2_RACE_LAP_H

#include "core/types.h"
#include "race/track.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sentinel for "no time recorded yet" -- a real lap time is always >= 0,
 * so callers check `== LAP_NO_TIME` rather than assuming any particular
 * value is impossible. */
#define LAP_NO_TIME (-1.0f)

typedef struct LapState {
    const Track *track;     /* borrowed, not owned -- caller keeps it alive
                              * for the LapState's whole lifetime           */
    int  cached_segment;    /* track_query's search-cache, see track.h     */

    bool primed;             /* false until the first lap_update call (or
                               * right after lap_notify_reset) -- that call
                               * seeds prev_progress instead of evaluating a
                               * delta against a previous sample that does
                               * not exist                                  */
    f32  prev_progress;      /* last step's track progress, 0..1           */
    bool far_side_reached;   /* see file header -- the gate on a forward
                               * crossing counting as a completed lap       */

    int  lap_count;          /* laps completed so far                      */
    f32  current_lap_time;   /* seconds, accumulated dt for the OPEN lap    */
    f32  last_lap_time;      /* seconds, LAP_NO_TIME until lap 1 finishes   */
    f32  best_lap_time;      /* seconds, LAP_NO_TIME until lap 1 finishes   */
    f32  total_time;         /* seconds, accumulated dt, never reset        */
} LapState;

/* Fresh LapState against `track` (borrowed, must outlive this LapState):
 * counters/times zeroed, last/best set to LAP_NO_TIME. Does not query a car
 * position itself -- the first lap_update call primes prev_progress (and,
 * per file header, the far-side latch if the car's very first position is
 * already on the far side -- i.e. starting mid-lap past the midpoint counts
 * driving forward to the line as completing a lap, matching how far the car
 * actually had to drive; starting before the midpoint requires the same
 * genuine trip through the far side as any other lap). */
void lap_init(LapState *state, const Track *track);

/* Call exactly once per physics step (PHYSICS_DT, see core/timestep.h),
 * after this step's car position is known. Accumulates `dt` into
 * current_lap_time and total_time, updates the track-progress state
 * machine, and on a genuinely completed lap: finalises last_lap_time,
 * updates best_lap_time if it improved, increments lap_count, and starts a
 * new current_lap_time. If the completion happens partway through THIS
 * step (the crossing falls between the previous and current position),
 * `dt` is split proportionally by how far past the crossing the new
 * position landed, so the finishing lap and the new one each get the
 * right fraction of this one step's time rather than the whole step being
 * assigned to one or the other.
 *
 * Returns true on the exact step a lap completed, false otherwise. */
bool lap_update(LapState *state, Vec3 car_world_pos, f32 dt);

/* Tells the LapState the car's position is about to become discontinuous
 * (a reset/respawn/replace -- NOT a drive), so the next lap_update call
 * re-primes prev_progress instead of comparing against a position from
 * before the jump, and clears the far-side latch (a teleport's "distance
 * travelled" is not real driving and must not itself be read as visiting
 * the far side, nor as a crossing in either direction).
 *
 * Deliberately does NOT touch lap_count, current_lap_time, last_lap_time,
 * best_lap_time or total_time -- a reset mid-lap keeps the clock and the
 * tally exactly as they were; that is this module's design choice (a
 * crashed/flipped car being placed back on track does not erase the laps
 * already banked), not an oversight. */
void lap_notify_reset(LapState *state);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RACE_LAP_H */
