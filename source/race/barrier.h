/*---------------------------------------------------------------------------------
 * barrier.h -- an invisible wall along each side of the track, keeping the
 * circuit physically inescapable.
 *
 * OWNER: race. Built entirely on top of track.h's track_query -- the exact
 * same "where is this world position relative to the track" query lap.h
 * already relies on, so containment costs no new track representation. Like
 * lap.h, this module has no idea what a car IS beyond a RigidBody's position
 * and velocity; it never reads or writes orientation or angular_velocity,
 * and does not depend on vehicle.h at all.
 *
 * WHY A WALL, NOT A SOFTER FORCE FIELD.
 *   steve chose "invisible walls along the track edge" as the circuit's
 *   containment. Today, driving off the track's ribbon means leaving
 *   world/testground.h's height zones entirely -- its height query returns
 *   false out there, and the car falls forever with no way back, because
 *   there is currently no floor and no wall past the edge. A hard positional
 *   clamp plus a velocity bounce is the simplest thing that actually stops
 *   that, deriving the containment boundary from the track's own half_width
 *   (see BARRIER_RUNOFF_M for the offset, and why that offset is not zero)
 *   instead of introducing new collision geometry.
 *
 * WHY THE COMMON CASE MUST BE A TRUE NO-OP.
 *   barrier_apply runs once per physics step for the car, and almost every
 *   step the car is nowhere near the edge. If it touched position or
 *   linear_velocity even when well inside the limit -- even a write of the
 *   same value back, even a zero-length correction -- a rounding wobble on
 *   that path would show up on real hardware as a permanent, invisible
 *   jitter nobody could diagnose from the outside. See barrier_apply's
 *   early-out, and tests/test_barrier.c's bit-for-bit "well inside" check.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RACE_BARRIER_H
#define DIRT2_RACE_BARRIER_H

#include "core/types.h"
#include "core/rigidbody.h"
#include "race/track.h"

#ifdef __cplusplus
extern "C" {
#endif

/* How far OUTSIDE the painted ribbon's edge the wall stands, in metres.
 *
 * WHY THE WALL IS NOT ON THE RIBBON EDGE. The first version of this module
 * put the wall at half_width minus an inset, i.e. just INSIDE the painted
 * edge. That contains the car -- but it also makes it physically impossible
 * to ever leave the ribbon, and main.c's tyre surface lookup gives the car
 * grippy tarmac on the ribbon and loose gravel off it. A wall at or inside
 * the ribbon edge would mean the gravel branch could never execute: a grip
 * penalty that exists in the source, passes its tests, and can never be
 * reached in the game. That is the same shape of defect as a feature landing
 * unreachable, and it is worse than having no wall at all, because the code
 * reads as if the penalty works.
 *
 * So the layout is three bands, measured from the centreline outward:
 *   0 .. half_width                 painted ribbon, tarmac grip
 *   half_width .. half_width + 4m   gravel run-off, real grip penalty
 *   half_width + 4m                 the wall
 * Running wide now COSTS something -- the car goes loose and slow and has to
 * be gathered up -- without ending the run. 4m is roughly a car length and a
 * half: wide enough that a corner overshoot lands on gravel and is felt,
 * narrow enough that a big one still finds the wall rather than the void. */
#define BARRIER_RUNOFF_M (4.0f)

/* How much nearer than the wall plane the car's CENTRE is stopped, in
 * metres. The barrier limit is applied to `body->position`, which is a
 * point; treating the wall plane itself as that point's limit means half
 * the car's body is already through the wall by the time anything stops it.
 * This module does not depend on the vehicle module (see file header), so
 * this is a fixed estimate of half a rally car's body width rather than a
 * value read from vehicle_params.h.
 *
 * Net drivable limit is therefore half_width + BARRIER_RUNOFF_M -
 * BARRIER_INSET_M. On track_build_example_oval (half_width 5.0m) that is
 * 5 + 4 - 0.6 = 8.4m from the centreline. Because the run-off term is far
 * larger than the inset, the limit stays comfortably positive on any
 * sanely-authored track; barrier_apply still clamps it at zero for the
 * pathological case (see its clamp). */
#define BARRIER_INSET_M (0.6f)

/* How much of the OUTWARD velocity component survives a wall hit, as a
 * fraction, reversed in direction -- e.g. hitting the wall at 5 m/s outward
 * leaves 1.25 m/s back toward the track at this default. Small enough that
 * scrubbing the wall feels like contact with something solid, not a
 * trampoline; big enough that the car doesn't just stick dead to the wall
 * either. Tangential (along-track) speed is never touched by this factor --
 * see barrier_apply's contract below. */
#define BARRIER_RESTITUTION_FACTOR (0.25f)

/* Called once per physics step, AFTER vehicle_step has integrated `body`
 * for this step (see rigidbody.h's rigidbody_step) -- this function reads
 * and corrects the RESULT of that integration; it does not run inside it.
 *
 * Queries `track` at `body->position` (via track_query, using and updating
 * `*cached_segment` exactly as lap_update does -- see track.h's caching
 * contract, including when to pass TRACK_UNKNOWN_SEGMENT).
 *
 * If |lateral_offset| <= the segment's half_width plus BARRIER_RUNOFF_M
 * minus BARRIER_INSET_M -- i.e. anywhere on the ribbon OR out on the gravel
 * run-off beyond it -- this function does nothing at all: no read of
 * linear_velocity's
 * components, no write to position or linear_velocity, not even a
 * zero-length one -- see file header on why the common case must be a true
 * no-op.
 *
 * If the car is beyond that limit, on either side:
 *   - `position` is moved along the segment's lateral (perpendicular-to-
 *     travel) direction, in the XZ plane only, so the car sits EXACTLY at
 *     the limit on the side it was already on -- never moved past the
 *     limit, never moved to the opposite side.
 *   - the OUTWARD component of `linear_velocity` (the part driving the car
 *     further past the limit) is replaced by that same component negated
 *     and scaled by BARRIER_RESTITUTION_FACTOR. Any component already
 *     heading back inward is left alone -- there is nothing to correct, and
 *     reflecting it anyway would inject energy instead of removing it.
 *   - the along-track (tangential) component of `linear_velocity`,
 *     `position.y`, and vertical velocity are never read for correction and
 *     never written -- the wall is a vertical plane parallel to the track's
 *     direction of travel at that point, not a ceiling, a floor, or a speed
 *     trap. Scrubbing a wall costs a little speed through the wall's normal
 *     only.
 *
 * Does nothing (no read beyond the initial finiteness/range checks, no
 * write) if `track`, `body` or `cached_segment` is NULL, if `track` has
 * fewer than 3 waypoints (cannot describe a loop -- see track_init), if
 * `body->position` is not finite (NaN/Inf in any component -- there is no
 * sensible lateral direction to correct along), if track_query's result
 * does not name a valid segment or returns a non-finite lateral_offset/
 * half_width, or if the nearest segment is degenerate (zero XZ length,
 * i.e. two coincident waypoints -- no lateral direction exists to correct
 * along). Each of these is a state some OTHER system broke; this
 * function's job is to never turn a bad input into a worse one. */
void barrier_apply(const Track *track, RigidBody *body, int *cached_segment);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RACE_BARRIER_H */
