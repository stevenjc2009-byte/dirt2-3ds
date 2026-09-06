/*---------------------------------------------------------------------------------
 * barrier.c -- see barrier.h for the module contract and design rationale.
 *
 * LATERAL DIRECTION MUST MATCH track_query's SIGN CONVENTION EXACTLY.
 *   track.c's point_segment_dist_sq_xz defines lateral_offset's sign as the
 *   sign of cross(segment_dir, to_point) in the XZ plane (positive = LEFT of
 *   the segment's driving direction -- see track.h's TrackQueryResult
 *   comment). Given a normalised segment direction d = (dx, 0, dz), the unit
 *   vector that INCREASES lateral_offset when you move a point along it is
 *   left = (-dz, 0, dx):
 *
 *       cross(d, left) = d.x*left.z - d.z*left.x
 *                       = dx*dx - dz*(-dz) = dx^2 + dz^2 > 0
 *
 *   i.e. always positive (for any non-zero d), matching "left" by track.c's
 *   own convention regardless of which way the segment happens to be
 *   pointing -- straight or hairpin, this derivation does not special-case
 *   either. Getting the rotation backwards doesn't just fail a test -- it
 *   pushes an off-track car FURTHER off the track on one side while
 *   over-correcting it on the other, which is exactly what
 *   DIRT2_INJECT_FAULT == 10 below exists to catch.
 *
 * FAULT INJECTION (see tests/test_race.c's header for the full scheme this
 * project uses). This module picks fault numbers 10-13, clear of track.c's 5
 * and lap.c's 1/2/3/4/6, since a test binary can link this file alongside
 * either of theirs in the same build. Building with -DDIRT2_INJECT_FAULT=<N>
 * deliberately breaks ONE specific piece of real logic below so a specific
 * check in tests/test_barrier.c is proven able to fail on real code, not
 * just an absent implementation:
 *   10 -- flips the lateral direction's sign (see note above)
 *   11 -- also zeroes the tangential (along-track) velocity component,
 *         which barrier_apply must never touch
 *   12 -- drops the restitution factor, fully cancelling the outward
 *         velocity component instead of bouncing it back at reduced speed
 *   13 -- skips the position correction, only fixing velocity
 * Default (no -D) is 0, the normal build.
 *---------------------------------------------------------------------------------*/
#include "race/barrier.h"
#include "core/vecmath.h"

#include <math.h>
#include <stddef.h>

#ifndef DIRT2_INJECT_FAULT
#define DIRT2_INJECT_FAULT 0
#endif

void barrier_apply(const Track *track, RigidBody *body, int *cached_segment) {
    TrackQueryResult q;
    Vec3 a, b, raw_dir, seg_dir, left_dir, outward_dir;
    f32 limit, v_outward;
    int count, seg;

    if (track == NULL || body == NULL || cached_segment == NULL) return;

    count = track->count;
    if (count < 3) return; /* not a valid closed loop -- see track_init */

    /* A non-finite position has no meaningful lateral direction to correct
     * along, and feeding it into track_query would poison lateral_offset
     * with NaN (NaN compares false against everything, including the
     * "< 0.0f" bootstrap check point_segment_dist_sq_xz's caller relies on,
     * so the first scanned candidate would silently "win" regardless of its
     * own value) -- bail before any of that happens. */
    if (!isfinite(body->position.x) || !isfinite(body->position.y) ||
        !isfinite(body->position.z)) {
        return;
    }

    track_query(track, body->position, cached_segment, &q);

    seg = q.segment_index;
    if (seg < 0 || seg >= count) return;
    if (!isfinite(q.lateral_offset) || !isfinite(q.half_width)) return;

    limit = q.half_width + BARRIER_RUNOFF_M - BARRIER_INSET_M;
    if (limit < 0.0f) limit = 0.0f; /* a track authored narrower than the
                                      * inset -- clamp to the centreline
                                      * rather than let the limit itself land
                                      * on the wrong side of zero */

    if (fabsf(q.lateral_offset) <= limit) return; /* common case: free */

    /* Segment direction in the XZ plane, matching track_query's own 2D
     * treatment (see track.h's file header) -- Y is never part of this
     * module's geometry. */
    a = track->waypoints[seg].center;
    b = track->waypoints[(seg + 1) % count].center;
    raw_dir = vec3_make(b.x - a.x, 0.0f, b.z - a.z);
    seg_dir = vec3_normalize(raw_dir);
    if (seg_dir.x == 0.0f && seg_dir.z == 0.0f) {
        /* Degenerate zero-length segment (two waypoints at the same XZ
         * point) -- vec3_normalize's documented zero-vector fallback (see
         * vecmath.h) makes this exact comparison safe. There is no lateral
         * direction to correct along, so do nothing. */
        return;
    }

    /* See this file's header comment for why this specific rotation of
     * seg_dir is the one that matches track_query's lateral_offset sign. */
    left_dir = vec3_make(-seg_dir.z, 0.0f, seg_dir.x);
#if DIRT2_INJECT_FAULT == 10
    left_dir = vec3_negate(left_dir);
#endif

    /* Move position back along the lateral axis to sit EXACTLY at the limit
     * on whichever side the car was already on -- never across to the
     * other side, and only ever inward (target_lateral has the same sign as
     * q.lateral_offset and a strictly smaller magnitude, since this branch
     * only runs when |q.lateral_offset| > limit). Scoped to its own block so
     * a build with fault 13 (which skips this entirely) never declares an
     * unused variable. */
#if DIRT2_INJECT_FAULT != 13
    {
        f32 target_lateral = (q.lateral_offset >= 0.0f) ? limit : -limit;
        Vec3 correction = vec3_scale(left_dir, target_lateral - q.lateral_offset);
        body->position.x += correction.x;
        body->position.z += correction.z;
        /* position.y untouched -- the wall is a vertical plane, not a floor. */
    }
#endif

    /* Outward = whichever side of the centreline the car is actually on --
     * the direction further motion would make things worse, and the
     * direction the restitution bounce pushes AWAY from. */
    outward_dir = (q.lateral_offset >= 0.0f) ? left_dir : vec3_negate(left_dir);
    v_outward = vec3_dot(body->linear_velocity, outward_dir);

    if (v_outward > 0.0f) {
        /* Standard reflect-with-restitution: subtracting (1+e) times the
         * outward component leaves exactly -e times it, i.e. a bounce back
         * toward the track at a fraction of the impact speed. v_outward <= 0
         * (already checked above) means the car is coasting or already
         * heading back inward on its own -- nothing to remove. */
        f32 removal;
#if DIRT2_INJECT_FAULT == 12
        removal = v_outward; /* deliberately broken: drops the restitution
                               * factor, fully cancelling instead of
                               * bouncing back */
#else
        removal = (1.0f + BARRIER_RESTITUTION_FACTOR) * v_outward;
#endif
        body->linear_velocity.x -= outward_dir.x * removal;
        body->linear_velocity.z -= outward_dir.z * removal;
        /* linear_velocity.y and the tangential (seg_dir) component are
         * never touched here -- see file header. */
#if DIRT2_INJECT_FAULT == 11
        {
            /* Deliberately broken: also strips the along-track component,
             * which barrier_apply must never touch (see barrier.h) -- a
             * light brush should cost a little speed through the wall's
             * normal only, not turn into a full stop. */
            f32 v_tangent = vec3_dot(body->linear_velocity, seg_dir);
            body->linear_velocity.x -= seg_dir.x * v_tangent;
            body->linear_velocity.z -= seg_dir.z * v_tangent;
        }
#endif
    }
}
