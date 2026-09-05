/*---------------------------------------------------------------------------------
 * track.c -- see track.h for the module contract. This file: waypoint
 * storage/setup (track_init), the one shipped example loop
 * (track_build_example_oval), and the nearest-segment query (track_query).
 *
 * FAULT INJECTION (see tests/test_race.c's header for the full scheme,
 * matching the convention already established by tests/test_physics.c):
 * building with -DDIRT2_INJECT_FAULT=5 deliberately breaks progress
 * computation (drops the divide-by-total_length normalisation) so
 * tests/test_race.c's monotonic-progress / wraps-once-per-lap check is
 * proven able to go red on real, non-stub code. Default (no -D) is 0, the
 * normal build -- every other value of this module's own logic is
 * untouched by the macro.
 *---------------------------------------------------------------------------------*/
#include "race/track.h"
#include "core/vecmath.h"

#include <math.h>

#ifndef DIRT2_INJECT_FAULT
#define DIRT2_INJECT_FAULT 0
#endif

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

/* How many segments either side of the cache to check before falling back
 * to a full scan -- see track.h's performance note. 6 either way covers a
 * 13-segment window, comfortably more than a car can cross in one physics
 * step at any speed this project's vehicle reaches, at any of the three
 * committed physics rates (60/100/120 Hz). */
#define TRACK_SEARCH_RADIUS 6

/* Squared point-to-segment distance in the XZ plane. Writes the clamped
 * projection fraction `t` (0..1 along a->b) and the 2D cross product of
 * (b-a) with (p-a) (its SIGN is track_query's lateral_offset sign
 * convention) -- never calls sqrtf itself so a caller comparing many
 * candidates only pays for one sqrt, on the eventual winner. */
static f32 point_segment_dist_sq_xz(Vec3 p, Vec3 a, Vec3 b,
                                     f32 *out_t, f32 *out_cross) {
    f32 abx = b.x - a.x;
    f32 abz = b.z - a.z;
    f32 apx = p.x - a.x;
    f32 apz = p.z - a.z;
    f32 ab_len_sq = abx * abx + abz * abz;
    f32 t;
    f32 closest_x, closest_z, dx, dz;

    if (ab_len_sq > 1e-8f) {
        t = (apx * abx + apz * abz) / ab_len_sq;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;
    } else {
        /* Degenerate (near-zero-length) segment -- clamp to the single
         * point `a` rather than dividing by ~zero. */
        t = 0.0f;
    }

    closest_x = a.x + abx * t;
    closest_z = a.z + abz * t;
    dx = p.x - closest_x;
    dz = p.z - closest_z;

    if (out_t) *out_t = t;
    if (out_cross) *out_cross = abx * apz - abz * apx;

    return dx * dx + dz * dz;
}

static f32 xz_distance(Vec3 a, Vec3 b) {
    f32 dx = a.x - b.x;
    f32 dz = a.z - b.z;
    return sqrtf(dx * dx + dz * dz);
}

void track_init(Track *track, const TrackWaypoint *waypoints, int count) {
    int i;

    if (count < 3) count = 3;
    if (count > TRACK_MAX_WAYPOINTS) count = TRACK_MAX_WAYPOINTS;

    track->count = count;
    for (i = 0; i < count; i++) {
        track->waypoints[i] = waypoints[i];
    }

    track->cumulative_distance[0] = 0.0f;
    for (i = 1; i < count; i++) {
        f32 seg_len = xz_distance(track->waypoints[i - 1].center,
                                   track->waypoints[i].center);
        track->cumulative_distance[i] = track->cumulative_distance[i - 1] + seg_len;
    }

    /* Closing segment, count-1 -> 0, folded into total_length but not into
     * cumulative_distance[] (which only ever indexes real waypoints). */
    track->total_length = track->cumulative_distance[count - 1] +
        xz_distance(track->waypoints[count - 1].center, track->waypoints[0].center);
}

void track_build_example_oval(Track *track) {
    /* See track.h's header comment for the world-extent margins this shape
     * is chosen against (world X in [-25,25], Z in [0,140]).
     *
     *   half_width   = 4m  (8m-wide track)
     *   turn_radius  = 12m (both hairpins)
     *   straight_x   = 12m (== turn_radius, so a straight's end lines up
     *                       exactly with the hairpin's tangent point)
     *   z_bottom/top = the two hairpin centres, 96m apart (the straights)
     *
     * Worst-case extent from the world origin is radius + half_width = 16m
     * in X (world bound 25m, 9m margin) and, at the hairpin apexes,
     * z_bottom_center - 16 = 6 (world bound 0, 6m margin) and
     * z_top_center + 16 = 134 (world bound 140, 6m margin). All margins
     * checked by hand against world/testground.h's zone lengths (flat 40 +
     * hills 60 + washboard 40 = 140) as configured by main.c's
     * make_placeholder_testground_config -- if that config ever changes,
     * these numbers need rechecking against it. */
    const f32 half_width = 4.0f;
    const f32 turn_radius = 12.0f;
    const f32 straight_x = turn_radius;
    const f32 z_bottom_center = 22.0f;
    const f32 z_top_center = 118.0f;
    const int arc_segments = 12; /* subdivisions per 180-degree hairpin */

    TrackWaypoint wp[TRACK_MAX_WAYPOINTS];
    int n = 0;
    int i;

    /* Left straight, driving south -> north (+Z), X = -straight_x. */
    wp[n].center = vec3_make(-straight_x, 0.0f, z_bottom_center);
    wp[n].half_width = half_width;
    n++;
    wp[n].center = vec3_make(-straight_x, 0.0f, z_top_center);
    wp[n].half_width = half_width;
    n++;

    /* Top hairpin: centre (0, z_top_center), sweeping PI -> 0 (heading
     * flips from +Z to -Z). i runs 1..arc_segments inclusive -- the final
     * point (angle 0) IS the right straight's north end, added here rather
     * than duplicated below. */
    for (i = 1; i <= arc_segments; i++) {
        f32 t = (f32)i / (f32)arc_segments;
        f32 angle = PI_F - t * PI_F;
        wp[n].center = vec3_make(cosf(angle) * turn_radius, 0.0f,
                                  z_top_center + sinf(angle) * turn_radius);
        wp[n].half_width = half_width;
        n++;
    }

    /* Right straight, driving north -> south, X = +straight_x. Only the
     * south end is new -- the north end was the hairpin loop's last point
     * above. */
    wp[n].center = vec3_make(straight_x, 0.0f, z_bottom_center);
    wp[n].half_width = half_width;
    n++;

    /* Bottom hairpin: centre (0, z_bottom_center), sweeping 0 -> -PI. i
     * stops at arc_segments-1 (NOT arc_segments): angle -PI would land
     * exactly back on waypoint 0, (-straight_x, z_bottom_center) -- track_init
     * already closes the loop with an implicit segment from the last
     * waypoint back to waypoint 0, so emitting that point here would be a
     * duplicate. Leaving it out means the closing segment covers exactly
     * one more arc_segments-th of the hairpin, same angular size as every
     * other step in this loop. */
    for (i = 1; i <= arc_segments - 1; i++) {
        f32 t = (f32)i / (f32)arc_segments;
        f32 angle = -t * PI_F;
        wp[n].center = vec3_make(cosf(angle) * turn_radius, 0.0f,
                                  z_bottom_center + sinf(angle) * turn_radius);
        wp[n].half_width = half_width;
        n++;
    }

    track_init(track, wp, n);
}

void track_query(const Track *track, Vec3 world_pos, int *cached_segment,
                  TrackQueryResult *out) {
    int count = track->count;
    int start = *cached_segment;
    int best_seg = 0;
    f32 best_dist_sq = -1.0f;
    f32 best_t = 0.0f;
    f32 best_cross = 0.0f;
    int radius;
    f32 fallback_limit;
    f32 seg_len, dist_along_lap, progress, dist, hw_a, hw_b;

    if (start < 0 || start >= count) start = 0;

    for (radius = 0; radius <= TRACK_SEARCH_RADIUS; radius++) {
        int n_offsets = (radius == 0) ? 1 : 2;
        int oi;
        for (oi = 0; oi < n_offsets; oi++) {
            int offset = (oi == 0) ? radius : -radius;
            int seg = ((start + offset) % count + count) % count;
            Vec3 a = track->waypoints[seg].center;
            Vec3 b = track->waypoints[(seg + 1) % count].center;
            f32 t, cross;
            f32 d2 = point_segment_dist_sq_xz(world_pos, a, b, &t, &cross);
            if (best_dist_sq < 0.0f || d2 < best_dist_sq) {
                best_dist_sq = d2;
                best_seg = seg;
                best_t = t;
                best_cross = cross;
            }
        }
    }

    /* Fallback full scan: only reached when the windowed search's best
     * candidate is farther away than a quarter of the whole loop's length
     * -- implausible for a car actually on or near the track, and the
     * signature of a stale cache after a position discontinuity (car
     * reset/teleport, see lap.h's lap_notify_reset). Self-scaling to the
     * track's own size rather than a fixed metre constant, so this stays
     * correct for a track of any size, not just the shipped example oval. */
    fallback_limit = track->total_length * 0.25f;
    if (best_dist_sq > fallback_limit * fallback_limit) {
        int seg;
        for (seg = 0; seg < count; seg++) {
            Vec3 a = track->waypoints[seg].center;
            Vec3 b = track->waypoints[(seg + 1) % count].center;
            f32 t, cross;
            f32 d2 = point_segment_dist_sq_xz(world_pos, a, b, &t, &cross);
            if (d2 < best_dist_sq) {
                best_dist_sq = d2;
                best_seg = seg;
                best_t = t;
                best_cross = cross;
            }
        }
    }

    *cached_segment = best_seg;

    seg_len = (best_seg == count - 1)
        ? (track->total_length - track->cumulative_distance[count - 1])
        : (track->cumulative_distance[best_seg + 1] - track->cumulative_distance[best_seg]);
    dist_along_lap = track->cumulative_distance[best_seg] + best_t * seg_len;

#if DIRT2_INJECT_FAULT == 5
    /* Deliberately broken: skip the divide-by-total_length normalisation,
     * so progress is a raw metre distance instead of a 0..1 fraction --
     * proves tests/test_race.c's monotonic/wraps-once-per-lap check can
     * fail on real code, not just an absent implementation. */
    progress = dist_along_lap;
#else
    progress = dist_along_lap / track->total_length;
#endif
    if (progress >= 1.0f) progress -= 1.0f;
    if (progress < 0.0f) progress += 1.0f;

    dist = sqrtf(best_dist_sq);
    hw_a = track->waypoints[best_seg].half_width;
    hw_b = track->waypoints[(best_seg + 1) % count].half_width;

    out->segment_index = best_seg;
    out->progress = progress;
    out->lateral_offset = (best_cross >= 0.0f) ? dist : -dist;
    out->half_width = hw_a + (hw_b - hw_a) * best_t;
}
