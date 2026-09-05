/*---------------------------------------------------------------------------------
 * track.h -- a closed-loop track described as an ordered ring of waypoints
 * (centreline point + track half-width), and the one query every other race
 * module needs: "where is this world position relative to the track?"
 *
 * OWNER: race. This module knows nothing about a car, a lap counter, or
 * timing -- lap.h owns all of that, built entirely on top of track_query's
 * output. It also knows nothing about ground height or surface grip
 * (world/testground.h owns those) -- a track waypoint's Y is not read by any
 * query here; the world's own height query is the only source of truth for
 * "how high is the ground at this X/Z", so a track laid over a heightfield
 * never has two disagreeing sources for the same number.
 *
 * DATA-DRIVEN, NOT HARDCODED.
 *   track_init builds a Track from a caller-supplied waypoint array, so nothing
 *   in this module bakes in one specific track shape. track_build_example_oval
 *   is the ONE example loop this Phase ships, built by calling track_init like
 *   any other caller would -- a later version that loads waypoints from a file
 *   replaces just that one function, with zero change to track_init or
 *   track_query.
 *
 * PERFORMANCE (old 3DS, no NEON, this query runs every physics step).
 *   track_query does NOT scan every waypoint. It searches a small window
 *   outward from a caller-owned "last known segment" cache (see its own
 *   comment), falling back to a full scan only when the windowed search
 *   comes back implausibly far away (a stale cache after a car
 *   reset/teleport -- see lap.h's lap_notify_reset). The per-candidate
 *   distance test never calls sqrtf; only the final winning segment's
 *   distance is square-rooted, once.
 *
 * 2D IN THE XZ PLANE.
 *   Every query in this module ignores Y entirely (both the track's own
 *   waypoint.center.y and the queried world_pos.y) -- "where am I on the
 *   track" is a ground-plan question, matching how world/testground.h's own
 *   height zones are defined purely in X/Z. A future jump/airborne feature
 *   still resolves against the same 2D track shape; height is a separate
 *   axis entirely.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_RACE_TRACK_H
#define DIRT2_RACE_TRACK_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed capacity, no dynamic allocation -- matches this project's other
 * data structures (see e.g. VEHICLE_WHEEL_COUNT in core/types.h). 64 is
 * comfortably above the ~26 waypoints track_build_example_oval ships, with
 * room for a hand-authored or file-loaded track before this ever needs to
 * grow. */
#define TRACK_MAX_WAYPOINTS 64

/* One point on the track's centreline plus the track's half-width there.
 * Y is stored (so a caller can reuse Vec3 wholesale) but is NOT read by any
 * query in this module -- see file header. */
typedef struct TrackWaypoint {
    Vec3 center;
    f32  half_width;    /* metres either side of centre considered "on
                          * track" at this waypoint -- linearly interpolated
                          * along a segment by track_query. */
} TrackWaypoint;

/* A closed loop: waypoints[0..count) in driving order, with an implicit
 * closing segment from waypoints[count-1] back to waypoints[0]. Segment i
 * runs from waypoints[i] to waypoints[(i+1)%count].
 *
 * cumulative_distance[i] is the along-loop distance from waypoints[0] to
 * waypoints[i] (cumulative_distance[0] == 0, strictly ascending) -- track_init
 * computes this once so track_query never re-derives it. total_length is the
 * full loop length including the closing segment. */
typedef struct Track {
    TrackWaypoint waypoints[TRACK_MAX_WAYPOINTS];
    f32 cumulative_distance[TRACK_MAX_WAYPOINTS];
    int count;
    f32 total_length;
} Track;

/* Result of a single track_query call. */
typedef struct TrackQueryResult {
    int segment_index;   /* [0, track->count) -- nearest segment           */
    f32 progress;        /* 0..1, fraction of the way around the loop from
                           * waypoints[0], monotonic while driving forward,
                           * wraps 1.0 -> 0.0 exactly once per lap          */
    f32 lateral_offset;  /* signed metres from the centreline; positive is
                           * to the LEFT of the segment's driving direction
                           * (i.e. sign of cross(segment_dir, to_point) in
                           * the XZ plane) -- a fixed, documented convention,
                           * not one a caller needs to derive itself        */
    f32 half_width;      /* track half-width at the nearest point,
                           * interpolated between the segment's two
                           * waypoints -- lets a later off-course/cut check
                           * do fabsf(lateral_offset) > half_width          */
} TrackQueryResult;

/* Builds a closed loop from waypoints[0..count), copying them in (Track
 * owns its own storage, does not keep the caller's pointer) and computing
 * cumulative_distance/total_length once. `count` must be in [3,
 * TRACK_MAX_WAYPOINTS] -- fewer than 3 points cannot describe a loop with a
 * meaningful interior. */
void track_init(Track *track, const TrackWaypoint *waypoints, int count);

/* Ships one example closed loop that fits inside the world extents of
 * source/world/testground.h's placeholder config as set up by main.c's
 * make_placeholder_testground_config (X in [-25, 25], Z in [0, 140],
 * world_half_width 25 / flat+hills+washboard summing to 140) -- a ~8m-wide
 * stadium oval: two 96m straights at X = +-12, joined by two 12m-radius
 * hairpins, laid out with a minimum 6m clearance to every world edge on
 * every side (see track.c for the exact numbers and margin check). Built
 * entirely through track_init, same as any hand- or file-authored track
 * would be -- swapping this for a loaded track later is a one-function
 * change at the call site, not a change to this module's logic. */
void track_build_example_oval(Track *track);

/* Where is world_pos (X/Z only, see file header) relative to `track`?
 *
 * `*cached_segment` is caller-owned state (e.g. embedded in LapState, see
 * lap.h) that persists BETWEEN calls -- pass the value this function wrote
 * last time, and it searches outward from there instead of scanning every
 * segment. Any out-of-range value (including on the very first call, e.g.
 * 0) is accepted and clamped in. Always updated to the segment this call
 * actually found, ready to seed the next call. */
void track_query(const Track *track, Vec3 world_pos, int *cached_segment,
                  TrackQueryResult *out);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_RACE_TRACK_H */
