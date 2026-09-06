/*---------------------------------------------------------------------------------
 * test_barrier.c -- host-side test suite for source/race/barrier.c (the
 * invisible-wall track containment).
 *
 * RATE INDEPENDENCE.
 *   The "repeated wall contact" test below expresses its run length as "N
 *   simulated seconds" via PHYSICS_HZ (core/timestep.h), converted to a step
 *   count, never as a raw step count -- matching tests/test_race.c's own
 *   convention, so the same source built at PHYSICS_HZ 60/100/120 exercises
 *   the same real-time window at each rate.
 *
 * INDEPENDENT GROUND TRUTH, NOT A MIRROR OF barrier.c's OWN MATH.
 *   Wherever a check needs to know "which side" or "how far" a corrected
 *   position ended up, it gets that answer from a FRESH, independent call to
 *   track_query (already covered by tests/test_race.c) rather than by
 *   re-deriving barrier.c's left_dir formula here -- a sign bug shared by
 *   both would otherwise cancel out and the test would pass on broken code.
 *   The one place this file DOES read Track waypoint data directly (to build
 *   a "push this far outward" probe point) is construction, never the
 *   pass/fail judgement -- see build_lateral_probe.
 *
 * FAULT INJECTION (see tests/test_race.c's header for the full scheme).
 * source/race/barrier.c carries `#if DIRT2_INJECT_FAULT == N` blocks
 * (N = 10..13, see that file's own header) that deliberately break one real
 * piece of its logic when built with -DDIRT2_INJECT_FAULT=N. This file
 * requires no changes to run under fault injection. Default (no -D) is 0,
 * the normal, all-passing build.
 *---------------------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "core/types.h"
#include "core/vecmath.h"
#include "core/rigidbody.h"
#include "core/timestep.h"
#include "race/track.h"
#include "race/barrier.h"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) \
    do { \
        g_checks++; \
        if (!(cond)) { \
            g_failures++; \
            fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
        } \
    } while (0)

static int nearly_equal_f32(f32 a, f32 b, f32 abs_tol, f32 rel_tol) {
    f32 diff = fabsf(a - b);
    if (diff <= abs_tol) return 1;
    f32 scale = fabsf(a) > fabsf(b) ? fabsf(a) : fabsf(b);
    return diff <= rel_tol * scale;
}

/* ---- test-only helper: a RigidBody with an arbitrary but harmless mass/
 * inertia, at a given position and velocity, orientation left at identity.
 * Mass/inertia are never exercised by barrier_apply (it only reads/writes
 * position and linear_velocity), so any positive, finite values do. ---- */
static RigidBody make_body(Vec3 position, Vec3 velocity) {
    RigidBody rb;
    rigidbody_init(&rb, 1000.0f, vec3_make(400.0f, 600.0f, 500.0f));
    rb.position = position;
    rb.linear_velocity = velocity;
    return rb;
}

/* ---- test-only helper, copied in spirit from tests/test_race.c's
 * sample_track_position: an exact centreline point at a given lap fraction,
 * derived from Track's own waypoint/cumulative_distance data, independent of
 * track_query (the thing barrier_apply calls internally). `lap_fraction` may
 * be any real number, wrapped into [0, 1) before sampling. ---- */
static Vec3 sample_track_position(const Track *track, f32 lap_fraction) {
    f32 frac = lap_fraction - floorf(lap_fraction);
    f32 target_dist = frac * track->total_length;
    int i;

    for (i = 0; i < track->count; i++) {
        f32 seg_start = track->cumulative_distance[i];
        f32 seg_len = (i == track->count - 1)
            ? (track->total_length - track->cumulative_distance[i])
            : (track->cumulative_distance[i + 1] - track->cumulative_distance[i]);
        if (i == track->count - 1 || target_dist <= seg_start + seg_len) {
            f32 t = (seg_len > 1e-6f) ? (target_dist - seg_start) / seg_len : 0.0f;
            Vec3 a, b;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            a = track->waypoints[i].center;
            b = track->waypoints[(i + 1) % track->count].center;
            return vec3_lerp(a, b, t);
        }
    }
    return track->waypoints[0].center;
}

/* ---- test-only helper: builds a probe position `distance` metres away from
 * `centerline_point` along the LOCAL lateral axis at that point (derived
 * fresh from two adjacent Track waypoints -- ground truth about the track's
 * SHAPE, not a call into barrier.c). `sign` selects which of the two lateral
 * directions (+1 or -1); which one of those track_query will itself call
 * "left" is deliberately NOT assumed here -- callers read that back from a
 * track_query call on the resulting point instead (see test bodies below),
 * so this helper cannot hide a sign bug shared with barrier.c. Returns false
 * (leaves *out untouched) only if the nearest segment at centerline_point is
 * degenerate, which none of this file's probe points are. ---- */
static int build_lateral_probe(const Track *track, Vec3 centerline_point,
                                f32 distance, f32 sign, Vec3 *out) {
    TrackQueryResult q;
    int cache = TRACK_UNKNOWN_SEGMENT;
    Vec3 a, b, seg_dir, left_dir;

    track_query(track, centerline_point, &cache, &q);
    a = track->waypoints[q.segment_index].center;
    b = track->waypoints[(q.segment_index + 1) % track->count].center;
    seg_dir = vec3_normalize(vec3_make(b.x - a.x, 0.0f, b.z - a.z));
    if (seg_dir.x == 0.0f && seg_dir.z == 0.0f) return 0;

    left_dir = vec3_make(-seg_dir.z, 0.0f, seg_dir.x);
    *out = vec3_add(centerline_point, vec3_scale(left_dir, distance * sign));
    return 1;
}

static f32 limit_at(const Track *track, Vec3 point) {
    TrackQueryResult q;
    int cache = TRACK_UNKNOWN_SEGMENT;
    f32 limit;
    track_query(track, point, &cache, &q);
    limit = q.half_width + BARRIER_RUNOFF_M - BARRIER_INSET_M;
    return limit > 0.0f ? limit : 0.0f;
}

/* ---- 0. THE RUN-OFF MUST EXIST. ----
 *
 * Every other check in this file derives its expected limit from limit_at()
 * above, which is the same formula barrier.c uses. That makes them all
 * RELATIVE: they prove the car is stopped exactly at whatever the limit is,
 * and would stay green if the wall were moved anywhere at all -- including
 * back onto the ribbon edge, which is where the first version of this module
 * actually put it.
 *
 * That placement is not a matter of taste. main.c gives the car tarmac grip
 * inside half_width and gravel grip outside it. If the wall sits at or inside
 * half_width, the car can never reach gravel, and that entire grip penalty
 * becomes unreachable code that still compiles and still passes its own
 * tests. This check is the only one here that can catch that, so it asserts
 * the ABSOLUTE relationship the design depends on, on every segment of the
 * real shipped track rather than on one sampled point. */
static void test_wall_stands_outside_the_ribbon(const Track *track) {
    int i;
    f32 worst_margin = 1.0e9f;

    for (i = 0; i < track->count; ++i) {
        f32 half_width = track->waypoints[i].half_width;
        f32 limit = half_width + BARRIER_RUNOFF_M - BARRIER_INSET_M;
        f32 margin = limit - half_width;
        if (margin < worst_margin) worst_margin = margin;
    }

    /* Strictly greater than zero, not >=: a wall exactly on the ribbon edge
     * leaves a run-off of zero width, which is the same unreachable-gravel
     * defect with a friendlier-looking number. */
    CHECK(worst_margin > 0.0f,
          "the wall must stand strictly OUTSIDE the ribbon on every segment, or "
          "the off-track grip penalty is unreachable code");

    /* And it must be wide enough to actually be driven on and felt, not a
     * technicality. Half a car length is the floor. */
    CHECK(worst_margin >= 2.0f,
          "the gravel run-off must be at least 2m wide, or running wide is not a "
          "penalty the driver can feel before the wall catches them");

    /* The behavioural half of the same claim: a car actually OUT on the
     * gravel -- past the painted edge, short of the wall -- must be left
     * completely alone. If barrier_apply corrected it, the run-off measured
     * above would exist on paper and not in the game. */
    {
        Vec3 center = sample_track_position(track, 0.1f);
        TrackQueryResult q;
        int probe_cache = TRACK_UNKNOWN_SEGMENT;
        Vec3 on_gravel;
        f32 sign;

        track_query(track, center, &probe_cache, &q);

        for (sign = -1.0f; sign <= 1.0f; sign += 2.0f) {
            /* Half a metre outside the painted edge: unambiguously on gravel,
             * and far short of the wall. */
            if (build_lateral_probe(track, center, q.half_width + 0.5f, sign,
                                    &on_gravel)) {
                RigidBody before = make_body(on_gravel, vec3_make(4.0f, 0.0f, 9.0f));
                RigidBody after = before;
                int cache = TRACK_UNKNOWN_SEGMENT;

                barrier_apply(track, &after, &cache);

                CHECK(memcmp(&before.position, &after.position, sizeof(Vec3)) == 0,
                      "a car out on the gravel run-off must not be shoved back onto "
                      "the ribbon by the barrier");
                CHECK(memcmp(&before.linear_velocity, &after.linear_velocity,
                             sizeof(Vec3)) == 0,
                      "a car out on the gravel run-off must not have its velocity "
                      "touched by the barrier");
            }
        }
    }
}

/* ---- 1. well inside the track: barrier_apply must not touch anything,
 * bit-for-bit, not even a zero-length write. ---- */
static void test_no_op_well_inside(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f);
    RigidBody before = make_body(center, vec3_make(3.0f, -1.0f, 12.0f));
    RigidBody after = before;
    int cache = TRACK_UNKNOWN_SEGMENT;

    barrier_apply(track, &after, &cache);

    CHECK(memcmp(&before.position, &after.position, sizeof(Vec3)) == 0,
          "a car well inside the track must not have its position touched at all");
    CHECK(memcmp(&before.linear_velocity, &after.linear_velocity, sizeof(Vec3)) == 0,
          "a car well inside the track must not have its velocity touched at all");
    CHECK(memcmp(&before.orientation, &after.orientation, sizeof(Quat)) == 0,
          "barrier_apply must never touch orientation");
    CHECK(memcmp(&before.angular_velocity, &after.angular_velocity, sizeof(Vec3)) == 0,
          "barrier_apply must never touch angular_velocity");
}

/* ---- 2. exactly at (just inside) the limit: a no-op, not a jitter. ---- */
static void test_no_op_at_limit_boundary(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f);
    f32 limit = limit_at(track, center);
    Vec3 probe;
    RigidBody before, after;
    int cache = TRACK_UNKNOWN_SEGMENT;

    CHECK(build_lateral_probe(track, center, limit - 1e-3f, 1.0f, &probe),
          "sanity: the left straight's segment must have a real lateral direction");

    before = make_body(probe, vec3_make(2.0f, 0.0f, 15.0f));
    after = before;
    barrier_apply(track, &after, &cache);

    CHECK(memcmp(&before.position, &after.position, sizeof(Vec3)) == 0,
          "a car just inside the limit must not be moved");
    CHECK(memcmp(&before.linear_velocity, &after.linear_velocity, sizeof(Vec3)) == 0,
          "a car just inside the limit must not have its velocity changed");
}

/* ---- 3/4. a car pushed past either edge is clamped to EXACTLY the limit,
 * on the SAME side it was already on -- checked against a fresh, independent
 * track_query call, not against barrier.c's own arithmetic. Run against
 * both a straight and both hairpins (requirement: curvature must not flip
 * the sign), covering "both hairpins and both straights" together with
 * test_left_right_on_straight below. ---- */
static void check_clamped_to_same_side(const Track *track, Vec3 centerline_point,
                                        f32 sign, const char *where) {
    f32 limit = limit_at(track, centerline_point);
    Vec3 probe;
    RigidBody body;
    int cache = TRACK_UNKNOWN_SEGMENT;
    TrackQueryResult q_before, q_after;
    char msg[192];

    if (!build_lateral_probe(track, centerline_point, limit + 2.0f, sign, &probe)) {
        CHECK(0, "sanity: probe construction must succeed for a real track segment");
        return;
    }

    /* Ground truth for "which side" and "how far", from track_query itself,
     * BEFORE barrier_apply runs. */
    track_query(track, probe, &cache, &q_before);
    CHECK(fabsf(q_before.lateral_offset) > limit, "sanity: the probe must actually be beyond the limit");

    cache = TRACK_UNKNOWN_SEGMENT;
    body = make_body(probe, vec3_zero());
    barrier_apply(track, &body, &cache);

    cache = TRACK_UNKNOWN_SEGMENT;
    track_query(track, body.position, &cache, &q_after);

    snprintf(msg, sizeof(msg), "%s: corrected position must land on the SAME side as before (sign preserved)", where);
    CHECK((q_before.lateral_offset >= 0.0f) == (q_after.lateral_offset >= 0.0f), msg);

    snprintf(msg, sizeof(msg), "%s: corrected position must sit at exactly the limit, not short of it or past it", where);
    CHECK(nearly_equal_f32(fabsf(q_after.lateral_offset), limit, 0.01f, 0.0f), msg);

    snprintf(msg, sizeof(msg), "%s: barrier_apply must never touch position.y", where);
    CHECK(body.position.y == probe.y, msg);
}

static void test_left_right_on_straight(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f); /* mid left straight */
    check_clamped_to_same_side(track, center, +1.0f, "left straight, +side");
    check_clamped_to_same_side(track, center, -1.0f, "left straight, -side");
}

static void test_both_straights_and_both_hairpins(const Track *track) {
    /* Rough centreline fractions for each zone of track_build_example_oval's
     * layout (left straight, top hairpin, right straight, bottom hairpin, in
     * driving order) -- see track.c's own comment for the shape. Each pick
     * is well clear of the zone boundaries computed from the straights'
     * combined ~192m and the two ~37.7m hairpin arcs out of a ~267m loop. */
    check_clamped_to_same_side(track, sample_track_position(track, 0.15f), +1.0f, "left straight");
    check_clamped_to_same_side(track, sample_track_position(track, 0.15f), -1.0f, "left straight");
    check_clamped_to_same_side(track, sample_track_position(track, 0.43f), +1.0f, "top hairpin");
    check_clamped_to_same_side(track, sample_track_position(track, 0.43f), -1.0f, "top hairpin");
    check_clamped_to_same_side(track, sample_track_position(track, 0.68f), +1.0f, "right straight");
    check_clamped_to_same_side(track, sample_track_position(track, 0.68f), -1.0f, "right straight");
    check_clamped_to_same_side(track, sample_track_position(track, 0.93f), +1.0f, "bottom hairpin");
    check_clamped_to_same_side(track, sample_track_position(track, 0.93f), -1.0f, "bottom hairpin");
}

/* ---- 5. velocity: outward component reduced+reversed by the restitution
 * factor, tangential component untouched. Uses the left straight, where the
 * track's own construction (see track.c) makes the segment direction exactly
 * (0,0,1) and the lateral direction exactly (-1,0,0) -- an axis-aligned case
 * where the expected numbers can be worked out by hand, no helper needed. ---- */
static void test_velocity_restitution_and_tangential_preserved(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f);
    f32 limit = limit_at(track, center);
    Vec3 probe = vec3_add(center, vec3_make(-(limit + 3.0f), 0.0f, 0.0f)); /* beyond the left edge */
    const f32 v_outward_in = 5.0f;   /* speed driving further left, off the track */
    const f32 v_tangent_in = 20.0f;  /* speed along the straight            */
    RigidBody body = make_body(probe, vec3_make(-v_outward_in, 0.0f, v_tangent_in));
    int cache = TRACK_UNKNOWN_SEGMENT;
    f32 expected_vx;

    barrier_apply(track, &body, &cache);

    /* left_dir on this straight is (-1,0,0) (see track.c: this segment runs
     * south->north, +Z) -- so outward velocity lives entirely in .x, and
     * tangential velocity lives entirely in .z, with zero cross-talk. */
    expected_vx = -v_outward_in + (1.0f + BARRIER_RESTITUTION_FACTOR) * v_outward_in;
    CHECK(nearly_equal_f32(body.linear_velocity.x, expected_vx, 1e-4f, 1e-4f),
          "outward velocity must be reversed and scaled by BARRIER_RESTITUTION_FACTOR");
    CHECK(body.linear_velocity.x > 0.0f,
          "the reflected velocity must point back toward the track, not further off it");
    CHECK(body.linear_velocity.z == v_tangent_in,
          "tangential velocity must be exactly unchanged (this straight has zero cross-talk between axes)");
    CHECK(body.linear_velocity.y == 0.0f, "barrier_apply must never touch vertical velocity");
}

/* ---- 5b. an inward-moving car past the limit still gets its position
 * clamped, but its (already-inward) velocity must be left alone -- there is
 * nothing to remove, and "fixing" it would inject energy. ---- */
static void test_inward_velocity_left_alone(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f);
    f32 limit = limit_at(track, center);
    Vec3 probe = vec3_add(center, vec3_make(-(limit + 3.0f), 0.0f, 0.0f));
    Vec3 inward_velocity = vec3_make(4.0f, 0.0f, 7.0f); /* +x is back toward the track here */
    RigidBody body = make_body(probe, inward_velocity);
    int cache = TRACK_UNKNOWN_SEGMENT;

    barrier_apply(track, &body, &cache);

    CHECK(memcmp(&body.linear_velocity, &inward_velocity, sizeof(Vec3)) == 0,
          "a car already heading back inward must not have its velocity altered");
    CHECK(fabsf(body.position.x - (center.x - limit)) < 0.01f,
          "position must still be clamped to the limit even when velocity needed no correction");
}

/* ---- 6. repeated wall contact over many steps stays bounded: does not
 * creep through the wall and does not gain energy, simulating a car that
 * keeps steering into the wall every single step. Rate-independent: the run
 * length is expressed in simulated seconds via PHYSICS_HZ, not a raw step
 * count. ---- */
static void test_repeated_contact_stays_bounded(const Track *track) {
    Vec3 center = sample_track_position(track, 0.1f);
    f32 limit = limit_at(track, center);
    RigidBody body = make_body(vec3_add(center, vec3_make(-(limit - 0.5f), 0.0f, 0.0f)), vec3_zero());
    int cache = TRACK_UNKNOWN_SEGMENT;
    /* A constant push toward the outside (-X on this straight, see the
     * axis-aligned note above) every step, as if the driver held the wheel
     * hard left into the wall the whole time. */
    const Vec3 push_force = vec3_make(-4000.0f, 0.0f, 0.0f);
    const int steps = (int)(3.0f / PHYSICS_DT + 0.5f);
    int i;
    int stayed_bounded = 1;
    int never_nan = 1;
    f32 max_speed = 0.0f;

    for (i = 0; i < steps; i++) {
        TrackQueryResult q;
        f32 step_limit;

        rigidbody_clear_accumulators(&body);
        rigidbody_apply_force_at_point(&body, push_force, body.position);
        rigidbody_step(&body, PHYSICS_DT);
        barrier_apply(track, &body, &cache);

        if (!isfinite(body.position.x) || !isfinite(body.position.z) ||
            !isfinite(body.linear_velocity.x) || !isfinite(body.linear_velocity.z)) {
            never_nan = 0;
            break;
        }

        track_query(track, body.position, &cache, &q);
        step_limit = q.half_width + BARRIER_RUNOFF_M - BARRIER_INSET_M;
        if (step_limit < 0.0f) step_limit = 0.0f;
        if (fabsf(q.lateral_offset) > step_limit + 0.01f) {
            stayed_bounded = 0;
        }

        {
            f32 speed = vec3_length(body.linear_velocity);
            if (speed > max_speed) max_speed = speed;
        }
    }

    CHECK(never_nan, "position/velocity must stay finite over hundreds of wall contacts");
    CHECK(stayed_bounded, "position must stay within the limit on every single step, never creeping through");
    CHECK(max_speed < 50.0f,
          "repeatedly bouncing off the wall must not accumulate unbounded energy");
}

/* ---- 7. degenerate inputs: NULL pointers, a track too small to be a loop,
 * a NaN position, and a zero-length nearest segment must each do nothing
 * rather than write garbage. ---- */
static void test_degenerate_inputs_are_safe(const Track *track) {
    RigidBody body;
    int cache;
    Vec3 pos_before;
    Track tiny_track;
    Track degenerate_track;

    /* NULL track/body/cached_segment must not crash. */
    body = make_body(vec3_make(-12.0f, 0.0f, 60.0f), vec3_make(1.0f, 0.0f, 1.0f));
    cache = TRACK_UNKNOWN_SEGMENT;
    barrier_apply(NULL, &body, &cache);
    barrier_apply(track, NULL, &cache);
    barrier_apply(track, &body, NULL);
    CHECK(1, "NULL track/body/cached_segment must not crash barrier_apply");

    /* A track with too few waypoints to be a valid loop. */
    memset(&tiny_track, 0, sizeof(tiny_track));
    tiny_track.count = 0;
    body = make_body(vec3_make(0.0f, 0.0f, 0.0f), vec3_make(5.0f, 0.0f, 0.0f));
    pos_before = body.position;
    cache = TRACK_UNKNOWN_SEGMENT;
    barrier_apply(&tiny_track, &body, &cache);
    CHECK(memcmp(&body.position, &pos_before, sizeof(Vec3)) == 0,
          "an invalid (too-small) track must leave the body completely untouched");

    /* A NaN position. */
    {
        f32 nan_val = NAN;
        body = make_body(vec3_make(nan_val, 0.0f, 60.0f), vec3_make(3.0f, 0.0f, 3.0f));
        Vec3 vel_before = body.linear_velocity;
        cache = TRACK_UNKNOWN_SEGMENT;
        barrier_apply(track, &body, &cache);
        CHECK(isnan(body.position.x), "a NaN position component must be left as-is, not overwritten");
        CHECK(memcmp(&body.linear_velocity, &vel_before, sizeof(Vec3)) == 0,
              "a NaN position must leave velocity untouched too");
    }

    /* A degenerate (zero-length) nearest segment: two coincident waypoints,
     * queried from a point that resolves to that exact segment (see this
     * function's construction: segment 0 has zero length and, by scan
     * order, wins ties against the other, non-zero-length segments at the
     * same distance). */
    memset(&degenerate_track, 0, sizeof(degenerate_track));
    {
        TrackWaypoint wp[3];
        wp[0].center = vec3_make(0.0f, 0.0f, 0.0f);
        wp[0].half_width = 4.0f;
        wp[1].center = vec3_make(0.0f, 0.0f, 0.0f); /* coincident with wp[0] */
        wp[1].half_width = 4.0f;
        wp[2].center = vec3_make(10.0f, 0.0f, 0.0f);
        wp[2].half_width = 4.0f;
        track_init(&degenerate_track, wp, 3);
    }
    body = make_body(vec3_make(0.0f, 0.0f, 5.0f), vec3_make(1.0f, 0.0f, 1.0f));
    pos_before = body.position;
    {
        Vec3 vel_before = body.linear_velocity;
        cache = TRACK_UNKNOWN_SEGMENT;
        barrier_apply(&degenerate_track, &body, &cache);
        CHECK(memcmp(&body.position, &pos_before, sizeof(Vec3)) == 0,
              "a zero-length nearest segment has no lateral direction -- position must be untouched");
        CHECK(memcmp(&body.linear_velocity, &vel_before, sizeof(Vec3)) == 0,
              "a zero-length nearest segment has no lateral direction -- velocity must be untouched");
    }
}

int main(void) {
    Track track;

    track_build_example_oval(&track);

    test_wall_stands_outside_the_ribbon(&track);
    test_no_op_well_inside(&track);
    test_no_op_at_limit_boundary(&track);
    test_left_right_on_straight(&track);
    test_both_straights_and_both_hairpins(&track);
    test_velocity_restitution_and_tangential_preserved(&track);
    test_inward_velocity_left_alone(&track);
    test_repeated_contact_stays_bounded(&track);
    test_degenerate_inputs_are_safe(&track);

    printf("test_barrier: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
