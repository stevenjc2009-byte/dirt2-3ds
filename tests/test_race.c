/*---------------------------------------------------------------------------------
 * test_race.c -- host-side test suite for source/race/track.c and
 * source/race/lap.c (v0.2 "it's a race": closed loop, start/finish line,
 * lap counter, lap timer, best time).
 *
 * RATE INDEPENDENCE.
 *   Every simulated duration below is expressed as "N simulated seconds"
 *   via PHYSICS_HZ (core/timestep.h's compile-time constant) converted to a
 *   step count, never as a raw step count -- so the same source, built
 *   three times at PHYSICS_HZ 60/100/120, exercises the same real-time
 *   window at each rate. A lap counter that only works at one physics rate
 *   is broken; see this project's dirt2 test_physics.c for the same
 *   convention.
 *
 * FAULT INJECTION (proving a check can go red).
 *   source/race/track.c and source/race/lap.c each carry a small number of
 *   `#if DIRT2_INJECT_FAULT == N` blocks that deliberately break one real
 *   piece of logic when built with -DDIRT2_INJECT_FAULT=N (see each file's
 *   own header comment for the numbered list). This file requires no
 *   changes to run under fault injection -- rebuilding the exact same
 *   tests/test_race.c against a faulted track.c/lap.c is what proves a
 *   specific CHECK below can fail on real code, not just on an absent
 *   implementation. Default (no -D) is 0, the normal, all-passing build.
 *---------------------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

#include "core/types.h"
#include "core/vecmath.h"
#include "core/timestep.h"
#include "race/track.h"
#include "race/lap.h"

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

/* ---- test-only helper: sample a world position at a given point along the
 * track's centreline, independent of track_query (the thing being tested)
 * so this file has its own ground truth to drive a car along. `lap_fraction`
 * may be any real number (including negative, or > 1) -- wrapped into
 * [0, 1) before sampling, so callers can express "reverse past the start"
 * as a negative fraction and get the correct wrapped-around position. ---- */
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

/* The simplest possible "did the car cross the line" detector: any jump in
 * progress bigger than half a lap, in EITHER direction, counts as a lap --
 * no forward/backward distinction, no far-side gate. This is deliberately
 * the naive thing an first-draft crossing test looks like, kept here (not
 * in lap.c) purely to be run side-by-side against the real LapState on the
 * exact same trajectory and show it gets the wrong answer -- see
 * test_reverse_crossing_counts_zero, test_out_and_back_counts_zero, and
 * test_back_and_forth_counts_zero. */
static int naive_crossing_count(const f32 *progress, int n) {
    int count = 0;
    int i;
    for (i = 1; i < n; i++) {
        f32 delta = progress[i] - progress[i - 1];
        if (delta < -0.5f || delta > 0.5f) count++;
    }
    return count;
}

static int seconds_to_steps(f32 seconds) {
    int steps = (int)(seconds / PHYSICS_DT + 0.5f);
    return steps > 0 ? steps : 1;
}

/* ---- track.c: progress is monotonic and wraps exactly once per lap ---- */

static void test_progress_monotonic_wraps_once(Track *track) {
    const int N = 400; /* samples per lap -- a spatial resolution choice,
                         * independent of PHYSICS_HZ (this test drives
                         * track_query directly, not through lap_update) */
    int cached_segment = 0;
    f32 prev_progress = 0.0f;
    int wraps = 0;
    int monotonic_violations = 0;
    int i;

    for (i = 0; i <= N; i++) {
        f32 lap_fraction = (f32)i / (f32)N; /* 0 .. 1 inclusive, one full lap */
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        track_query(track, pos, &cached_segment, &q);

        if (i > 0) {
            f32 delta = q.progress - prev_progress;
            if (delta < -0.5f) {
                wraps++;
            } else if (delta < 0.0f) {
                monotonic_violations++;
            }
        }
        prev_progress = q.progress;
    }

    CHECK(wraps == 1, "progress must wrap exactly once over one full lap");
    CHECK(monotonic_violations == 0,
          "progress must never decrease except at the single wrap point");
}

/* ---- lap.c: a full clean lap counts exactly one ---- */

static void test_clean_lap_counts_one(const Track *track) {
    LapState lap;
    const f32 lap_duration_s = 12.0f;
    int steps = seconds_to_steps(lap_duration_s);
    int completions = 0;
    int i;

    lap_init(&lap, track);
    for (i = 0; i <= steps; i++) {
        f32 lap_fraction = (f32)i / (f32)steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        if (lap_update(&lap, pos, PHYSICS_DT)) completions++;
    }

    CHECK(lap.lap_count == 1, "one full clean lap must count exactly one lap");
    CHECK(completions == 1, "lap_update must return true exactly once for one clean lap");
    CHECK(lap.last_lap_time != LAP_NO_TIME, "last_lap_time must be recorded after a completed lap");
    CHECK(nearly_equal_f32(lap.last_lap_time, lap_duration_s, 0.05f, 0.01f),
          "last_lap_time must match the simulated lap duration");
    CHECK(lap.best_lap_time == lap.last_lap_time, "best_lap_time must equal the only lap run so far");
}

/* ---- lap.c: a reverse crossing (never having gone around) counts zero --- */

static void test_reverse_crossing_counts_zero(const Track *track) {
    LapState lap;
    const int back_steps = seconds_to_steps(1.0f);
    f32 progress_samples[256];
    int n_samples = 0;
    int i;

    lap_init(&lap, track);
    /* Prime at the line, then drive BACKWARD across it -- never anywhere
     * near the far side. */
    progress_samples[n_samples++] = 0.0f;
    lap_update(&lap, sample_track_position(track, 0.0f), PHYSICS_DT);

    for (i = 1; i <= back_steps; i++) {
        f32 lap_fraction = -0.02f * (f32)i; /* small steps backward, wraps
                                              * negative through the line */
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        int dummy_cache = 0;
        track_query(track, pos, &dummy_cache, &q);
        if (n_samples < 256) progress_samples[n_samples++] = q.progress;
        lap_update(&lap, pos, PHYSICS_DT);
    }

    CHECK(lap.lap_count == 0, "reversing back over the line must not count as a lap");
    CHECK(naive_crossing_count(progress_samples, n_samples) >= 1,
          "sanity: this trajectory really does cross the line at least once "
          "(a naive any-direction crossing counter wrongly reports it as a lap)");
}

/* ---- lap.c: an out-and-back over the line counts zero ---- */

static void test_out_and_back_counts_zero(const Track *track) {
    LapState lap;
    const int out_steps = seconds_to_steps(0.5f);
    const int back_steps = seconds_to_steps(0.5f);
    f32 progress_samples[256];
    int n_samples = 0;
    int i;

    lap_init(&lap, track);
    progress_samples[n_samples++] = 0.0f;
    lap_update(&lap, sample_track_position(track, 0.0f), PHYSICS_DT);

    /* Out: forward a little, never near the far side. */
    for (i = 1; i <= out_steps; i++) {
        f32 lap_fraction = 0.03f * (f32)i / (f32)out_steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        int dummy_cache = 0;
        track_query(track, pos, &dummy_cache, &q);
        if (n_samples < 256) progress_samples[n_samples++] = q.progress;
        lap_update(&lap, pos, PHYSICS_DT);
    }
    /* Back: reverse past the start line by the same margin. */
    for (i = 1; i <= back_steps; i++) {
        f32 lap_fraction = 0.03f - 0.06f * (f32)i / (f32)back_steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        int dummy_cache = 0;
        track_query(track, pos, &dummy_cache, &q);
        if (n_samples < 256) progress_samples[n_samples++] = q.progress;
        lap_update(&lap, pos, PHYSICS_DT);
    }

    CHECK(lap.lap_count == 0, "an out-and-back over the line must not count as a lap");
    CHECK(naive_crossing_count(progress_samples, n_samples) >= 1,
          "sanity: this trajectory crosses the line (a naive any-direction "
          "crossing counter wrongly reports it as a lap)");
}

/* ---- lap.c: back-and-forth over the line (reverse across it, THEN forward
 * across it again) counts zero -- specifically exercises the far-side gate
 * on a FORWARD wrap, unlike the two tests above.
 *
 * test_reverse_crossing_counts_zero's only crossing is a BACKWARD wrap, and
 * test_out_and_back_counts_zero's only crossing is also a BACKWARD wrap
 * (forward a little with no wrap at all, then back across the line) -- the
 * far-side gate (lap.c's `gate_open`) is only ever read inside the FORWARD
 * wrap branch, so neither of those trajectories can tell a correctly-gated
 * forward wrap apart from an ungated one (DIRT2_INJECT_FAULT=1 removes the
 * gate entirely and both of those tests still pass). This test's second leg
 * -- forward again, back across the line -- is a genuine forward wrap with
 * far_side_reached still false (the car never went anywhere near the
 * midpoint), so it is the one trajectory here that actually goes red under
 * DIRT2_INJECT_FAULT=1. */

static void test_back_and_forth_counts_zero(const Track *track) {
    LapState lap;
    const int back_steps = seconds_to_steps(0.5f);
    const int forward_steps = seconds_to_steps(0.5f);
    f32 progress_samples[256];
    int n_samples = 0;
    int i;

    lap_init(&lap, track);
    /* Prime just past the line, nowhere near the far side. */
    progress_samples[n_samples++] = 0.02f;
    lap_update(&lap, sample_track_position(track, 0.02f), PHYSICS_DT);

    /* Leg 1: reverse across the line (a BACKWARD wrap -- must not count,
     * and must not touch far_side_reached). */
    for (i = 1; i <= back_steps; i++) {
        f32 lap_fraction = 0.02f - 0.06f * (f32)i / (f32)back_steps; /* -> ~-0.04 */
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        int dummy_cache = 0;
        track_query(track, pos, &dummy_cache, &q);
        if (n_samples < 256) progress_samples[n_samples++] = q.progress;
        lap_update(&lap, pos, PHYSICS_DT);
    }
    CHECK(lap.lap_count == 0, "reversing across the line must not count as a lap");

    /* Leg 2: forward again, back across the line (a FORWARD wrap -- the one
     * this module's far-side gate must catch, since the car still hasn't
     * been anywhere near the midpoint). */
    for (i = 1; i <= forward_steps; i++) {
        f32 lap_fraction = -0.04f + 0.06f * (f32)i / (f32)forward_steps; /* -> ~0.02 */
        Vec3 pos = sample_track_position(track, lap_fraction);
        TrackQueryResult q;
        int dummy_cache = 0;
        track_query(track, pos, &dummy_cache, &q);
        if (n_samples < 256) progress_samples[n_samples++] = q.progress;
        lap_update(&lap, pos, PHYSICS_DT);
    }

    CHECK(lap.lap_count == 0,
          "a forward crossing after a reverse crossing, still nowhere near the far side, "
          "must not count as a lap");
    CHECK(naive_crossing_count(progress_samples, n_samples) >= 2,
          "sanity: this trajectory crosses the line twice (a naive any-direction "
          "crossing counter wrongly reports two laps)");
}

/* ---- lap.c: best time only updates on improvement ---- */

static void test_best_time_updates_on_improvement(const Track *track) {
    LapState lap;
    f32 lap_durations[3];
    f32 recorded_last[3];
    f32 recorded_best[3];
    f32 cursor = 0.0f; /* running lap fraction, carried across all three laps */
    int lap_idx;

    lap_durations[0] = 10.0f; /* baseline            */
    lap_durations[1] = 13.0f; /* slower -- must NOT become best */
    lap_durations[2] = 8.0f;  /* fastest -- must become best    */

    lap_init(&lap, track);

    for (lap_idx = 0; lap_idx < 3; lap_idx++) {
        int steps = seconds_to_steps(lap_durations[lap_idx]);
        int i;
        for (i = 1; i <= steps; i++) {
            f32 lap_fraction = cursor + (f32)i / (f32)steps;
            Vec3 pos = sample_track_position(track, lap_fraction);
            lap_update(&lap, pos, PHYSICS_DT);
        }
        cursor += 1.0f;
        recorded_last[lap_idx] = lap.last_lap_time;
        recorded_best[lap_idx] = lap.best_lap_time;
    }

    CHECK(lap.lap_count == 3, "three full laps must count as three");

    CHECK(nearly_equal_f32(recorded_last[0], lap_durations[0], 0.05f, 0.01f),
          "lap 1's recorded time must match its simulated duration");
    CHECK(nearly_equal_f32(recorded_best[0], lap_durations[0], 0.05f, 0.01f),
          "best after lap 1 must equal lap 1 (the only lap so far)");

    CHECK(nearly_equal_f32(recorded_last[1], lap_durations[1], 0.05f, 0.01f),
          "lap 2's recorded time must match its simulated duration");
    CHECK(nearly_equal_f32(recorded_best[1], lap_durations[0], 0.05f, 0.01f),
          "best after a SLOWER lap 2 must stay lap 1's time, not update");

    CHECK(nearly_equal_f32(recorded_last[2], lap_durations[2], 0.05f, 0.01f),
          "lap 3's recorded time must match its simulated duration");
    CHECK(nearly_equal_f32(recorded_best[2], lap_durations[2], 0.05f, 0.01f),
          "best after a FASTER lap 3 must update to lap 3's time");
}

/* ---- lap.c: times are driven by accumulated dt, not step count ---- */

static void test_dt_driven_timing(const Track *track) {
    LapState lap_fine, lap_coarse;
    const f32 lap_duration_s = 9.0f;
    int fine_steps = seconds_to_steps(lap_duration_s);
    int coarse_steps = fine_steps / 8;
    f32 coarse_dt;
    int i;

    if (coarse_steps < 1) coarse_steps = 1;
    coarse_dt = lap_duration_s / (f32)coarse_steps;

    lap_init(&lap_fine, track);
    for (i = 1; i <= fine_steps; i++) {
        f32 lap_fraction = (f32)i / (f32)fine_steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        lap_update(&lap_fine, pos, PHYSICS_DT);
    }

    lap_init(&lap_coarse, track);
    for (i = 1; i <= coarse_steps; i++) {
        f32 lap_fraction = (f32)i / (f32)coarse_steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        lap_update(&lap_coarse, pos, coarse_dt);
    }

    CHECK(lap_fine.lap_count == 1 && lap_coarse.lap_count == 1,
          "both the fine-step and coarse-step run must complete exactly one lap");
    /* Same total elapsed time (fine_steps*PHYSICS_DT == coarse_steps*coarse_dt
     * by construction), reached via a very different NUMBER of lap_update
     * calls -- if timing were driven by step count instead of dt, these
     * would disagree sharply (coarse run has 1/8th as many steps). */
    CHECK(nearly_equal_f32(lap_fine.last_lap_time, lap_coarse.last_lap_time, 0.05f, 0.02f),
          "lap time must depend on summed dt, not on how many steps it took to sum it");
    CHECK(nearly_equal_f32(lap_fine.total_time, lap_coarse.total_time, 0.05f, 0.02f),
          "total_time must depend on summed dt, not on how many steps it took to sum it");
}

/* ---- lap.c: a lap completed on the exact same step the car crosses the
 * line is handled -- no crash, no double count, and this step's dt is
 * split between the finishing lap and the new one. ---- */

static void test_same_step_completion(const Track *track) {
    /* Realistic setup, NOT an artificial multi-second hitch: every step
     * here is exactly PHYSICS_DT, matching how lap_update is actually
     * meant to be driven (once per physics step, see lap.h). The crossing
     * still lands mid-step -- it always does, since a discrete sample can
     * never land exactly on the line -- but by choosing N so that the
     * approach's last sample is exactly half a step short of the line,
     * the final step's crossing is deliberately centred (half the step's
     * distance before the line, half after), giving a clean, non-
     * degenerate 50/50 time split to check instead of an accidental ~0 or
     * ~1 fraction. */
    LapState lap;
    const int N = seconds_to_steps(12.0f);
    f32 expected_lap_time = (f32)N * PHYSICS_DT;
    int i;
    bool completed_this_call = false;

    lap_init(&lap, track);
    /* i = 1 .. N-1 lands the approach at progress (N-1)/N, exactly one
     * step short of the line; final call below covers the last step. */
    for (i = 1; i < N; i++) {
        f32 lap_fraction = (f32)i / (f32)N;
        Vec3 pos = sample_track_position(track, lap_fraction);
        bool completed = lap_update(&lap, pos, PHYSICS_DT);
        CHECK(!completed, "must not report a completion before the line is actually reached");
    }
    CHECK(lap.lap_count == 0, "must not have completed a lap yet before the final step");

    /* One more ordinary PHYSICS_DT step, landing exactly one step's worth
     * PAST the line (progress 1/N) -- the crossing happens inside THIS
     * call. */
    completed_this_call = lap_update(&lap, sample_track_position(track, (f32)(N + 1) / (f32)N), PHYSICS_DT);

    CHECK(completed_this_call, "lap_update must return true on the exact step it completes");
    CHECK(lap.lap_count == 1, "a same-step crossing must count as exactly one lap, not zero or two");
    CHECK(isfinite(lap.current_lap_time) && isfinite(lap.last_lap_time),
          "current/last lap time must stay finite after a same-step completion");
    CHECK(nearly_equal_f32(lap.last_lap_time, expected_lap_time, 0.05f, 0.02f),
          "the finished lap's time must match the simulated duration up to the crossing");
    /* The crossing was engineered to fall exactly halfway through the
     * final step (equal remaining-progress-to-line and progress-past-line)
     * -- so the new lap should have picked up very close to half of that
     * one step's dt, not the whole step and not none of it. */
    CHECK(nearly_equal_f32(lap.current_lap_time, PHYSICS_DT * 0.5f, PHYSICS_DT * 0.1f, 0.0f),
          "a mid-step crossing must split that step's dt between the finishing and new lap, not assign it all to one");
}

/* ---- lap.c: lap_notify_reset stops a teleport from being misread as a
 * completed lap. ---- */

static void test_reset_prevents_false_lap(const Track *track) {
    LapState lap;
    const int approach_steps = seconds_to_steps(6.0f);
    int i;

    lap_init(&lap, track);
    /* Drive forward past the far side (0.7 > 0.5), so far_side_reached is
     * latched, WITHOUT reaching the line yet. */
    for (i = 1; i <= approach_steps; i++) {
        f32 lap_fraction = 0.7f * (f32)i / (f32)approach_steps;
        Vec3 pos = sample_track_position(track, lap_fraction);
        lap_update(&lap, pos, PHYSICS_DT);
    }
    CHECK(lap.lap_count == 0, "must not have completed a lap yet before the reset");

    /* A car reset/respawn near the start (progress ~0.05) is a position
     * DISCONTINUITY, not driving -- without lap_notify_reset, this jump
     * from progress 0.7 to 0.05 (raw delta -0.65) looks exactly like a
     * genuine forward wrap with far_side_reached already true, and would
     * be wrongly credited as a completed lap. */
    lap_notify_reset(&lap);
    lap_update(&lap, sample_track_position(track, 0.05f), PHYSICS_DT);

    CHECK(lap.lap_count == 0,
          "a reset/teleport must never itself be credited as a completed lap");

    /* And the lap counter must still work normally afterwards: driving a
     * full genuine lap from the reprimed position must count exactly one. */
    {
        int steps = seconds_to_steps(11.0f);
        int completions = 0;
        for (i = 1; i <= steps; i++) {
            f32 lap_fraction = 0.05f + (f32)i / (f32)steps;
            Vec3 pos = sample_track_position(track, lap_fraction);
            if (lap_update(&lap, pos, PHYSICS_DT)) completions++;
        }
        CHECK(lap.lap_count == 1 && completions == 1,
              "after a reset, a genuine full lap from the new position must still count exactly one");
    }
}

/* ---- lap.c: starting mid-lap already past the midpoint credits a
 * (shorter) first lap on reaching the line -- see lap.h's lap_init note. --- */

static void test_start_mid_lap_past_midpoint_credits_short_lap(const Track *track) {
    LapState lap;
    const int steps = seconds_to_steps(3.0f);
    int completions = 0;
    int i;

    lap_init(&lap, track);
    /* First sample is already at progress 0.8 -- past the midpoint. */
    completions += lap_update(&lap, sample_track_position(track, 0.8f), PHYSICS_DT) ? 1 : 0;

    for (i = 1; i <= steps; i++) {
        f32 lap_fraction = 0.8f + 0.2f * (f32)i / (f32)steps; /* -> wraps to ~0.0 */
        Vec3 pos = sample_track_position(track, lap_fraction);
        if (lap_update(&lap, pos, PHYSICS_DT)) completions++;
    }

    CHECK(lap.lap_count == 1 && completions == 1,
          "starting past the midpoint and driving forward to the line must credit one (short) lap");
}

int main(void) {
    Track track;

    track_build_example_oval(&track);
    CHECK(track.count >= 3, "the example oval must build a valid closed loop");
    CHECK(track.total_length > 0.0f, "the example oval must have a positive loop length");

    test_progress_monotonic_wraps_once(&track);
    test_clean_lap_counts_one(&track);
    test_reverse_crossing_counts_zero(&track);
    test_out_and_back_counts_zero(&track);
    test_back_and_forth_counts_zero(&track);
    test_best_time_updates_on_improvement(&track);
    test_dt_driven_timing(&track);
    test_same_step_completion(&track);
    test_reset_prevents_false_lap(&track);
    test_start_mid_lap_past_midpoint_credits_short_lap(&track);

    printf("test_race: %d checks, %d failures (PHYSICS_HZ=%d)\n",
           g_checks, g_failures, PHYSICS_HZ);
    return g_failures == 0 ? 0 : 1;
}
