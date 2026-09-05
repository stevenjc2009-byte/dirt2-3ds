/*---------------------------------------------------------------------------------
 * input.c -- Circle Pad dead-zone + response-curve steering, and digital-
 * button throttle/brake ramped over real elapsed time. See input.h for the
 * full hardware-facts contract (raw range, dead zone, why throttle/brake
 * must never reach a caller as a raw button state) this file implements.
 *
 * This is the ONLY translation unit in the project allowed to call
 * libctru's hidCircleRead/hidKeysHeld -- that call is isolated inside
 * input_update's #ifdef __3DS__ block below, so the host build
 * (Makefile.host, zero 3DS deps) still compiles this file cleanly. All of
 * the actual math (dead zone, over-travel clamp, response curve, frame-rate
 * -independent ramp) lives in input_update_raw, which touches no libctru
 * symbol at all and is exercised directly by tests/test_physics.c's link
 * and by this module's host-side verification probe.
 *
 * CONTROL SCHEME (this file's own decision -- input.h documents the L/R
 * hardware facts but does not pin a scheme down, and nothing else in the
 * codebase references a specific KEY_* mapping). Circle Pad, under the left
 * thumb, steers. The two shoulder buttons, one per index finger, are the
 * ramped throttle/brake this file exists to produce: KEY_R accelerates,
 * KEY_L brakes -- the common shoulder-trigger convention. KEY_A, under the
 * right thumb and otherwise unused since steering isn't on the face
 * buttons here, is the handbrake. This mapping is isolated to input_update
 * below; nothing outside this file needs to change to pick a different one.
 *---------------------------------------------------------------------------------*/
#include "input/input.h"

#include <math.h>

#ifdef __3DS__
#include <3ds.h>
#endif

void input_init(InputState *state) {
    if (!state) return;
    state->steer = 0.0f;
    state->throttle = 0.0f;
    state->brake = 0.0f;
    state->handbrake = false;
}

static f32 clampf(f32 v, f32 lo, f32 hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Sane default and upper bound for steer_response_exponent -- see
 * InputConfig's field comment in input.h. The default is a feel choice
 * (finer near-centre control than linear); the max is a safety rail, not a
 * feel choice: it exists purely so a garbage config value (uninitialised
 * memory can be ANY bit pattern, not just zero) can't produce a
 * near-dead-feeling curve the same way an unclamped exponent <= 0 could
 * produce an instant-full-lock one. */
#define INPUT_STEER_RESPONSE_EXPONENT_DEFAULT 1.6f
#define INPUT_STEER_RESPONSE_EXPONENT_MAX     4.0f

/* Moves `current` toward `target` (expected 0.0 or 1.0) by at most
 * `rate` * `dt` and never past `target` -- this is what makes the ramp
 * FRAME-RATE INDEPENDENT: the wall-clock time to go from 0.0 to 1.0 is the
 * sum of `dt` across however many calls it took, which telescopes to
 * exactly elapsed_seconds = 1.0 / rate regardless of how that elapsed time
 * was split into steps, up to less than one dt of rounding on the final
 * (clamped) step -- see this module's verification probe, which measures
 * that directly at 60/100/120 Hz rather than assuming it.
 *
 * A non-positive `rate` or `dt` (garbage config, or a caller passing a
 * frozen/negative clock) holds `current` rather than dividing by zero or
 * ramping backwards -- fails safe instead of corrupting a value every other
 * module trusts to stay in [0, 1]. */
static f32 ramp_toward(f32 current, f32 target, f32 rate, f32 dt) {
    f32 max_step;
    if (rate <= 0.0f || dt <= 0.0f) return current;
    max_step = rate * dt;
    if (target > current) {
        current += max_step;
        if (current > target) current = target;
    } else if (target < current) {
        current -= max_step;
        if (current < target) current = target;
    }
    return current;
}

/* Moves `current` toward the freshly-shaped `target` steer value, with a
 * rate limit that applies ONLY when the player is easing off the stick --
 * see InputConfig.steer_return_rate's field comment for why. Concretely,
 * "returning toward centre" means `target` is strictly between 0 and
 * `current` (or equal to it) on the SAME side of centre as `current`:
 *
 *   - Steering further IN (|target| > |current|, same side, or `current`
 *     was 0): applied instantly. A rally car needs immediate response to
 *     an increasing steering input.
 *   - A direction reversal (target and current have opposite, non-zero
 *     signs -- e.g. catching a slide by throwing the stick the other way):
 *     applied instantly. Rate-limiting this would lag exactly the
 *     countersteer a loose, slidey car most needs to be responsive.
 *   - Easing off (0 <= |target| < |current|, same side): rate-limited by
 *     `return_rate` (fraction of full lock per second), so releasing the
 *     stick returns toward centre smoothly instead of snapping -- see
 *     input.h's contract.
 *
 * A non-positive `return_rate` or `dt` means "no rate limit configured":
 * falls through to the instant/old behaviour rather than dividing by zero
 * or holding `current` forever (holding would be wrong here -- unlike
 * ramp_toward's ramps, which fail safe by freezing, an un-configured
 * return rate should not leave steering stuck away from centre). */
static f32 steer_toward(f32 current, f32 target, f32 return_rate, f32 dt) {
    f32 max_step;
    bool returning;

    if (return_rate <= 0.0f || dt <= 0.0f) return target;

    returning = (current > 0.0f && target >= 0.0f && target < current) ||
                (current < 0.0f && target <= 0.0f && target > current);
    if (!returning) return target;

    max_step = return_rate * dt;
    if (current > target) {
        current -= max_step;
        if (current < target) current = target;
    } else {
        current += max_step;
        if (current > target) current = target;
    }
    return current;
}

/* Dead-zones and shapes one raw Circle Pad axis into [-1, 1]. `radius` and
 * `deadzone` are the already-sanitised config values (see input_update_raw);
 * `exponent` likewise. Steps, matching input.h's contract:
 *
 *   1. Normalise by `radius` (the measured ~156 raw-unit usable travel),
 *      not the full s16 range.
 *   2. Dead-zone as a RESCALE, not a hard subtract: below the dead zone the
 *      result is exactly 0; at and above it, the remaining travel is
 *      remapped back onto the full 0..1 range so there is no discontinuous
 *      jump at the dead-zone boundary.
 *   3. OVER-TRAVEL CLAMP: a worn Circle Pad can read past the nominal
 *      radius. Clamp the post-deadzone fraction to [0, 1] before shaping,
 *      so over-travel saturates at exactly 1.0 instead of exceeding it.
 *   4. Response curve: fraction^exponent. exponent > 1 compresses small
 *      deflections near centre (finer low-speed steering) while the curve
 *      still passes through exactly 0 and 1, because 0^n == 0 and 1^n == 1
 *      for any n > 0 -- both ends are pinned regardless of the exponent,
 *      and the curve is monotonic since `fraction` is already in [0, 1]. */
static f32 shape_axis(int16_t raw, f32 radius, f32 deadzone, f32 exponent) {
    f32 norm, mag, sign, t, shaped;

    if (radius <= 0.0f) return 0.0f;

    norm = (f32)raw / radius;
    mag = fabsf(norm);
    sign = (norm < 0.0f) ? -1.0f : 1.0f;

    if (mag <= deadzone) return 0.0f;

    t = (mag - deadzone) / (1.0f - deadzone);
    t = clampf(t, 0.0f, 1.0f); /* over-travel clamp -- see point 3 above */

    shaped = powf(t, exponent);
    return sign * shaped; /* already in [-1, 1]: t in [0,1] and exponent > 0
                            * (guaranteed by input_update_raw's sanitisation)
                            * means powf(t, exponent) is in [0,1] too -- no
                            * second clamp here, so the over-travel clamp
                            * above is the ONE thing standing between a
                            * worn pad and a >1.0 steer value. */
}

void input_update_raw(InputState *state, const InputConfig *config, f32 dt,
                       int16_t raw_circlepad_x, bool throttle_held,
                       bool brake_held, bool handbrake_held) {
    f32 deadzone, radius, exponent;

    if (!state || !config) return;

    /* Sanitise every tuning number pulled from config before using it.
     * This struct is filled in by main.c (outside this module's
     * ownership), and input.h's contract is that input.c takes no tuning
     * numbers of its own -- so a bad or indeterminate config value must
     * fail safe here rather than feed NaN/Inf into the curve or ramp.
     * fmaxf/fminf return the non-NaN argument when exactly one side is
     * NaN (C99 Annex F), which is what makes clampf's fmaxf/fminf-free
     * comparisons below still need an explicit NaN guard for the
     * exponent -- a `> 0.0f` comparison is false for NaN, so that guard
     * alone already falls through to the linear default. */
    deadzone = clampf(config->circlepad_deadzone, 0.0f, 0.95f);
    radius = config->circlepad_radius;
    /* `> 0.0f` is false for NaN (IEEE 754), so this one comparison already
     * catches <= 0, NaN, and -Inf; the second comparison catches +Inf and
     * any finite-but-absurd value above the documented safety rail. Either
     * failure mode -- exponent <= 0 (instant full lock, fraction^0 == 1)
     * or exponent too large (near-dead feel, fraction^huge_n ~= 0) -- falls
     * back to the same documented default rather than being used raw. */
    exponent = config->steer_response_exponent;
    if (!(exponent > 0.0f) || !(exponent <= INPUT_STEER_RESPONSE_EXPONENT_MAX)) {
        exponent = INPUT_STEER_RESPONSE_EXPONENT_DEFAULT;
    }

    {
        f32 shaped_target = shape_axis(raw_circlepad_x, radius, deadzone, exponent);
        state->steer = steer_toward(state->steer, shaped_target,
                                     config->steer_return_rate, dt);
    }

    state->throttle = ramp_toward(
        state->throttle, throttle_held ? 1.0f : 0.0f,
        throttle_held ? config->throttle_ramp_rate : config->throttle_release_rate,
        dt);
    state->brake = ramp_toward(
        state->brake, brake_held ? 1.0f : 0.0f,
        brake_held ? config->brake_ramp_rate : config->brake_release_rate,
        dt);
    state->handbrake = handbrake_held;
}

void input_update(InputState *state, const InputConfig *config, f32 dt) {
    int16_t raw_x = 0;
    bool throttle_held = false;
    bool brake_held = false;
    bool handbrake_held = false;

    if (!state || !config) return;

#ifdef __3DS__
    {
        circlePosition cp;
        u32 held = hidKeysHeld();
        hidCircleRead(&cp);
        raw_x = cp.dx;
        throttle_held = (held & KEY_R) != 0;
        brake_held = (held & KEY_L) != 0;
        handbrake_held = (held & KEY_A) != 0;
    }
#endif
    /* Host build (no __3DS__): raw_x/held flags stay at their zero/false
     * defaults above -- there is no hardware to read, so this call still
     * exercises the real dead-zone/ramp math on "centred stick, nothing
     * held", exactly the no-op-but-still-linked behaviour
     * Makefile.host's comment on this file documents. A host caller that
     * wants to drive synthetic input through the production path (this
     * module's verification probe, or a future host test) calls
     * input_update_raw directly instead of this function. */

    input_update_raw(state, config, dt, raw_x, throttle_held, brake_held,
                      handbrake_held);
}
