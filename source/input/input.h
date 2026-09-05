/*---------------------------------------------------------------------------------
 * input.h -- reads the 3DS Circle Pad and buttons and turns them into a
 * ramped, vehicle-ready InputState.
 *
 * OWNER: input.
 *
 * HARDWARE FACTS THIS INTERFACE IS SHAPED AROUND.
 *   - The Circle Pad (hidCircleRead) reports raw (x, y) in roughly
 *     [-156, +156] per axis -- NOT a clean [-1, 1] or a documented exact
 *     radius; 156 is the commonly-measured usable magnitude before the pad
 *     physically maxes out, and real pads vary a little unit to unit and
 *     drift near centre. This module owns normalising and dead-zoning that
 *     raw range into [-1, 1] steering -- callers never see raw
 *     circlePosition values.
 *   - L and R are DIGITAL buttons (hidKeysHeld's KEY_L / KEY_R) with NO
 *     analog travel at all -- there is no "half throttle" reading straight
 *     off the hardware the way a modern gamepad's analog triggers would
 *     give you. If throttle/brake were passed straight through as 0-or-1
 *     from the raw button state, every application of power or brake would
 *     be an instant full-value step, which feels undrivable for a rally
 *     car that needs to modulate power on loose surfaces.
 *
 *     This is why InputConfig below carries RAMP RATES (throttle_ramp_rate,
 *     brake_ramp_rate, units/second) rather than the vehicle ever seeing a
 *     raw button state: input.c owns turning "L held" / "not held" into a
 *     smoothly ramped 0..1 value over time, and vehicle.c (and
 *     drivetrain.h) only ever see the ramped result in InputState. Do not
 *     add a "raw L held" field to InputState -- that would let a caller
 *     bypass the ramp and reintroduce the instant-step problem this
 *     interface exists to prevent.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_INPUT_INPUT_H
#define DIRT2_INPUT_INPUT_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Tuning for how raw hardware input is turned into InputState. One of these
 * is set up once at startup (or per-vehicle, if different cars should feel
 * differently responsive to input someday) and passed to input_update every
 * call. */
typedef struct InputConfig {
    f32 circlepad_deadzone;    /* 0..1, fraction of full radius (post-
                                 * normalisation) treated as centred/zero --
                                 * absorbs pad drift near centre            */
    f32 circlepad_radius;      /* raw hidCircleRead units representing full
                                 * deflection -- measured ~156 for the 3DS
                                 * Circle Pad, kept as a config value rather
                                 * than a hardcoded literal so a New 3DS
                                 * C-Stick (added later, per this project's
                                 * "New 3DS enhancements layered on later"
                                 * scope note) can reuse this same struct
                                 * shape with its own measured radius       */
    f32 steer_response_exponent; /* ADDITIVE (input, 2026-09-05): shapes the
                                 * post-deadzone Circle Pad fraction as
                                 * fraction^exponent before it becomes
                                 * InputState.steer. 1.0 is linear;
                                 * exponent > 1.0 makes small deflections
                                 * near centre finer while still reaching
                                 * exactly +-1 at full deflection (0 and 1
                                 * are fixed points of x^n for any n > 0).
                                 *
                                 * RECOMMENDED: 1.6f -- gives noticeably
                                 * finer low-speed control near centre
                                 * (where a rally car needs to hold a small,
                                 * precise line-correction angle) while
                                 * still reaching full lock at full stick
                                 * deflection; chosen for feel, not
                                 * measured.
                                 *
                                 * ANY value <= 0, non-finite (NaN/Inf), or
                                 * above input.c's documented upper bound
                                 * (including an uninitialised struct's
                                 * indeterminate stack/heap value, which can
                                 * legally be ANY bit pattern -- not just
                                 * zero) is replaced by input.c's own
                                 * documented default rather than used raw.
                                 * This is what makes exponent == 0.0f's
                                 * fraction^0 == 1.0 "instant full steering
                                 * lock" landmine impossible regardless of
                                 * what a caller's config happens to
                                 * contain -- see input.c's
                                 * input_update_raw. An unreasonably large
                                 * exponent is guarded too, not just <= 0:
                                 * it has the opposite-but-equally-broken
                                 * failure mode of a near-dead steering feel
                                 * (fraction^huge_n ~= 0 for almost all of
                                 * the stick's travel).
                                 * NOTE: main.c's input_config in source/main.c
                                 * does not yet set this field; that file is
                                 * outside input/'s ownership, so it was not
                                 * edited here -- see the input module's
                                 * verification report.                     */
    f32 steer_return_rate;     /* ADDITIVE (input, 2026-09-05): units of
                                 * steer (0..1 of full lock) per second that
                                 * InputState.steer is allowed to move
                                 * TOWARD centre when the shaped Circle Pad
                                 * reading has a smaller magnitude, same
                                 * sign, than the previous frame's steer --
                                 * i.e. the player easing off the stick, not
                                 * steering further in and not reversing
                                 * direction to countersteer. Steering IN
                                 * (increasing magnitude) and direction
                                 * reversals (catching a slide) are always
                                 * applied instantly, at full stick
                                 * response, regardless of this rate -- a
                                 * laggy countersteer would be actively
                                 * dangerous on a loose-surface car that is
                                 * supposed to need one. This field only
                                 * softens the RELEASE, replacing what would
                                 * otherwise be an instant snap back toward
                                 * centre with a rate-limited return -- see
                                 * input.c's steer_toward.
                                 *
                                 * RECOMMENDED: 8.0f (full lock to centre in
                                 * 1/8 s = 125 ms) -- fast enough to still
                                 * feel responsive, slow enough to smooth
                                 * out the small Circle Pad centring
                                 * jitter/spring-back a real pad exhibits
                                 * as the player's thumb comes off it;
                                 * chosen for feel, not measured.
                                 * A value <= 0 (including an uninitialised
                                 * struct's indeterminate value) is treated
                                 * by input.c as "no rate limit" (instant
                                 * snap, the old behaviour) rather than used
                                 * raw or divided against -- see input.c's
                                 * steer_toward. Unlike steer_response_exponent
                                 * there is no unsafe direction for a
                                 * garbage positive value here: any positive
                                 * rate, sane or not, still monotonically
                                 * approaches the correct target and can
                                 * only ever slow the return, never
                                 * overshoot or invert it.                  */
    f32 throttle_ramp_rate;    /* units of throttle (0..1) per second while
                                 * the throttle button is held -- see header
                                 * note on why this exists instead of a
                                 * digital 0-or-1 throttle                  */
    f32 throttle_release_rate; /* units of throttle (0..1) per second while
                                 * the throttle button is NOT held -- kept
                                 * separate from throttle_ramp_rate so
                                 * lifting off can be tuned to feel snappier
                                 * or gentler than pressing on, independently */
    f32 brake_ramp_rate;       /* same idea as throttle_ramp_rate, for the
                                 * brake button                              */
    f32 brake_release_rate;    /* same idea as throttle_release_rate, for the
                                 * brake button                              */
} InputConfig;

/* Persisted frame-to-frame input state -- ramping needs the previous value
 * to ramp FROM, so this can't be recomputed stateless each frame. */
typedef struct InputState {
    f32 steer;      /* -1 (full left) .. +1 (full right), dead-zoned and
                      * normalised Circle Pad x -- consumed directly by
                      * vehicle.c to set front Wheel.steer_angle            */
    f32 throttle;   /* 0..1, RAMPED value -- never a raw button state, see
                      * header note                                        */
    f32 brake;      /* 0..1, RAMPED value -- same                          */
    bool handbrake; /* Phase 1: digital is fine here -- a handbrake is
                      * meant to be an abrupt, fully-on rear-lock input for
                      * a rally-style pull, not something that benefits
                      * from the same ramping throttle/brake need. Revisit
                      * only if playtesting says otherwise.                 */
} InputState;

void input_init(InputState *state);

/* Reads the current hardware Circle Pad / button state (via libctru's
 * hidCircleRead / hidKeysHeld -- input.c is the ONLY module in this project
 * allowed to call those, so every other module stays testable on the host
 * build without libctru) and advances `state` by `dt` seconds according to
 * `config`'s dead zone and ramp rates.
 *
 * Called once per physics step (same cadence as vehicle_step, so ramp rates
 * are in real seconds regardless of PHYSICS_HZ), BEFORE vehicle_step for
 * that same step -- input for a step must be read before that step's
 * vehicle_step call so drivetrain.h sees this step's throttle/brake, not
 * last step's. */
void input_update(InputState *state, const InputConfig *config, f32 dt);

/* ADDITIVE (input, 2026-09-05): the same dead-zone/response-curve/ramp math
 * input_update uses on real hardware, but taking the raw Circle Pad X axis
 * and digital button-held flags as plain arguments instead of reading them
 * from libctru. input_update (the __3DS__ path) reads hidCircleRead/
 * hidKeysHeld and forwards straight into this function -- it is not a
 * separate reimplementation.
 *
 * This exists so the host build (Makefile.host, no libctru) and this
 * module's own verification probe can drive the EXACT production
 * dead-zone/ramp/curve code with synthetic values, instead of a hand-copied
 * stand-in that could silently drift from what actually ships on console.
 * `raw_circlepad_x` is a raw hidCircleRead-range value (see
 * InputConfig.circlepad_radius), not pre-normalised. */
void input_update_raw(InputState *state, const InputConfig *config, f32 dt,
                       int16_t raw_circlepad_x, bool throttle_held,
                       bool brake_held, bool handbrake_held);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_INPUT_INPUT_H */
