/*---------------------------------------------------------------------------------
 * drivetrain.c -- engine RPM model, fixed gear/final-drive ratio, and the
 * per-wheel drive/brake torque split. See drivetrain.h for the full contract
 * this implements, and vehicle.h's step 8 for how vehicle.c uses the output.
 *
 * PHASE 1 SCOPE, per the header: single fixed gear ratio, no clutch, no
 * gearbox shifting, no differential. Concretely that means the gear/final
 * drive connection is treated as RIGID -- engine RPM is a direct, instant
 * function of driven-wheel spin through the fixed ratio, not something
 * integrated against an engine flywheel inertia over `dt` (there is no
 * clutch to let the two sides of the drivetrain slip apart, so there is
 * nothing for `dt` to integrate here; it is accepted for contract
 * compatibility with a future clutch model and otherwise unused).
 *
 * ENGINE TORQUE CURVE.
 *   DrivetrainConfig gives exactly one interior curve point
 *   (peak_torque_rpm, max_engine_torque) plus the RPM range it is valid
 *   over (idle_rpm..max_rpm). Rather than inventing extra tuning numbers
 *   this module has no authority to pick (see this task's brief: tuning
 *   values arrive through config, this module only supplies correct math),
 *   the curve is the simplest piecewise-linear shape that uses ONLY those
 *   supplied numbers and two natural, non-arbitrary boundary values (zero
 *   torque at zero RPM, zero torque at the rev limiter):
 *
 *       torque
 *         ^
 *   peak--|            /\
 *         |           /  \
 *         |          /    \
 *         |         /      \
 *       0 +--------o--------o------> rpm
 *         0   idle_rpm  peak_rpm  max_rpm
 *
 *   i.e. torque rises linearly from 0 N*m at 0 RPM to max_engine_torque at
 *   peak_torque_rpm, then falls linearly back to 0 N*m at max_rpm (a
 *   rev-limiter cut). Since drivetrain_update always clamps the RPM used
 *   for the lookup to [idle_rpm, max_rpm] first, only the region from
 *   idle_rpm up is ever actually sampled in normal operation.
 *---------------------------------------------------------------------------------*/
#include "vehicle/drivetrain.h"
#include "core/types.h"
#include <math.h>

/* Not relying on <math.h>'s M_PI -- it is not part of the C standard and is
 * only conditionally exposed by some libcs, so define the one constant this
 * file needs directly. */
#define DRIVETRAIN_PI 3.14159265358979323846f
#define DRIVETRAIN_RAD_S_TO_RPM (60.0f / (2.0f * DRIVETRAIN_PI))

static f32 drivetrain_clampf(f32 v, f32 lo, f32 hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Piecewise-linear engine torque (N*m) at 100% throttle for a given RPM --
 * see the file header comment for the exact shape and why it uses no
 * numbers beyond what DrivetrainConfig supplies. Guards every division so a
 * malformed config (e.g. peak_torque_rpm <= 0, or at/past max_rpm) degrades
 * to a flat curve rather than dividing by zero or a negative span. */
static f32 drivetrain_engine_torque_at_rpm(const DrivetrainConfig *config, f32 rpm) {
    f32 peak_rpm = config->peak_torque_rpm;
    f32 redline = config->max_rpm;
    f32 peak_torque = config->max_engine_torque;

    if (peak_torque <= 0.0f) return 0.0f;
    if (rpm <= 0.0f) return 0.0f;

    if (peak_rpm <= 0.0f) {
        /* No meaningful rising segment defined -- hold flat at peak. */
        return peak_torque;
    }

    if (rpm <= peak_rpm) {
        return peak_torque * (rpm / peak_rpm);
    }

    if (redline <= peak_rpm) {
        /* No meaningful falling segment defined -- hold flat past peak. */
        return peak_torque;
    }

    if (rpm >= redline) return 0.0f;

    {
        f32 t = (rpm - peak_rpm) / (redline - peak_rpm);
        return peak_torque * (1.0f - t);
    }
}

void drivetrain_update(const DrivetrainConfig *config, DrivetrainState *state,
                        f32 throttle, f32 brake, f32 driven_wheel_omega_avg,
                        f32 dt, DrivetrainWheelTorque out_wheel_torque[]) {
    (void)dt; /* no clutch/flywheel model in Phase 1 -- see file header */

    if (!state) return;
    if (!out_wheel_torque) return;

    {
        int i;
        bool driven[VEHICLE_WHEEL_COUNT] = { false, false, false, false };
        int driven_count = 0;
        f32 raw_rpm, rpm;
        f32 throttle_clamped, brake_clamped;
        f32 curve_torque, engine_torque, wheel_torque_total, per_wheel_drive;
        f32 brake_torque_mag;

        if (!config) {
            for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
                out_wheel_torque[i].drive_torque = 0.0f;
                out_wheel_torque[i].brake_torque = 0.0f;
            }
            return;
        }

        /* --- engine RPM from driven-wheel spin through the fixed ratio --- */
        /* gear_ratio/final_drive_ratio are engine-rev -> wheel-rev (see
         * drivetrain.h), so engine speed is wheel speed multiplied by both,
         * not divided. fabsf: RPM is a magnitude regardless of which way the
         * car is currently rolling -- there is no reverse gear modelled in
         * Phase 1, so a rolling-backwards wheel still reads as a positive
         * engine RPM rather than a negative one. */
        raw_rpm = fabsf(driven_wheel_omega_avg) * config->gear_ratio
                  * config->final_drive_ratio * DRIVETRAIN_RAD_S_TO_RPM;

        /* Clamp to [idle_rpm, max_rpm] so a stationary or airborne car (zero
         * driven-wheel spin) reads as idle rather than 0 RPM, and an
         * over-revved wheel (e.g. airborne and spinning freely) cannot push
         * the model past the rev limiter. */
        if (config->max_rpm > config->idle_rpm) {
            rpm = drivetrain_clampf(raw_rpm, config->idle_rpm, config->max_rpm);
        } else {
            /* Malformed config (redline at or below idle) -- degrade to a
             * fixed idle rather than propagate a nonsense range. */
            rpm = config->idle_rpm;
        }
        state->engine_rpm = rpm;

        /* --- torque at that RPM, scaled by throttle, multiplied through the
         * gear/final-drive ratio (an ideal fixed-ratio gearbox multiplies
         * torque by exactly the ratio that divides speed) --- */
        throttle_clamped = drivetrain_clampf(throttle, 0.0f, 1.0f);
        curve_torque = drivetrain_engine_torque_at_rpm(config, rpm);
        engine_torque = curve_torque * throttle_clamped;
        wheel_torque_total = engine_torque * config->gear_ratio * config->final_drive_ratio;

        /* --- which wheels are driven, per layout (even split, no diff) --- */
        switch (config->layout) {
            case DRIVE_FWD:
                driven[WHEEL_FL] = true;
                driven[WHEEL_FR] = true;
                driven_count = 2;
                break;
            case DRIVE_RWD:
                driven[WHEEL_RL] = true;
                driven[WHEEL_RR] = true;
                driven_count = 2;
                break;
            case DRIVE_AWD:
                driven[WHEEL_FL] = true;
                driven[WHEEL_FR] = true;
                driven[WHEEL_RL] = true;
                driven[WHEEL_RR] = true;
                driven_count = 4;
                break;
            default:
                driven_count = 0;
                break;
        }
        per_wheel_drive = (driven_count > 0) ? (wheel_torque_total / (f32)driven_count) : 0.0f;

        /* --- brake torque: MAGNITUDE only, same for all four wheels (no
         * front/rear bias split in Phase 1) -- see drivetrain.h's
         * DrivetrainWheelTorque comment: vehicle.c opposes this to each
         * wheel's own current spin direction, since drivetrain_update only
         * ever sees the driven-wheel AVERAGE spin, never each wheel's own
         * spin, and so has no way to know per-wheel which sign to apply. --- */
        brake_clamped = drivetrain_clampf(brake, 0.0f, 1.0f);
        brake_torque_mag = config->max_brake_torque * brake_clamped;

        for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
            out_wheel_torque[i].drive_torque = driven[i] ? per_wheel_drive : 0.0f;
            out_wheel_torque[i].brake_torque = brake_torque_mag;
        }
    }
}
