/*---------------------------------------------------------------------------------
 * drivetrain.h -- engine, gearing and per-wheel drive/brake torque split.
 *
 * OWNER: vehicle. Feeds step 8 of vehicle.h's canonical order ("integrate
 * wheel spin from drive/brake torque minus tyre-force reaction") -- this
 * module computes how much drive or brake torque EACH wheel receives this
 * step; vehicle.c is the one that actually integrates wheel angular
 * velocity with it, because that integration also needs the tyre's
 * reaction torque (tyre.h's fx times wheel radius), which this module has
 * no visibility into.
 *
 * PHASE 1 SCOPE: a single fixed gear ratio (no gearbox/shifting logic yet)
 * and a simple RPM-to-torque curve, enough to drive the test plane. Gear
 * shifting, a clutch model and a differential are explicitly OUT of scope
 * for this header -- adding them now would be scope creep ahead of what
 * Phase 1 needs (drive over a bumpy plane, not race a stage), so the struct
 * below has room to grow (gear_ratio is a single f32, not an array) rather
 * than pre-building unused machinery.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VEHICLE_DRIVETRAIN_H
#define DIRT2_VEHICLE_DRIVETRAIN_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Which wheels receive engine drive torque. Phase 1 needs exactly one of
 * these to get the test vehicle moving; kept as an enum (not a hardcoded
 * assumption inside drivetrain.c) so vehicle_params.h can pick per car. */
typedef enum DriveLayout {
    DRIVE_FWD = 0,   /* front axle only        */
    DRIVE_RWD = 1,   /* rear axle only         */
    DRIVE_AWD = 2,   /* both axles, split evenly (no centre differential
                       * logic yet -- straight 50/50 torque split)          */
} DriveLayout;

typedef struct DrivetrainConfig {
    DriveLayout layout;
    f32 gear_ratio;        /* single fixed ratio, engine rev -> wheel rev,
                             * dimensionless (> 1 means the wheel turns
                             * slower than the engine)                      */
    f32 final_drive_ratio; /* additional fixed reduction after gear_ratio,
                             * same units, kept separate so either can be
                             * retuned independently when a gearbox is
                             * added later                                  */
    f32 max_engine_torque; /* N*m, peak of the RPM-to-torque curve          */
    f32 peak_torque_rpm;   /* RPM at which max_engine_torque occurs         */
    f32 max_rpm;           /* RPM the engine is limited to (rev limiter)    */
    f32 idle_rpm;          /* RPM the engine sits at with zero throttle,
                             * clutch engaged, wheels stationary            */
    f32 max_brake_torque;  /* N*m, per wheel, at brake_input == 1.0 -- see
                             * input.h's InputState.brake                   */
    f32 wheel_radius;      /* metres -- shared by drivetrain (converting
                             * engine RPM to wheel angular velocity for the
                             * clutch-slip model) and vehicle.c (converting
                             * wheel angular velocity to contact-patch
                             * speed for tyre.h's slip ratio)                */
} DrivetrainConfig;

/* Runtime engine state, persisted frame to frame. */
typedef struct DrivetrainState {
    f32 engine_rpm;
} DrivetrainState;

/* Per-wheel torque request for this step -- what drivetrain_update decided
 * each wheel should receive, before vehicle.c integrates wheel spin with
 * it (step 8). Indexed by WheelIndex (core/types.h); a wheel with no drive
 * and no brake applied this step gets drive_torque == 0 and
 * brake_torque == 0 in its slot, not an omitted entry -- callers always
 * read all VEHICLE_WHEEL_COUNT slots. */
typedef struct DrivetrainWheelTorque {
    f32 drive_torque;  /* N*m, sign matches intended direction of travel   */
    f32 brake_torque;  /* N*m, MAGNITUDE only -- always >= 0; vehicle.c is
                         * responsible for opposing it to the wheel's
                         * current spin direction, since brake torque always
                         * resists motion regardless of which way the wheel
                         * is currently turning                             */
} DrivetrainWheelTorque;

/* Advances the engine model by `dt` using `throttle` (0..1, see input.h's
 * ramped throttle) and the current average driven-wheel angular velocity
 * `driven_wheel_omega_avg` (rad/s, computed by vehicle.c from whichever
 * wheels config->layout says are driven -- drivetrain.c does not know which
 * WheelIndex slots those are, only the caller does), updates
 * `state->engine_rpm`, and fills `out_wheel_torque[VEHICLE_WHEEL_COUNT]`
 * with each wheel's drive_torque (zero for non-driven wheels) and
 * brake_torque (from `brake`, 0..1, applied to every wheel per
 * config->max_brake_torque -- Phase 1 has no front/rear brake bias split).
 */
void drivetrain_update(const DrivetrainConfig *config, DrivetrainState *state,
                        f32 throttle, f32 brake, f32 driven_wheel_omega_avg,
                        f32 dt, DrivetrainWheelTorque out_wheel_torque[]);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_VEHICLE_DRIVETRAIN_H */
