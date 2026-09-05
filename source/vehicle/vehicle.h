/*---------------------------------------------------------------------------------
 * vehicle.h -- the whole car: one RigidBody chassis, four wheels, and the
 * per-step order that ties suspension, tyres, drivetrain and rigid-body
 * integration together correctly.
 *
 * OWNER: vehicle. This is the CONTRACT header for Phase 1 -- every other
 * module under vehicle/ is written to be driven by vehicleStep() in exactly
 * the order below, and nothing outside vehicle.c should call
 * suspension_solve, tyre_solve or drivetrain_update directly.
 *
 *=================================================================================
 * THE CANONICAL PER-FRAME ORDER (vehicleStep, once per wheel where noted,
 * called once per PHYSICS_DT -- see core/timestep.h):
 *
 *   1. per-wheel suspension raycast
 *        suspension_solve's ground_query call, per wheel. Finds contact
 *        point/normal and raw compression for THIS step.
 *
 *   2. spring + damper force from compression and compression velocity
 *        Still inside suspension_solve: compression_velocity is computed
 *        from THIS step's compression vs the PREVIOUS step's
 *        (state->compression_prev), never from a value computed later in
 *        this same step.
 *
 *   3. normal load = suspension force, clamped at or above zero
 *        suspension_solve's return value / state->normal_load. This is the
 *        wheel's Fz for step 5, and it is fully known before step 4 even
 *        starts for any wheel -- see the "stale normal load" note below on
 *        why that ordering matters.
 *
 *   4. tyre slip ratio and slip angle from wheel spin vs contact-patch
 *      velocity
 *        vehicle.c computes these from Wheel.spin_velocity (this step's
 *        value, carried over from the END of the PREVIOUS step's step 8 --
 *        see the "lagged slip" note below) and
 *        rigidbody_point_velocity(chassis, contact_point).
 *
 *   5. tyre longitudinal and lateral force from slip, normal load, and
 *      surface
 *        tyre_solve(surface, {slip_ratio, slip_angle, normal_load}), where
 *        normal_load is THIS step's value from step 3, not a stale one.
 *
 *   6. sum all wheel forces and torques into the rigid body
 *        GRAVITY IS OWNED HERE, and nowhere else in the project. It is
 *        applied at the centre of mass at the top of this phase, before any
 *        wheel force. This phase is the only one that sums forces into the
 *        chassis and the only one that runs whether the car is grounded or
 *        airborne, so it is the only correct home; a caller cannot supply it
 *        either, because vehicleStep clears the accumulators first. An
 *        earlier revision of this list omitted gravity entirely and no module
 *        applied it -- the car simply never fell. Do not remove this line.
 *
 *        Then rigidbody_apply_force_at_point per wheel, using each wheel's
 *        step-5 force (rotated into world space) applied at its step-1
 *        contact point. Suspension force from step 2/3 and tyre force from
 *        step 5 are both applied here, after every wheel has finished
 *        steps 1-5 -- not interleaved wheel-by-wheel, so a later wheel's
 *        force computation can never see an already-partially-integrated
 *        chassis state from an earlier wheel in the SAME step.
 *
 *   7. integrate the rigid body (semi-implicit Euler)
 *        rigidbody_step(chassis, dt) -- exactly once per physics step,
 *        after all four wheels have contributed in step 6.
 *
 *   8. integrate wheel spin from drive/brake torque minus tyre-force
 *      reaction
 *        Uses drivetrain_update's per-wheel drive_torque/brake_torque
 *        (computed from THIS step's throttle/brake input, not deferred)
 *        MINUS the reaction torque from THIS step's tyre force (step 5's
 *        fx * wheel_radius) to update Wheel.spin_velocity for the NEXT
 *        step's step 4. Uses the chassis velocity/contact-patch velocity
 *        that PRODUCED this step's tyre force (i.e. the pre-step-7 state),
 *        never the NEW post-integration velocity from step 7 -- see the
 *        third ordering-bug note below.
 *
 *=================================================================================
 * THREE ORDERING BUGS THIS SEQUENCE EXISTS TO PREVENT
 *
 * (a) Stale normal load.
 *     If tyre force (step 5) were computed using a normal load left over
 *     from the PREVIOUS step instead of the freshly-solved one from step 3
 *     of THIS step, grip would lag load by one frame. At 120 Hz that is
 *     only ~8ms, but it is exactly the kind of one-frame lag that turns
 *     "compresses hard, tyre should grip harder right now" into "grips
 *     harder one physics tick after the load already changed" -- invisible
 *     as a number, but it shows up as suspension and grip feeling
 *     decoupled, especially over the washboard bump strip
 *     (world/testground.h) where load is changing every single step. Fixed
 *     here by computing normal load (steps 1-3) for ALL wheels before ANY
 *     wheel's tyre solve (step 4-5) runs, and by tyre_solve.h's contract
 *     explicitly requiring "this step's" normal_load (see tyre.h's
 *     TyreSlipInput comment).
 *
 * (b) One-frame-lagged slip feedback.
 *     Slip ratio depends on wheel spin velocity. If wheel spin were
 *     integrated (step 8) BEFORE tyre force was computed (step 4-5) in the
 *     same step -- e.g. by looping "update wheel spin, then compute tyre
 *     force" per wheel in one pass -- the tyre force used for THIS step's
 *     rigid-body integration would be reacting to a slip computed from a
 *     wheel speed that has already moved on to next step's value, while the
 *     wheel-spin integration in step 8 would then react to a tyre force
 *     that was itself computed from an even-older spin value. The two
 *     halves of the same physical feedback loop (tyre force <-> wheel spin)
 *     would be reading each other's state one frame apart in opposite
 *     directions, which is a classic source of numerical instability in
 *     vehicle sims (the wheel can appear to "hunt" between grip and slip
 *     with no driver input change). Fixed here by fixing the read/write
 *     order for the whole step: step 4 always reads the spin velocity that
 *     step 8 wrote at the END of the PREVIOUS step, and step 8 always
 *     writes the NEXT step's spin velocity using THIS step's tyre force --
 *     never both in the same pass per wheel.
 *
 * (c) Updating wheel spin from the body's NEW velocity rather than the one
 *     used for this frame's tyre force.
 *     Step 8 needs "the reaction torque from this step's tyre force," which
 *     was itself computed (step 5) from a contact-patch velocity taken
 *     BEFORE rigidbody_step (step 7) ran. If step 8 were implemented by
 *     re-reading chassis velocity AFTER step 7's integration -- e.g. by
 *     folding the wheel-spin update into the same loop that reads
 *     "current" chassis velocity for convenience -- it would integrate
 *     wheel spin against a chassis velocity that already includes THIS
 *     step's tyre force's own effect on the chassis, double-applying part
 *     of the same physical interaction within one step and biasing the
 *     wheel-spin/tyre-force feedback loop. Fixed here by step 8 using only
 *     values already computed earlier in the SAME step (the drive/brake
 *     torque from this step's input, and the tyre fx from step 5), and by
 *     never reading chassis->linear_velocity or angular_velocity again
 *     after step 7 has run for the rest of this vehicleStep call.
 *=================================================================================
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VEHICLE_VEHICLE_H
#define DIRT2_VEHICLE_VEHICLE_H

#include "core/types.h"
#include "core/rigidbody.h"
#include "vehicle/suspension.h"
#include "vehicle/tyre.h"
#include "vehicle/drivetrain.h"
#include "vehicle/vehicle_params.h"
#include "input/input.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Per-wheel runtime state that belongs to the vehicle as a whole rather than
 * to any one subsystem -- suspension.h owns SuspensionState, this struct
 * owns the spin/steer state that vehicle.c itself integrates in step 8. */
typedef struct Wheel {
    SuspensionState suspension;    /* owned by suspension.h, see that header */
    f32 spin_velocity;             /* rad/s, this wheel's own rotation about
                                     * its axle -- written by step 8, read by
                                     * step 4 of the NEXT step (see ordering
                                     * bug (b) above)                        */
    f32 steer_angle;               /* radians, current steering angle for
                                     * this wheel (0 for rear wheels in
                                     * Phase 1's front-steer-only setup).
                                     * SIGN: a rotation about the chassis up
                                     * axis by the right-hand rule, so in this
                                     * project's basis (x fwd, y up, z right)
                                     * POSITIVE steers the car LEFT. That is
                                     * the opposite of InputState.steer, whose
                                     * +1 is full RIGHT -- vehicle.c's
                                     * vehicle_phase_steering negates when it
                                     * converts, and getting that wrong ships
                                     * a car that steers backwards.          */
} Wheel;

/* One complete vehicle: a chassis rigid body, four wheels, its tuning
 * parameters, and a pointer to whatever ground query the suspension raycast
 * should use (see suspension.h's SuspensionGroundQuery) -- stored here
 * rather than passed into vehicleStep every call so the call site
 * (main.c's frame loop) doesn't have to thread world state through on every
 * single call. */
/* "What am I driving on at world (x, z)?" -- the tyre-grip counterpart to
 * SuspensionGroundQuery's "how high is the ground here?".
 *
 * A callback rather than a direct call into world/testground.h on purpose:
 * vehicle/ must not depend on world/, or the physics core stops being
 * testable without a world and the two layers grow into each other. The
 * caller supplies the adapter (see main.c).
 *
 * MUST be total -- every query returns a valid surface, never NULL. If it is
 * left NULL on the Vehicle itself, step 5 falls back to
 * params.default_tyre_surface, which means every surface in the world feels
 * identical. That is a legitimate configuration (the assertion suite uses
 * it), but it is NOT the shipping one. */
typedef const TyreSurfaceParams *(*VehicleSurfaceQuery)(void *userdata,
                                                         f32 world_x, f32 world_z);

typedef struct Vehicle {
    RigidBody chassis;
    Wheel wheels[VEHICLE_WHEEL_COUNT];      /* indexed by WheelIndex        */
    VehicleParams params;
    DrivetrainState drivetrain_state;

    SuspensionGroundQuery ground_query;
    void *ground_userdata;

    /* Optional; NULL means "use params.default_tyre_surface everywhere".
     * Shares ground_userdata -- both queries answer about the same world. */
    VehicleSurfaceQuery surface_query;
} Vehicle;

/* Sets up chassis (rigidbody_init from params.chassis_mass/inertia), zeroes
 * all wheel state, and stores the ground query callback/userdata for later
 * suspension raycasts. Does not set chassis position/orientation -- caller
 * places the vehicle afterwards via vehicle->chassis.position/orientation
 * directly. */
void vehicle_init(Vehicle *v, const VehicleParams *params,
                   SuspensionGroundQuery ground_query, void *ground_userdata);

/* Runs exactly one physics step of PHYSICS_DT seconds (core/timestep.h),
 * following the eight-step canonical order documented at the top of this
 * file, in that order, with no step reordered or interleaved differently
 * per wheel. `input` is this step's driver input (see input/input.h) --
 * already ramped/shaped, vehicle.c does not do any input smoothing itself.
 *
 * Called once per physics step owed, per the return value of
 * timestep_advance (core/timestep.h) -- never called with a variable dt
 * derived from render frame time. */
void vehicle_step(Vehicle *v, const InputState *input, f32 dt);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_VEHICLE_VEHICLE_H */
