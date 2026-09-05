/*---------------------------------------------------------------------------------
 * vehicle.c -- the whole car: one RigidBody chassis, four wheels, and the
 * eight-step canonical per-frame order documented at length in vehicle.h.
 *
 * HOW THE ORDER IS ENFORCED HERE (read this before editing anything below).
 *   vehicle.h's three ordering bugs are all "one loop that happens to be in
 *   the right order today, until someone merges two passes for convenience".
 *   So this file does NOT implement vehicle_step as one per-wheel loop. Each
 *   phase of the canonical order is a separate static function with a
 *   separate per-wheel loop, and the intermediate per-wheel results are
 *   carried between phases in a local WheelStepScratch array. The phase
 *   boundaries are therefore function-call boundaries, which is what makes
 *   the order visible instead of incidental.
 *
 *   On top of that, each phase function's SIGNATURE is chosen so the bug it
 *   prevents cannot even be written inside it:
 *
 *     bug (a) stale normal load
 *         vehicle_phase_suspension (steps 1-3) runs its own complete
 *         four-wheel loop and writes every wheel's normal_load into scratch
 *         BEFORE vehicle_phase_tyres (steps 4-5) is called at all. Step 5
 *         reads scratch[i].normal_load, which physically cannot be a
 *         previous step's value: the scratch array is a fresh local of
 *         vehicle_step, so it holds nothing at all from last step.
 *
 *     bug (b) one-frame-lagged slip feedback
 *         vehicle_phase_tyres takes `const Wheel wheels[]`. Step 4 therefore
 *         cannot write Wheel.spin_velocity even by accident -- the compiler
 *         rejects it. Wheel.spin_velocity is written in exactly one place in
 *         this file, vehicle_phase_wheel_spin (step 8), which runs last.
 *
 *     bug (c) wheel spin integrated from the post-integration body velocity
 *         vehicle_phase_wheel_spin (step 8) takes NO RigidBody parameter of
 *         any kind. It has no access to the chassis, so it cannot read
 *         linear_velocity or angular_velocity after rigidbody_step (step 7)
 *         has run, and cannot call rigidbody_point_velocity either. It works
 *         only from values computed earlier in the same step (scratch's
 *         step-5 tyre fx, and this step's drive/brake torque). Do not add a
 *         RigidBody parameter to it "just to read the speed" -- that
 *         parameter's absence IS the fix.
 *
 *   The two rules that keep the above true if this file is edited:
 *     1. A phase never grows a second responsibility. If a new quantity is
 *        needed by a later phase, it goes in WheelStepScratch, it does not
 *        get recomputed in the later phase from live chassis state.
 *     2. No phase function's parameter list gets loosened (const removed,
 *        RigidBody added) to make something convenient.
 *---------------------------------------------------------------------------------*/
#include "vehicle/vehicle.h"
#include "core/vecmath.h"

#include <math.h>
/* NULL. The 3DS build gets it free via libctru's include chain, so leaving
 * this out still cross-compiles clean and only fails on the host build --
 * exactly the kind of one-sided break the two-target split exists to catch. */
#include <stddef.h>

/*---------------------------------------------------------------------------------
 * PHASE 1 PLACEHOLDER CONSTANTS.
 *
 * These are file-local on purpose. vehicle_params.h is owned by another
 * module and deliberately ships no tuning numbers (see its header comment),
 * and it currently has no field for any of these three. They are the minimum
 * needed for vehicle_step to be implementable at all, and they are marked
 * here so they are easy to find and delete the moment vehicle_params.h grows
 * real fields for them. They are NOT tuned or playtested.
 *---------------------------------------------------------------------------------*/

/* VEHICLE_MAX_STEER_ANGLE_RAD and VEHICLE_SLIP_SPEED_FLOOR used to live here
 * as unparameterised local constants. They are now real VehicleParams fields
 * (max_steer_angle, slip_speed_floor) and these #defines are deliberately
 * gone -- if you find a stray reference to either name, it is a leftover, not
 * a fallback. */

/* Slip ratio is clamped to +/- this before reaching tyre_solve. Past the
 * tyre curve's sliding tail the exact value carries no more information, and
 * clamping keeps a locked or wildly spinning wheel from feeding a huge
 * number into the curve lookup. */
#define VEHICLE_SLIP_RATIO_LIMIT 4.0f

/* Standard gravity, m/s^2, downward along world -Y. Lives in vehicle.c
 * because vehicle_phase_apply_forces (step 6) is the sole owner of gravity in
 * this project -- see the comment there. If a later phase ever needs a
 * different value (low-gravity tracks, a debug flier mode), promote this to
 * VehicleParams rather than adding a second gravity somewhere else. */
#define VEHICLE_GRAVITY_MS2 9.81f

/*---------------------------------------------------------------------------------
 * Per-wheel intermediate results, carried BETWEEN phases of one vehicle_step.
 *
 * This exists so the phases can be separate loops. It is a local of
 * vehicle_step, freshly indeterminate every step and fully written by the
 * phases that own each field before any phase reads it -- which is exactly
 * why a stale previous-step value (ordering bug (a)) cannot appear in it.
 *---------------------------------------------------------------------------------*/
typedef struct WheelStepScratch {
    /* written by steps 1-3 (vehicle_phase_suspension) */
    Vec3 suspension_force;   /* world space, N, from suspension_solve       */
    Vec3 contact_point;      /* world space, valid only if grounded         */
    Vec3 contact_normal;     /* world space unit vector, if grounded        */
    Vec3 force_point;        /* where step 6 applies this wheel's forces --
                               * the contact point when grounded, the wheel
                               * position otherwise (nothing is applied when
                               * airborne, but the point stays well-defined) */
    bool grounded;
    f32  normal_load;        /* N, THIS step's Fz for step 5                */

    /* written by steps 4-5 (vehicle_phase_tyres) */
    f32  tyre_fx;            /* N, wheel-longitudinal -- step 8's reaction
                               * torque source, captured BEFORE step 7 runs  */
    Vec3 tyre_force_world;   /* fx*forward + fy*lateral, rotated to world    */
} WheelStepScratch;

/* Body-space axes. core/rigidbody.h documents the chassis convention as
 * x forward, y up, z right; this file follows that header rather than
 * inventing a second convention. */
static Vec3 vehicle_body_forward(void) { return vec3_make(1.0f, 0.0f, 0.0f); }
static Vec3 vehicle_body_up(void)      { return vec3_make(0.0f, 1.0f, 0.0f); }

static f32 vehicle_clampf(f32 value, f32 lo, f32 hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* Which wheels this drive layout sends engine torque to. drivetrain.c does
 * not know which WheelIndex slots an axle owns (see drivetrain.h) -- this is
 * the caller-side knowledge that header refers to. */
static bool vehicle_wheel_is_driven(DriveLayout layout, WheelIndex wheel) {
    bool front = (wheel == WHEEL_FL || wheel == WHEEL_FR);
    switch (layout) {
        case DRIVE_FWD: return front;
        case DRIVE_RWD: return !front;
        case DRIVE_AWD: return true;
        default:        return false;
    }
}

/*---------------------------------------------------------------------------------
 * step 0 -- not part of the numbered contract.
 *
 * Turns this step's steering input into per-wheel steer angles. Runs before
 * step 1 because step 4 needs each wheel's heading, and because vehicle.h is
 * explicit that a step's input must be this step's input, not last step's
 * (see input.h's note on input_update being called before vehicle_step).
 * Phase 1 is front-steer only, so the rear wheels are pinned to zero rather
 * than left at whatever they happened to hold.
 *---------------------------------------------------------------------------------*/
static void vehicle_phase_steering(const VehicleParams *params, Wheel wheels[],
                                    const InputState *input) {
    /* NOTE THE MINUS. InputState.steer is +1 for full RIGHT (input.h), while
     * Wheel.steer_angle is a rotation about the chassis up axis by the
     * right-hand rule -- and in this project's basis (x forward, y up,
     * z right) a POSITIVE rotation about +Y swings forward toward -Z, which
     * is the car's LEFT. Same handedness fact that makes place_car_at_start
     * use a -90 degree yaw to face world +Z.
     *
     * Without the minus the car steers exactly backwards, which is what
     * shipped in v1.0.0 and what steve hit on hardware: "whenever I move the
     * circle pad left, I turn right; whenever I move the circle pad right, I
     * turn left." Measured, not reasoned: at 12.4 m/s with full right lock,
     * dot(forward_after_2s, right_before) was -0.925 -- a hard left. */
    f32 steer = vehicle_clampf(input->steer, -1.0f, 1.0f);
    wheels[WHEEL_FL].steer_angle = -steer * params->max_steer_angle;
    wheels[WHEEL_FR].steer_angle = -steer * params->max_steer_angle;
    wheels[WHEEL_RL].steer_angle = 0.0f;
    wheels[WHEEL_RR].steer_angle = 0.0f;
}

/*---------------------------------------------------------------------------------
 * steps 1-3 -- suspension raycast, spring+damper force, normal load.
 *
 * All three happen inside suspension_solve (see suspension.h). This loop runs
 * to completion for ALL FOUR wheels before vehicle_phase_tyres is called even
 * once: that is the fix for ordering bug (a).
 *
 * `chassis` is const -- this phase reads the chassis pose to place the wheels
 * and hands the same const pointer to suspension_solve; it cannot integrate
 * or otherwise disturb the body mid-phase, so wheel 3's solve sees exactly
 * the chassis state wheel 0's solve saw.
 *---------------------------------------------------------------------------------*/
static void vehicle_phase_suspension(const VehicleParams *params,
                                      Wheel wheels[],
                                      const RigidBody *chassis,
                                      SuspensionGroundQuery ground_query,
                                      void *ground_userdata,
                                      f32 dt,
                                      WheelStepScratch scratch[]) {
    int i;
    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        const SuspensionConfig *config = &params->suspension[i];
        SuspensionState *state = &wheels[i].suspension;
        WheelStepScratch *s = &scratch[i];

        /* step 1's ray origin: the wheel's mount point taken from body space
         * into world space. Only vehicle.c knows the chassis pose, which is
         * why suspension.h takes this as a parameter (see that header). */
        Vec3 wheel_pos_world =
            vec3_add(chassis->position,
                     vec3_rotate_by_quat(config->mount_point_body, chassis->orientation));

        /* steps 1, 2 and 3 */
        s->suspension_force = suspension_solve(config, state, chassis, wheel_pos_world,
                                                ground_query, ground_userdata, dt);

        /* Snapshot step 3's result into scratch. Steps 4-8 read the snapshot
         * rather than the live SuspensionState, so there is exactly one
         * moment at which this step's normal load is captured. */
        s->grounded       = state->grounded;
        s->normal_load    = state->normal_load;
        s->contact_point  = state->contact_point;
        s->contact_normal = state->contact_normal;
        s->force_point    = s->grounded ? state->contact_point : wheel_pos_world;

        /* Owned by the next phase; zeroed here so no field of scratch is
         * ever read indeterminate if that phase skips this wheel. */
        s->tyre_fx          = 0.0f;
        s->tyre_force_world = vec3_zero();
    }
}

/*---------------------------------------------------------------------------------
 * steps 4-5 -- slip ratio / slip angle, then combined-slip tyre force.
 *
 * `wheels` is const: step 4 READS Wheel.spin_velocity (the value step 8 of
 * the PREVIOUS step wrote) and must not write it. That const is the
 * compile-time form of ordering bug (b)'s fix.
 *
 * `chassis` is const and this phase runs strictly before step 7, so
 * rigidbody_point_velocity here returns the pre-integration contact-patch
 * velocity -- the same one that produces this step's tyre force, which is
 * the value step 8 is required to have been driven by (ordering bug (c)).
 *
 * SIGN CONVENTIONS (tyre.h leaves fy's sign "consistent with slip_angle's"):
 *   slip_ratio = (omega*R - v_long) / max(|v_long|, floor). Positive means
 *   the wheel is outrunning the ground, and tyre_solve returns fx with the
 *   sign of slip_ratio, so a positive slip ratio pushes the car forward --
 *   correct.
 *   slip_angle = atan2(-v_lat, max(|v_long|, floor)). The minus sign is what
 *   makes fy follow the same rule: friction must oppose the contact patch's
 *   sideways motion, so a patch sliding to the wheel's right (+v_lat) needs a
 *   force to the left, i.e. a negative fy, which is what tyre_solve returns
 *   for the negative slip angle this produces.
 *---------------------------------------------------------------------------------*/
static void vehicle_phase_tyres(const VehicleParams *params,
                                 const Wheel wheels[],
                                 const RigidBody *chassis,
                                 VehicleSurfaceQuery surface_query,
                                 void *surface_userdata,
                                 WheelStepScratch scratch[]) {
    const f32 wheel_radius = params->drivetrain.wheel_radius;
    Vec3 chassis_forward = vec3_rotate_by_quat(vehicle_body_forward(), chassis->orientation);
    Vec3 chassis_up      = vec3_rotate_by_quat(vehicle_body_up(), chassis->orientation);
    int i;

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        WheelStepScratch *s = &scratch[i];
        Quat steer_rotation;
        Vec3 heading, forward, lateral, patch_velocity;
        Vec3 forward_unprojected;
        f32 v_long, v_lat, denominator, slip_ratio, slip_angle;
        TyreSlipInput slip;
        TyreForceOutput force;

        /* An airborne wheel has no contact patch: no slip is defined and no
         * tyre force exists. Its scratch tyre fields keep the zeros
         * vehicle_phase_suspension wrote. */
        if (!s->grounded) continue;

        /* The wheel's heading: chassis forward, steered about the chassis's
         * own up axis, then projected into the contact plane so slip is
         * measured in the plane the tyre is actually touching. */
        steer_rotation = quat_from_axis_angle(chassis_up, wheels[i].steer_angle);
        heading = vec3_rotate_by_quat(chassis_forward, steer_rotation);
        forward_unprojected =
            vec3_sub(heading, vec3_scale(s->contact_normal,
                                          vec3_dot(heading, s->contact_normal)));
        /* Degenerate only if the wheel is pointed straight into the surface,
         * which a sane contact normal never produces -- skip rather than
         * normalise a zero vector into a direction that means nothing. */
        if (vec3_length_sq(forward_unprojected) < 1e-8f) continue;
        forward = vec3_normalize(forward_unprojected);
        lateral = vec3_normalize(vec3_cross(forward, s->contact_normal)); /* +right */

        /* step 4 */
        patch_velocity = rigidbody_point_velocity(chassis, s->contact_point);
        v_long = vec3_dot(patch_velocity, forward);
        v_lat  = vec3_dot(patch_velocity, lateral);

        denominator = fabsf(v_long);
        if (denominator < params->slip_speed_floor) denominator = params->slip_speed_floor;

        slip_ratio = (wheels[i].spin_velocity * wheel_radius - v_long) / denominator;
        slip_ratio = vehicle_clampf(slip_ratio, -VEHICLE_SLIP_RATIO_LIMIT,
                                     VEHICLE_SLIP_RATIO_LIMIT);
        slip_angle = atan2f(-v_lat, denominator);

        /* step 5 -- normal_load is THIS step's, from step 3 above, never a
         * carried-over value (ordering bug (a)). */
        slip.slip_ratio  = slip_ratio;
        slip.slip_angle  = slip_angle;
        slip.normal_load = s->normal_load;

        /* The surface is queried AT THIS WHEEL'S OWN CONTACT POINT, not at
         * the chassis centre -- with a 2.6 m wheelbase the front wheels can
         * be on tarmac while the rears are still on gravel, and that
         * split-mu moment is most of what makes a surface change feel like
         * anything. Querying once for the whole car would erase it.
         *
         * A NULL query, or one that returns NULL, falls back to the single
         * default surface. That fallback is why this needs saying out loud:
         * before this call existed, testground_surface_at was written,
         * tested and reachable from nothing, so tarmac and gravel felt
         * identical no matter what the world said. */
        {
            const TyreSurfaceParams *surface = NULL;
            if (surface_query) {
                surface = surface_query(surface_userdata,
                                         s->contact_point.x, s->contact_point.z);
            }
            if (!surface) surface = &params->default_tyre_surface;
            force = tyre_solve(surface, slip);
        }

        s->tyre_fx = force.fx;
        s->tyre_force_world = vec3_add(vec3_scale(forward, force.fx),
                                        vec3_scale(lateral, force.fy));
    }
}

/*---------------------------------------------------------------------------------
 * step 6 -- sum every wheel's forces and torques into the rigid body.
 *
 * Runs only after all four wheels have finished steps 1-5, so no wheel's
 * force was computed against a chassis that an earlier wheel in the SAME step
 * had already disturbed. Nothing in this phase computes a force; it only
 * applies forces computed in earlier phases, which is what keeps that true.
 *
 * The anti-roll bar is applied here rather than in the suspension phase
 * because it needs BOTH wheels of an axle solved first -- it is a function of
 * the compression difference across the axle. It is applied only when both
 * wheels of the axle are grounded: with one wheel in the air there is no
 * contact point to react the bar's force against, and pushing on a
 * non-contact would be inventing a force out of nothing.
 *---------------------------------------------------------------------------------*/
static void vehicle_phase_apply_forces(RigidBody *chassis,
                                        const VehicleParams *params,
                                        const Wheel wheels[],
                                        const WheelStepScratch scratch[]) {
    static const WheelIndex axle_left[VEHICLE_AXLE_COUNT]  = { WHEEL_FL, WHEEL_RL };
    static const WheelIndex axle_right[VEHICLE_AXLE_COUNT] = { WHEEL_FR, WHEEL_RR };
    int grounded_count = 0;
    int i;

    /* Gravity, applied at the centre of mass (no torque by definition) before
     * any wheel force. It belongs HERE, in step 6, and nowhere else: this is
     * the only phase that sums forces into the chassis, and it is the only one
     * that runs whether the car is grounded or airborne. It cannot be injected
     * by the caller -- vehicle_step calls rigidbody_clear_accumulators first,
     * so anything a caller accumulated before the call is erased. */
    chassis->force_accum.y -= params->chassis_mass * VEHICLE_GRAVITY_MS2;

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        const WheelStepScratch *s = &scratch[i];
        if (!s->grounded) continue;
        grounded_count++;
        rigidbody_apply_force_at_point(chassis, s->suspension_force, s->force_point);
        rigidbody_apply_force_at_point(chassis, s->tyre_force_world, s->contact_point);
    }

    for (i = 0; i < VEHICLE_AXLE_COUNT; i++) {
        WheelIndex l = axle_left[i];
        WheelIndex r = axle_right[i];
        Vec3 left_force, right_force;
        if (!scratch[l].grounded || !scratch[r].grounded) continue;
        suspension_antiroll(&wheels[l].suspension, &wheels[r].suspension,
                             params->antiroll_rate[i],
                             scratch[l].contact_normal, scratch[r].contact_normal,
                             &left_force, &right_force);
        rigidbody_apply_force_at_point(chassis, left_force, scratch[l].contact_point);
        rigidbody_apply_force_at_point(chassis, right_force, scratch[r].contact_point);
    }

    /* Airborne auto-level: suspension.h's contract is that the CALLER decides
     * all four wheels are off the ground and this function trusts it. */
    if (grounded_count == 0) {
        Vec3 autolevel = suspension_airborne_autolevel(chassis,
                                                        params->autolevel_strength,
                                                        params->autolevel_damping);
        chassis->torque_accum = vec3_add(chassis->torque_accum, autolevel);
    }
}

/*---------------------------------------------------------------------------------
 * step 8 -- integrate wheel spin from drive/brake torque minus tyre reaction.
 *
 * NOTE THE PARAMETER LIST: there is no RigidBody here, const or otherwise.
 * That is deliberate and load-bearing. This phase runs AFTER rigidbody_step
 * (step 7), so any chassis velocity it could reach would already include this
 * step's own tyre force's effect on the body; using it would double-apply
 * half of the tyre-force/wheel-spin feedback loop inside one step (vehicle.h
 * ordering bug (c)). By taking no chassis at all, this function is
 * structurally incapable of doing that -- it can only use scratch[i].tyre_fx,
 * captured in step 5 from the PRE-integration state, plus this step's
 * drive/brake torque.
 *
 * Brake torque is a magnitude (drivetrain.h) and always opposes the wheel's
 * current spin direction. It is applied as a limited delta so a brake can
 * bring a wheel to rest but never drive it backwards inside a single step --
 * an unlimited brake delta at a low physics rate makes a stopping wheel
 * oscillate between forward and reverse spin every step.
 *---------------------------------------------------------------------------------*/
static void vehicle_phase_wheel_spin(const VehicleParams *params,
                                      DrivetrainState *drivetrain_state,
                                      Wheel wheels[],
                                      const InputState *input,
                                      const WheelStepScratch scratch[],
                                      f32 dt) {
    DrivetrainWheelTorque wheel_torque[VEHICLE_WHEEL_COUNT];
    const f32 wheel_radius = params->drivetrain.wheel_radius;
    f32 inv_inertia = (params->wheel_mass_moment_of_inertia > 0.0f)
                      ? (1.0f / params->wheel_mass_moment_of_inertia) : 0.0f;
    f32 driven_omega_sum = 0.0f;
    int driven_count = 0;
    int i;

    /* Average spin of the DRIVEN wheels only, from the values still standing
     * from the previous step's step 8 -- this is the same set of values step
     * 4 read this step, so the engine and the tyres agree about wheel speed
     * within a step. */
    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        if (!vehicle_wheel_is_driven(params->drivetrain.layout, (WheelIndex)i)) continue;
        driven_omega_sum += wheels[i].spin_velocity;
        driven_count++;
    }

    drivetrain_update(&params->drivetrain, drivetrain_state,
                       input->throttle, input->brake,
                       (driven_count > 0) ? (driven_omega_sum / (f32)driven_count) : 0.0f,
                       dt, wheel_torque);

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        f32 net_torque, spin, brake_torque, brake_delta;

        /* Drive torque in, tyre reaction out. scratch[i].tyre_fx is step 5's
         * value from BEFORE step 7 integrated the chassis. */
        net_torque = wheel_torque[i].drive_torque - scratch[i].tyre_fx * wheel_radius;
        spin = wheels[i].spin_velocity + net_torque * inv_inertia * dt;

        brake_torque = wheel_torque[i].brake_torque;
        if (brake_torque < 0.0f) brake_torque = 0.0f;
        /* Handbrake is a rear-axle lock (input.h: "an abrupt, fully-on
         * rear-lock input for a rally-style pull"). drivetrain_update has no
         * handbrake parameter, so vehicle.c owns this. It has its own torque
         * rather than reusing max_brake_torque, and it is deliberately the
         * larger of the two: a handbrake pull must reliably lock the rear
         * even at a speed where the footbrake alone would not, or the pull
         * does nothing and the rally-style rotation never happens. */
        if (input->handbrake && (i == WHEEL_RL || i == WHEEL_RR)) {
            brake_torque += params->handbrake_torque;
        }

        brake_delta = brake_torque * inv_inertia * dt;
        if (spin > 0.0f) {
            spin -= brake_delta;
            if (spin < 0.0f) spin = 0.0f;
        } else if (spin < 0.0f) {
            spin += brake_delta;
            if (spin > 0.0f) spin = 0.0f;
        }

        /* The ONE write of spin_velocity in this file. Read next step by
         * step 4 (vehicle_phase_tyres), never earlier -- ordering bug (b). */
        wheels[i].spin_velocity = spin;
    }
}

/*---------------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------------*/

void vehicle_init(Vehicle *v, const VehicleParams *params,
                   SuspensionGroundQuery ground_query, void *ground_userdata) {
    int i;
    if (!v) return;

    if (params) {
        v->params = *params;
        rigidbody_init(&v->chassis, params->chassis_mass, params->chassis_inertia_diag);
    }

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        Wheel *w = &v->wheels[i];
        w->spin_velocity = 0.0f;
        w->steer_angle = 0.0f;
        w->suspension.compression = 0.0f;
        w->suspension.compression_prev = 0.0f;
        w->suspension.compression_velocity = 0.0f;
        w->suspension.grounded = false;
        w->suspension.contact_point = vec3_zero();
        w->suspension.contact_normal = vec3_make(0.0f, 1.0f, 0.0f);
        w->suspension.normal_load = 0.0f;
    }

    v->drivetrain_state.engine_rpm = 0.0f;
    v->ground_query = ground_query;
    v->ground_userdata = ground_userdata;
    /* NULL until the caller sets it (vehicle.h) -- every surface then reads
     * as params.default_tyre_surface. Zeroed here rather than left as
     * whatever the caller's stack held, because an uninitialised function
     * pointer called once per wheel per step is a crash, not a wrong number. */
    v->surface_query = NULL;
}

void vehicle_step(Vehicle *v, const InputState *input, f32 dt) {
    /* Fresh every step. Nothing here survives from the previous step, which
     * is what makes a stale normal load (ordering bug (a)) unrepresentable
     * rather than merely avoided. */
    WheelStepScratch scratch[VEHICLE_WHEEL_COUNT];

    if (!v || !input) return;

    /* step 0 (outside the numbered contract): this step's accumulators and
     * this step's steer angles. */
    rigidbody_clear_accumulators(&v->chassis);
    vehicle_phase_steering(&v->params, v->wheels, input);

    /* steps 1-3, all four wheels */
    vehicle_phase_suspension(&v->params, v->wheels, &v->chassis,
                              v->ground_query, v->ground_userdata, dt, scratch);

    /* steps 4-5, all four wheels -- only now that every normal load is known */
    vehicle_phase_tyres(&v->params, v->wheels, &v->chassis,
                         v->surface_query, v->ground_userdata, scratch);

    /* step 6 */
    vehicle_phase_apply_forces(&v->chassis, &v->params, v->wheels, scratch);

    /* step 7 -- exactly once. After this line, no chassis velocity may be
     * read again for the rest of this call; the next line's parameter list
     * is what guarantees it. */
    rigidbody_step(&v->chassis, dt);

    /* step 8 -- no chassis parameter, by design (ordering bug (c)) */
    vehicle_phase_wheel_spin(&v->params, &v->drivetrain_state, v->wheels,
                              input, scratch, dt);
}
