/*---------------------------------------------------------------------------------
 * suspension.c -- per-wheel raycast spring/damper suspension, anti-roll bar,
 * bottom-out handling, and an airborne auto-level PD torque. See
 * suspension.h for the full contract; this file implements it, no redesign.
 *---------------------------------------------------------------------------------*/
#include "vehicle/suspension.h"
#include "core/vecmath.h"

Vec3 suspension_solve(const SuspensionConfig *config, SuspensionState *state,
                       const RigidBody *chassis, Vec3 wheel_pos_world,
                       SuspensionGroundQuery ground_query, void *ground_userdata,
                       f32 dt) {
    /* `chassis` is not read here: per this file's ordering contract,
     * compression_velocity comes ONLY from this step's compression versus
     * state->compression_prev (finite difference), never from
     * rigidbody_point_velocity -- see the compression_velocity note below.
     * Kept as a parameter per suspension.h's contract (and for symmetry with
     * how vehicle.c calls this once per wheel), but suspension.c does not
     * touch its position/orientation/velocity fields. */
    (void)chassis;

    if (!state) {
        return vec3_zero();
    }

    /* ---- 1. raycast: vertical column query at the wheel's current (x, z).
     * The ground query is a pure heightfield lookup (world_x, world_z) ->
     * height, not a general ray -- so the probe direction is always straight
     * down in world space (matches world/testground.h's contract exactly,
     * see suspension.h's header comment on RAYCAST SOURCE). */
    f32 ground_height = 0.0f;
    Vec3 ground_normal = vec3_make(0.0f, 1.0f, 0.0f);
    bool found = ground_query &&
                 ground_query(ground_userdata, wheel_pos_world.x, wheel_pos_world.z,
                              &ground_height, &ground_normal);

    /* Distance from the mount point straight down to the ground. Probe out
     * to rest_length + max_travel (suspension.h's documented "found ground
     * within rest_length + max_travel" grounded condition) -- this is a
     * slightly more generous reach than the loaded range [0, rest_length],
     * so a wheel just barely out of spring range still reports contact
     * (contact_point/normal valid, tyre.h still gets a contact patch to
     * compute slip against) even though its LOADED compression clamps to
     * zero (see below) and it therefore carries no normal load. */
    f32 dist_to_ground = wheel_pos_world.y - ground_height;
    f32 probe_dist = config->rest_length + config->max_travel;
    /* Only an UPPER bound on reach (suspension.h: "found ground within
     * rest_length + max_travel") -- no lower bound. dist_to_ground can go
     * negative when the wheel mount is compressed deep enough to sit below
     * the ground sample (compression pushed past rest_length, well into
     * bottom-out territory); that is not "airborne", it is the deepest kind
     * of grounded contact, and must keep flowing into the compression/force
     * formula below with no discontinuity. Requiring dist_to_ground >= 0.0f
     * here was wrong: it silently declared the wheel airborne (force snaps
     * to exactly zero) the instant compression exceeded rest_length, which
     * is exactly the kind of single-sample force cliff this module's
     * bottom-out contract exists to avoid. */
    bool grounded = found && dist_to_ground <= probe_dist;

    /* ---- 2. compression for THIS step.
     * compression = rest_length - dist_to_ground: ground closer than the
     * fully-extended length means the spring is pushed in by that much.
     * Floored at zero -- "0 = fully extended" (suspension.h's SuspensionState
     * doc) is a hard floor, the suspension cannot extend past rest_length,
     * so a wheel that is technically within the generous grounded probe
     * range but farther than rest_length away just carries zero compression
     * (and, via the spring formula below, zero spring force). No ceiling is
     * applied here -- overtravel past max_travel is handled by the bottom-out
     * term in step 3/4, not by clamping compression itself. */
    f32 new_compression;
    if (grounded) {
        f32 raw = config->rest_length - dist_to_ground;
        new_compression = (raw < 0.0f) ? 0.0f : raw;
    } else {
        /* Airborne: nothing is pushing the wheel in, so it sits at its
         * fully-extended position -- compression snaps to zero rather than
         * being integrated back down over time (this module has no
         * independent wheel-position dynamics beyond the raycast; see the
         * task report for why this simplification is acceptable for
         * Phase 1). */
        new_compression = 0.0f;
    }

    /* Compression velocity MUST be this step's freshly-solved compression
     * versus the PREVIOUS step's stored value, read from
     * state->compression_prev BEFORE it is overwritten below -- never from
     * state->compression re-derived later in this same call, and never from
     * a value this same step computes further down (e.g. a "corrected"
     * compression after bottom-out). Using anything else here reintroduces
     * exactly the kind of one-step staleness/lag bug vehicle.h's ordering
     * comment warns about for normal load and slip. */
    f32 prev_compression = state->compression_prev;
    f32 compression_velocity = (dt > 0.0f) ? (new_compression - prev_compression) / dt : 0.0f;

    /* Commit state for this step, and roll `compression_prev` forward so the
     * NEXT call's finite difference reads what THIS step just computed. */
    state->compression_prev = new_compression;
    state->compression = new_compression;
    state->compression_velocity = compression_velocity;
    state->grounded = grounded;

    if (!grounded) {
        state->normal_load = 0.0f;
        return vec3_zero();
    }

    state->contact_point = vec3_make(wheel_pos_world.x, ground_height, wheel_pos_world.z);
    state->contact_normal = ground_normal;

    /* ---- 3. spring force, with bottom-out handled as a BILINEAR curve, not
     * a hard clamp: up to max_travel the spring rate is config->spring_rate;
     * past it, an additional (much stiffer) config->bottom_out_spring_rate
     * applies only to the overtravel. The two halves agree exactly at
     * compression == max_travel (both evaluate to spring_rate * max_travel),
     * so the FORCE is continuous there -- only the slope (effective
     * stiffness) increases. A hard clamp on compression instead would hold
     * force constant right at the point of hardest impact, i.e. exactly the
     * discontinuous "impulse spike" this project's suspension is required to
     * avoid; the progressive extra term keeps pushing back harder the
     * further the suspension is driven past max_travel, with no jump. */
    f32 spring_force;
    if (new_compression <= config->max_travel) {
        spring_force = config->spring_rate * new_compression;
    } else {
        f32 overtravel = new_compression - config->max_travel;
        spring_force = config->spring_rate * config->max_travel
                     + config->bottom_out_spring_rate * overtravel;
    }

    /* ---- 4. damper force, asymmetric: compressing uses damper_compression,
     * extending (or momentarily still) uses damper_rebound -- see
     * suspension.h's SuspensionConfig comment on why real dampers (and
     * believable ones) are firmer on rebound than compression. */
    f32 damper_rate = (compression_velocity >= 0.0f) ? config->damper_compression
                                                       : config->damper_rebound;
    f32 damper_force = damper_rate * compression_velocity;

    /* ---- 5. normal load = Fs + Fd, clamped at or above zero -- a
     * suspension can only push the chassis away from the ground, never pull
     * it down (vehicle.h step 3). */
    f32 total_force = spring_force + damper_force;
    if (total_force < 0.0f) {
        total_force = 0.0f;
    }

    state->normal_load = total_force;

    return vec3_scale(ground_normal, total_force);
}

void suspension_antiroll(const SuspensionState *left, const SuspensionState *right,
                          f32 bar_rate, Vec3 left_normal, Vec3 right_normal,
                          Vec3 *out_left_force, Vec3 *out_right_force) {
    f32 diff = 0.0f;
    if (left && right) {
        diff = left->compression - right->compression;
    }

    /* F_arb = k_arb * (compression_left - compression_right), applied along
     * each wheel's own contact normal, in opposite directions across the
     * axle -- the more-compressed side gets more push, the less-compressed
     * side gets correspondingly less, resisting the roll without changing
     * the axle's total vertical load (see suspension.h's doc comment). */
    f32 f_arb = bar_rate * diff;

    if (out_left_force) {
        *out_left_force = vec3_scale(left_normal, f_arb);
    }
    if (out_right_force) {
        *out_right_force = vec3_scale(right_normal, -f_arb);
    }
}

Vec3 suspension_airborne_autolevel(const RigidBody *chassis, f32 strength, f32 damping) {
    if (!chassis) {
        return vec3_zero();
    }

    const Vec3 world_up = vec3_make(0.0f, 1.0f, 0.0f);
    Vec3 current_up = vec3_rotate_by_quat(world_up, chassis->orientation);

    /* Proportional term: cross(current_up, world_up) points exactly along
     * the horizontal axis (its Y component is always zero by construction,
     * since world_up = (0,1,0) -- see the derivation this file's report
     * cites) that rotates current_up back toward world_up, with magnitude
     * sin(tilt angle). That means this term can only ever produce a torque
     * about a horizontal axis: it changes pitch and roll and is
     * mathematically incapable of containing a yaw (about world Y)
     * component, so yaw is left untouched by construction, not by an
     * after-the-fact mask. No normalize/divide is needed (and none is
     * done), so there is no singularity to guard even when the car is
     * exactly level (cross is zero, torque is zero, as it should be) or
     * exactly inverted (cross degenerates to zero too -- an inherent
     * ambiguity for any up-vector-only correction, not something this PD
     * term is expected to resolve). */
    Vec3 p_term = vec3_scale(vec3_cross(current_up, world_up), strength);

    /* Derivative term: damps angular velocity, but ONLY its pitch/roll part.
     * World-space angular velocity's Y component IS the yaw rate about the
     * world vertical axis (world_up = (0,1,0)), so zeroing just that
     * component before damping leaves yaw rate completely unaffected while
     * still damping tumbling about the horizontal axes. */
    Vec3 omega_pitch_roll = chassis->angular_velocity;
    omega_pitch_roll.y = 0.0f;
    Vec3 d_term = vec3_scale(omega_pitch_roll, damping);

    return vec3_sub(p_term, d_term);
}
