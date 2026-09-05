/*---------------------------------------------------------------------------------
 * rigidbody.h -- a single free rigid body: position, orientation, velocities,
 * mass and inertia, plus semi-implicit Euler integration.
 *
 * OWNER: core. vehicle.h owns exactly one RigidBody (the chassis) and drives
 * it through the canonical per-frame order documented in vehicle.h -- this
 * module does not know about wheels, suspension or tyres at all, and must
 * stay that way so it is testable and reusable on its own.
 *
 * STATE.
 *   - position, linear_velocity: world space.
 *   - orientation: unit QUATERNION (see core/types.h Quat, core/vecmath.h),
 *     not Euler angles -- Euler gimbal-locks and this vehicle can end up
 *     upside down or airborne at any attitude during a rally jump.
 *   - angular_velocity: world space, rad/s.
 *   - mass: kg (or an internally-consistent Phase-1 unit -- vehicle_params.h
 *     is where real-world-ish numbers get decided; this struct just stores
 *     whatever it's given).
 *   - inertia_body: BODY-SPACE diagonal inertia tensor (Ixx, Iyy, Izz about
 *     the body's own principal axes). Diagonal is a deliberate
 *     simplification -- a real car's inertia tensor is very nearly diagonal
 *     in its own body frame when the body axes are chosen sensibly (x
 *     forward, y up, z right, roughly aligned with the chassis's principal
 *     axes), and a full symmetric 3x3 buys accuracy this project does not
 *     need at the cost of one more matrix inversion per step on hardware
 *     with no fast float divide.
 *   - inertia_world_inv: the INVERSE world-space inertia tensor, i.e.
 *     R * inertia_body^-1 * R^T where R is the current orientation's
 *     rotation matrix. Recomputed once per step (see rigidbody_step) because
 *     R changes every step; storing the inverse (rather than inertia_world
 *     itself) means the hot path that turns torque into angular acceleration
 *     is one matrix-vector multiply, never a runtime matrix inverse.
 *
 * ORDERING CONTRACT: renormalise every step.
 *   rigidbody_step MUST renormalise `orientation` after integrating it
 *   (quat_normalize, see vecmath.h) before returning. Skipping this for
 *   "just one step to save cycles" is exactly how a quaternion drifts off
 *   the unit sphere over a few hundred frames and starts silently scaling
 *   every vector it rotates -- by the time it's visible on screen the bug is
 *   hundreds of frames upstream of where it's noticed.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_CORE_RIGIDBODY_H
#define DIRT2_CORE_RIGIDBODY_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RigidBody {
    Vec3 position;              /* world space, metres                     */
    Vec3 linear_velocity;       /* world space, m/s                        */

    Quat orientation;           /* unit quaternion, world <- body          */
    Vec3 angular_velocity;      /* world space, rad/s                      */

    f32  mass;                  /* kg                                      */
    f32  inv_mass;              /* 1/mass, cached -- 0 for an infinite-mass
                                  * (immovable) body, which this project does
                                  * not currently use but the field exists so
                                  * the divide-by-mass never happens twice */

    Vec3 inertia_body_diag;     /* body-space (Ixx, Iyy, Izz), kg*m^2       */
    Mat3 inertia_world_inv;     /* world-space inverse inertia, recomputed
                                  * every step -- see header note above     */

    /* Force/torque accumulators for the CURRENT step. vehicle.c calls
     * rigidbody_clear_accumulators() at the start of each step (per the
     * canonical order in vehicle.h), then rigidbody_apply_force_at_point()
     * once per wheel as tyre forces are computed, then rigidbody_step()
     * once at the end of the step to integrate. Exposed as fields (not just
     * an opaque accumulate function) so tests can inspect the summed force
     * and torque directly before integration happens. */
    Vec3 force_accum;           /* world space, N                          */
    Vec3 torque_accum;          /* world space, N*m                        */
} RigidBody;

/* Zeroes velocities and accumulators, sets orientation to identity, sets
 * inv_mass from `mass`, and computes the initial inertia_world_inv from
 * `inertia_body_diag` at the identity orientation. Does NOT set position --
 * caller sets that afterwards (or before; order between the two doesn't
 * matter, this function doesn't touch position). */
void rigidbody_init(RigidBody *rb, f32 mass, Vec3 inertia_body_diag);

/* Zeroes force_accum and torque_accum. Call once at the START of a physics
 * step, before any per-wheel force is applied -- see vehicle.h step 6. */
void rigidbody_clear_accumulators(RigidBody *rb);

/* Adds a force applied at world-space point `point` to the body's force and
 * torque accumulators (torque += (point - position) x force). This is the
 * ONE entry point suspension.h and tyre.h forces funnel through -- neither
 * of those modules touches force_accum/torque_accum directly. */
void rigidbody_apply_force_at_point(RigidBody *rb, Vec3 force, Vec3 point);

/* Semi-implicit ("symplectic") Euler integration over `dt` seconds:
 *   1. velocity += (force_accum * inv_mass) * dt
 *   2. angular_velocity += (inertia_world_inv * torque_accum) * dt
 *   3. position += velocity * dt                 (uses the NEW velocity --
 *      that's what makes this semi-implicit rather than explicit Euler, and
 *      is what gives it its much better energy/stability behaviour for a
 *      stiff spring-damper system like a suspension)
 *   4. orientation = quat_integrate(orientation, angular_velocity, dt),
 *      then orientation = quat_normalize(orientation) -- MANDATORY, see
 *      header note above.
 *   5. recompute inertia_world_inv from the NEW orientation, ready for next
 *      step's torque application.
 * Does NOT clear the accumulators -- that is rigidbody_clear_accumulators's
 * job, called at the top of the NEXT step, so a caller that wants to inspect
 * the force/torque that produced this integration still can immediately
 * after calling this function. */
void rigidbody_step(RigidBody *rb, f32 dt);

/* Convenience: world-space velocity of a point fixed in the body's frame at
 * world-space position `point` (v = linear_velocity + angular_velocity x
 * (point - position)). Used by suspension.h (contact-patch velocity for
 * damper force) and tyre.h (contact-patch velocity for slip). */
Vec3 rigidbody_point_velocity(const RigidBody *rb, Vec3 point);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_CORE_RIGIDBODY_H */
