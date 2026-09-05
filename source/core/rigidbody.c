/*---------------------------------------------------------------------------------
 * rigidbody.c -- a single free rigid body, semi-implicit Euler. See
 * rigidbody.h for the full contract, including the mandatory per-step
 * quaternion renormalisation and the world-space inverse-inertia rebuild.
 *---------------------------------------------------------------------------------*/
#include "core/rigidbody.h"
#include "core/vecmath.h"

/* Rebuilds the world-space INVERSE inertia tensor from the current
 * orientation and the body-space diagonal inertia: R * I_body^-1 * R^T.
 * Shared by rigidbody_init (identity orientation) and rigidbody_step (the
 * new orientation each step) so there is exactly one place this formula
 * lives. A body-space inertia component of zero or less is treated as
 * "infinite" (no angular response about that axis) rather than dividing by
 * zero -- this project does not currently ship such a body, but the guard
 * costs nothing and avoids a NaN inertia_world_inv if one ever is. */
static Mat3 rigidbody_compute_inertia_world_inv(Quat orientation, Vec3 inertia_body_diag) {
    f32 inv_xx = (inertia_body_diag.x > 0.0f) ? (1.0f / inertia_body_diag.x) : 0.0f;
    f32 inv_yy = (inertia_body_diag.y > 0.0f) ? (1.0f / inertia_body_diag.y) : 0.0f;
    f32 inv_zz = (inertia_body_diag.z > 0.0f) ? (1.0f / inertia_body_diag.z) : 0.0f;
    Mat3 inertia_body_inv = mat3_diagonal(inv_xx, inv_yy, inv_zz);

    Mat3 rot = quat_to_mat3(orientation);
    Mat3 rot_t = mat3_transpose(rot);
    return mat3_mul(mat3_mul(rot, inertia_body_inv), rot_t);
}

void rigidbody_init(RigidBody *rb, f32 mass, Vec3 inertia_body_diag) {
    if (!rb) return;

    rb->mass = mass;
    rb->inv_mass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;

    rb->linear_velocity = vec3_zero();
    rb->orientation = quat_identity();
    rb->angular_velocity = vec3_zero();

    rb->inertia_body_diag = inertia_body_diag;
    rb->inertia_world_inv = rigidbody_compute_inertia_world_inv(rb->orientation, inertia_body_diag);

    rb->force_accum = vec3_zero();
    rb->torque_accum = vec3_zero();
}

void rigidbody_clear_accumulators(RigidBody *rb) {
    if (!rb) return;
    rb->force_accum = vec3_zero();
    rb->torque_accum = vec3_zero();
}

void rigidbody_apply_force_at_point(RigidBody *rb, Vec3 force, Vec3 point) {
    if (!rb) return;
    rb->force_accum = vec3_add(rb->force_accum, force);
    Vec3 r = vec3_sub(point, rb->position);
    rb->torque_accum = vec3_add(rb->torque_accum, vec3_cross(r, force));
}

void rigidbody_step(RigidBody *rb, f32 dt) {
    if (!rb) return;

    /* 1. linear velocity from accumulated force. */
    Vec3 linear_accel = vec3_scale(rb->force_accum, rb->inv_mass);
    rb->linear_velocity = vec3_add(rb->linear_velocity, vec3_scale(linear_accel, dt));

    /* 2. angular velocity from accumulated torque, via the inverse inertia
     * computed from LAST step's orientation (rebuilt at the end of this
     * function for next step). */
    Vec3 angular_accel = mat3_mul_vec3(rb->inertia_world_inv, rb->torque_accum);
    rb->angular_velocity = vec3_add(rb->angular_velocity, vec3_scale(angular_accel, dt));

    /* 3. position uses the NEW linear velocity -- semi-implicit/symplectic. */
    rb->position = vec3_add(rb->position, vec3_scale(rb->linear_velocity, dt));

    /* 4. orientation integrated by the NEW angular velocity, then MANDATORY
     * renormalisation -- see rigidbody.h's ordering contract. */
    rb->orientation = quat_integrate(rb->orientation, rb->angular_velocity, dt);
    rb->orientation = quat_normalize(rb->orientation);

    /* 5. rebuild the world-space inverse inertia from the NEW orientation,
     * ready for next step's torque application -- never cached from init. */
    rb->inertia_world_inv = rigidbody_compute_inertia_world_inv(rb->orientation, rb->inertia_body_diag);
}

Vec3 rigidbody_point_velocity(const RigidBody *rb, Vec3 point) {
    if (!rb) return vec3_zero();
    Vec3 r = vec3_sub(point, rb->position);
    return vec3_add(rb->linear_velocity, vec3_cross(rb->angular_velocity, r));
}
