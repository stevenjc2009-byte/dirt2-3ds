/*---------------------------------------------------------------------------------
 * vecmath.h -- vector, quaternion and 3x3 matrix operations.
 *
 * OWNER: core. Layout for Vec3/Quat/Mat3 lives in types.h; this header owns
 * every OPERATION on them. Every other module (rigidbody, suspension, tyre,
 * vehicle, camera) should reach for a function here rather than hand-rolling
 * dot products or cross products inline -- one implementation to profile and
 * to get right on VFPv2.
 *
 * NO NEON. Old 3DS's ARM11 has VFPv2 only, so these are plain scalar
 * float ops; do not add SIMD/NEON intrinsics or assume 4-wide operations.
 *
 * Conventions:
 *   - All angles in radians.
 *   - Quaternions are (x, y, z, w), w scalar-first is NOT used anywhere in
 *     this codebase -- always w last, matching types.h's Quat layout.
 *   - Functions that return a new value take their inputs by const-value or
 *     const-pointer and never mutate an argument in place, except the
 *     `*_normalize` family which is explicit about mutating its argument.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_CORE_VECMATH_H
#define DIRT2_CORE_VECMATH_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Vec3 ---------------------------------------------------------- */

Vec3 vec3_make(f32 x, f32 y, f32 z);
Vec3 vec3_zero(void);

Vec3 vec3_add(Vec3 a, Vec3 b);
Vec3 vec3_sub(Vec3 a, Vec3 b);
Vec3 vec3_scale(Vec3 a, f32 s);
Vec3 vec3_negate(Vec3 a);

f32  vec3_dot(Vec3 a, Vec3 b);
Vec3 vec3_cross(Vec3 a, Vec3 b);

f32  vec3_length(Vec3 a);
f32  vec3_length_sq(Vec3 a);

/* Returns a zero vector if `a` is (near-)zero length rather than dividing by
 * zero -- callers must not assume the result is unit length without checking
 * the input first if a zero vector is a possible input. */
Vec3 vec3_normalize(Vec3 a);

/* Linear interpolation, t is NOT clamped -- callers clamp if they need to. */
Vec3 vec3_lerp(Vec3 a, Vec3 b, f32 t);

/* Rotate a vector by a unit quaternion. */
Vec3 vec3_rotate_by_quat(Vec3 v, Quat q);

/* ---- Quat ------------------------------------------------------------ */

Quat quat_identity(void);
Quat quat_from_axis_angle(Vec3 axis, f32 angle_rad);

Quat quat_mul(Quat a, Quat b);

/* Renormalises q. MUST be called once per physics step on the rigid body's
 * orientation (see rigidbody.h) -- repeated multiplication by an angular-
 * velocity delta quaternion drifts off the unit sphere over hundreds of
 * steps, and an unnormalised quaternion silently scales every vector it
 * rotates. */
Quat quat_normalize(Quat q);

/* Integrates orientation forward by angular velocity `omega` (world-space,
 * rad/s) over `dt` seconds using the standard first-order quaternion
 * derivative (q_dot = 0.5 * omega_quat * q), NOT renormalised internally --
 * the caller (rigidbody.c) is expected to call quat_normalize() itself so
 * the "renormalise once per step" contract stays visible at the call site
 * rather than hidden inside this helper. */
Quat quat_integrate(Quat q, Vec3 omega, f32 dt);

Mat3 quat_to_mat3(Quat q);

/* Spherical linear interpolation between two orientations, t in [0, 1].
 * Used by the renderer's interpolated draw pose (see timestep.h) -- NOT by
 * the physics step itself, which uses quat_integrate. */
Quat quat_slerp(Quat a, Quat b, f32 t);

/* ---- Mat3 -------------------------------------------------------------- */

Mat3 mat3_identity(void);
Mat3 mat3_mul(Mat3 a, Mat3 b);
Mat3 mat3_transpose(Mat3 a);
Vec3 mat3_mul_vec3(Mat3 m, Vec3 v);

/* Builds a diagonal matrix, used to turn rigidbody.h's body-space diagonal
 * inertia (Ixx, Iyy, Izz) into a Mat3 before rotating it into world space. */
Mat3 mat3_diagonal(f32 xx, f32 yy, f32 zz);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_CORE_VECMATH_H */
