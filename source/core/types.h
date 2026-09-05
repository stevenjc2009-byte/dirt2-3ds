/*---------------------------------------------------------------------------------
 * types.h -- shared scalar/vector primitive types for the whole engine.
 *
 * OWNER: core. Every other module includes this and must not redefine any of
 * these names.
 *
 * FLOAT, NOT FIXED-POINT.
 *   Old 3DS's ARM11 has VFPv2 (hardware single-precision float, no NEON, no
 *   double-precision fast path). That is enough headroom for a physics
 *   integrator running at 120 Hz. Fixed-point is a possible LATER
 *   optimisation if profiling on real hardware says float is the bottleneck,
 *   but it is not a decision to bake into the type system at Phase 1 -- doing
 *   so now would force every one of the seven parallel implementation agents
 *   to hand-roll fixed-point arithmetic against a moving target. Use `f32`
 *   everywhere a scalar is needed so a future fixed-point experiment is a
 *   typedef-and-operator-overload change in ONE place, not a rewrite.
 *
 * WHEEL INDEXING.
 *   Every per-wheel array in the codebase (suspension state, tyre state,
 *   wheel forces, input split, ...) is indexed by the WheelIndex enum below.
 *   Always use the enum, never a bare 0..3 literal, so the four wheels stay
 *   self-documenting at every call site.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_CORE_TYPES_H
#define DIRT2_CORE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Single scalar type for the whole simulation. See file header: float, not fixed. */
typedef float f32;

/* Fixed array sizes shared by every module that iterates "all wheels" or
 * "all axles". A car is hard-assumed to be a 4-wheeled, 2-axle vehicle for
 * the whole of Phase 1 -- nothing here supports trailers, bikes or more than
 * one driven axle pair. If that ever changes, this is the one place the
 * count changes from. */
#define VEHICLE_WHEEL_COUNT   4
#define VEHICLE_AXLE_COUNT    2

/* Canonical per-wheel index. Every WHEEL_* named array (WheelIndex-indexed)
 * in suspension.h, tyre.h, vehicle.h and drivetrain.h uses these, in this
 * order, so a raw loop `for (i = 0; i < VEHICLE_WHEEL_COUNT; i++)` visits
 * them FL, FR, RL, RR. */
typedef enum WheelIndex {
    WHEEL_FL = 0,   /* front-left  */
    WHEEL_FR = 1,   /* front-right */
    WHEEL_RL = 2,   /* rear-left   */
    WHEEL_RR = 3,   /* rear-right  */
} WheelIndex;

/* Canonical axle index. Front is the steered axle for Phase 1 (no
 * four-wheel-steer). */
typedef enum AxleIndex {
    AXLE_FRONT = 0,
    AXLE_REAR  = 1,
} AxleIndex;

/* Minimal 3D vector. Defined here (not in vecmath.h) because types.h is the
 * one header every module includes, and Vec3/Quat need to be nameable from
 * rigidbody.h, suspension.h, tyre.h etc. without pulling in vecmath.h's
 * function declarations too. vecmath.h owns all the OPERATIONS on this type;
 * this header owns only the layout. */
typedef struct Vec3 {
    f32 x, y, z;
} Vec3;

/* Unit quaternion, (x, y, z, w) with w the scalar part. Renormalised every
 * physics step by rigidbody.c -- see rigidbody.h's ordering note. */
typedef struct Quat {
    f32 x, y, z, w;
} Quat;

/* Row-major 3x3 matrix, used for the world-space inertia tensor and for
 * orientation-derived bases. m[row][col]. */
typedef struct Mat3 {
    f32 m[3][3];
} Mat3;

/* Which of the two contact-relevant vehicle sides a wheel is on. Used by
 * suspension.h's anti-roll-bar pairing (front-left pairs with front-right,
 * etc) so the pairing logic reads by name instead of by parity-of-index. */
typedef enum VehicleSide {
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1,
} VehicleSide;

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_CORE_TYPES_H */
