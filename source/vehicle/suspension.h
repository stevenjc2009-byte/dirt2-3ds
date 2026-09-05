/*---------------------------------------------------------------------------------
 * suspension.h -- per-wheel raycast spring/damper suspension, anti-roll bar,
 * bottom-out handling, and an airborne auto-level PD torque.
 *
 * OWNER: vehicle. Called once per wheel per physics step from vehicle.c, in
 * the position documented as steps 1-3 of vehicle.h's canonical order:
 *   1. raycast to find ground contact and compression
 *   2. spring + damper force from compression and compression velocity
 *   3. normal load = suspension force, clamped at or above zero
 *
 * This module does NOT touch the rigid body directly -- it returns forces
 * and lets vehicle.c apply them via rigidbody_apply_force_at_point (see
 * core/rigidbody.h), so suspension.c stays testable in isolation with a
 * plain RigidBody* and a heightfield query function, no world/testground.h
 * dependency baked in.
 *
 * RAYCAST SOURCE.
 *   The raycast itself is a simple analytic ray-vs-heightfield query against
 *   whatever world module is active (world/testground.h for Phase 1). This
 *   header takes the query as a function pointer (SuspensionGroundQuery) so
 *   suspension.c has zero #include dependency on world/testground.h -- the
 *   vehicle module must not know or care what kind of world it's driving
 *   over.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VEHICLE_SUSPENSION_H
#define DIRT2_VEHICLE_SUSPENSION_H

#include "core/types.h"
#include "core/rigidbody.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ground-query callback: given a world-space (x, z) column, returns the
 * ground height at that column and a unit surface normal. Returns false if
 * the query is out of the world's bounds (suspension.c treats that as "no
 * ground found within travel", i.e. the wheel is airborne over the void).
 * `userdata` is passed through unchanged -- typically a world/testground.h
 * heightfield pointer, but suspension.c never assumes that. */
typedef bool (*SuspensionGroundQuery)(void *userdata, f32 world_x, f32 world_z,
                                       f32 *out_height, Vec3 *out_normal);

/* Static per-wheel suspension configuration -- geometry and spring/damper
 * tuning. One of these per wheel, set from vehicle_params.h at vehicle
 * creation and not mutated during play. */
typedef struct SuspensionConfig {
    f32 rest_length;        /* metres, fully extended (unloaded) length     */
    f32 max_travel;         /* metres, total compressible travel from fully
                              * extended to fully bottomed-out               */
    f32 spring_rate;        /* N/m                                          */
    f32 damper_compression; /* N*s/m, damping coefficient while compressing */
    f32 damper_rebound;     /* N*s/m, damping coefficient while extending --
                              * kept separate from damper_compression because
                              * a real damper (and a believable-feeling one)
                              * is asymmetric: firmer on rebound than on
                              * compression so the car doesn't pogo back up
                              * after absorbing a bump                       */
    f32 bottom_out_spring_rate; /* N/m, extra stiffness applied only once
                                  * compression exceeds max_travel -- see
                                  * suspension_solve's bottom-out note        */
    Vec3 mount_point_body;  /* body-space attachment point the raycast
                              * originates from (roughly the wheel's
                              * unloaded position relative to the chassis's
                              * centre of mass)                              */
} SuspensionConfig;

/* Per-wheel runtime state, persisted frame to frame (compression VELOCITY
 * needs the previous frame's compression to compute, and the airborne
 * auto-level PD term needs continuity too). One of these per wheel, owned by
 * Vehicle (see vehicle.h). */
typedef struct SuspensionState {
    f32  compression;          /* metres, 0 = fully extended, max_travel =
                                 * fully bottomed out                        */
    f32  compression_prev;     /* previous step's compression, for computing
                                 * compression_velocity by finite difference */
    f32  compression_velocity; /* m/s, positive = compressing                */
    bool grounded;             /* true if the raycast found ground within
                                 * rest_length + max_travel this step        */
    Vec3 contact_point;        /* world space, valid only if grounded        */
    Vec3 contact_normal;       /* world space unit vector, valid only if
                                 * grounded                                  */
    f32  normal_load;          /* N, this wheel's contribution -- see
                                 * suspension_solve's return contract. This
                                 * is what tyre.h reads as its Fz input, and
                                 * it MUST be this step's freshly-computed
                                 * value, not last step's -- see vehicle.h's
                                 * "stale normal load" ordering-bug note.    */
} SuspensionState;

/* Runs the raycast, spring/damper and bottom-out solve for ONE wheel and
 * writes the result into `state` (compression, compression_velocity,
 * grounded, contact point/normal, normal_load all updated). Returns the
 * suspension force to apply to the chassis, in world space, already
 * clamped so its magnitude along contact_normal is >= 0 -- a suspension can
 * only push, never pull, the chassis toward the ground (see vehicle.h step
 * 3, "normal load ... clamped at or above zero"). Returns a zero vector
 * (and state->grounded = false) if the wheel is airborne.
 *
 * BOTTOM-OUT: once `state->compression` would exceed `config->max_travel`,
 * the extra penetration past max_travel is resisted by an additional spring
 * term using `config->bottom_out_spring_rate` (much stiffer than
 * spring_rate) rather than clamping compression outright -- a hard clamp
 * with no extra force is indistinguishable from driving through the ground,
 * and produces a visible "thunk with no cause" the first time a hard
 * landing bottoms the suspension out.
 *
 * `wheel_pos_world` is where the wheel currently sits (chassis mount point
 * transformed to world space by vehicle.c, since only vehicle.c knows the
 * chassis's current pose) -- suspension.c does not read RigidBody's
 * orientation/position fields directly, keeping it decoupled from exactly
 * how the caller computes wheel placement. */
Vec3 suspension_solve(const SuspensionConfig *config, SuspensionState *state,
                       const RigidBody *chassis, Vec3 wheel_pos_world,
                       SuspensionGroundQuery ground_query, void *ground_userdata,
                       f32 dt);

/* Anti-roll bar: given the two SuspensionState compressions on one axle
 * (left and right wheel of the same axle -- see core/types.h's AxleIndex),
 * returns a force pair (applied at each wheel's contact point, same
 * magnitude, opposite... not quite opposite -- see below) proportional to
 * the compression DIFFERENCE across the axle. A conventional anti-roll bar
 * pushes the less-compressed wheel down and the more-compressed wheel's
 * force is reduced by the same amount, resisting body roll without changing
 * the axle's total vertical load. `bar_rate` is N per metre of compression
 * difference.
 *
 * Returns the two forces via out params rather than a struct so the call
 * site (vehicle.c, once per axle) stays a single readable line naming both
 * wheels explicitly. */
void suspension_antiroll(const SuspensionState *left, const SuspensionState *right,
                          f32 bar_rate, Vec3 left_normal, Vec3 right_normal,
                          Vec3 *out_left_force, Vec3 *out_right_force);

/* Airborne auto-level: when NONE of the four wheels are grounded (a full
 * jump), applies a PD (proportional-derivative) torque about the chassis's
 * pitch and roll axes that gently rotates the car back toward level (yaw is
 * left alone -- a rally car should keep whatever yaw/rotation it launched
 * with, it should not un-spin itself in the air). This is a gameplay
 * assist, not a physical force -- real cars have no such thing -- included
 * because a from-scratch rigid body with no aerodynamic damping tumbles
 * unrecoverably after almost any jump, which reads as broken rather than as
 * realistic.
 *
 * `strength` and `damping` are tuning gains (vehicle_params.h), both >= 0.
 * Returns the torque to apply via rigidbody_apply_force_at_point-style
 * torque accumulation -- vehicle.c adds it straight to
 * chassis->torque_accum, since a pure torque has no meaningful application
 * point. Callers must only invoke this when ALL FOUR wheels report
 * !grounded this step; it is not vehicle.c's job to re-check that inside
 * this function, so the caller decides and this function trusts it. */
Vec3 suspension_airborne_autolevel(const RigidBody *chassis, f32 strength, f32 damping);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_VEHICLE_SUSPENSION_H */
