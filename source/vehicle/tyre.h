/*---------------------------------------------------------------------------------
 * tyre.h -- combined-slip tyre force model.
 *
 * OWNER: vehicle. Called once per wheel per physics step from vehicle.c,
 * step 4-5 of vehicle.h's canonical order:
 *   4. tyre slip ratio and slip angle from wheel spin vs contact-patch velocity
 *   5. tyre longitudinal and lateral force from slip, normal load, and surface
 *
 * COMBINED SLIP, ONE CURVE -- READ BEFORE CHANGING THIS INTERFACE.
 *   A tyre's longitudinal grip (accelerating/braking) and lateral grip
 *   (cornering) are NOT independent -- they share one friction budget (the
 *   "friction circle" / "friction ellipse"). A tyre already using most of
 *   its grip to brake has little left over to resist sideways slip, and vice
 *   versa. Modelling Fx and Fy as two independent 1-D curves (e.g. a
 *   longitudinal Pacejka curve and a separate lateral Pacejka curve, each
 *   evaluated on its own slip and summed as if orthogonal) DOUBLE-COUNTS
 *   grip at the boundary: a tyre at 100% of its longitudinal curve AND 100%
 *   of its lateral curve would be asked for 141% of its actual available
 *   force (sqrt(1^2+1^2)), which is physically impossible and reads as the
 *   car mysteriously refusing to slide when it should, or gripping
 *   impossibly hard while braking and turning at once.
 *
 *   The fix used here is the standard combined-slip normalisation:
 *     1. normalise each slip component by its own "peak" slip (the slip
 *        value at which that component's force would peak in isolation):
 *          sr_norm    = slip_ratio / SR_peak
 *          alpha_norm = slip_angle / alpha_peak
 *     2. combine into one scalar magnitude:
 *          rho = sqrt(sr_norm^2 + alpha_norm^2)
 *     3. look up ONE force curve on rho (tyre_force_curve, a single 1-D
 *        table -- see below) to get a combined force MAGNITUDE
 *     4. split that single magnitude back into Fx/Fy in proportion to each
 *        normalised slip's share of rho:
 *          Fx = combined_force_mag * (sr_norm    / rho) * (normal_load ...)
 *          Fy = combined_force_mag * (alpha_norm / rho) * (normal_load ...)
 *
 *   This is why tyre_solve's signature takes BOTH slip components and
 *   returns BOTH forces from one call -- there is deliberately no
 *   tyre_solve_longitudinal / tyre_solve_lateral pair. Do not add one; it
 *   would let a caller reintroduce the double-counting bug this interface
 *   exists to prevent.
 *
 * WHY ONE 1-D LOOKUP TABLE.
 *   Collapsing combined slip to a single normalised magnitude rho means the
 *   whole nonlinear tyre curve (rises to a peak around rho ~= 1, then falls
 *   off into the sliding region) is exactly ONE 1-D function of one input,
 *   which is what a device with no fast hardware float divide wants: a
 *   small precomputed table (tyre_force_curve.c, an implementation detail
 *   not exposed here) plus a cheap lerp between two entries, instead of
 *   evaluating a multi-term Pacejka "Magic Formula" (several sin/atan/pow
 *   calls) twice per wheel per step, eight times per physics step, at
 *   120 Hz, on a 268 MHz ARM11.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VEHICLE_TYRE_H
#define DIRT2_VEHICLE_TYRE_H

#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Per-surface tyre curve parameters. Different surfaces (tarmac, gravel,
 * mud, ...) get different instances of this struct -- see
 * world/testground.h for how a surface type reaches vehicle.c. */
typedef struct TyreSurfaceParams {
    f32 peak_slip_ratio;   /* SR_peak: slip ratio at peak longitudinal force,
                             * dimensionless (0.1-0.2 is a typical real-tyre
                             * range)                                        */
    f32 peak_slip_angle;   /* alpha_peak: slip angle (RADIANS) at peak
                             * lateral force                                 */
    f32 peak_mu;           /* peak combined friction coefficient at rho == 1
                             * (force / normal_load at the curve's maximum)  */
    f32 sliding_mu;        /* friction coefficient once fully sliding
                             * (rho >> 1, the flat/falling tail of the curve)
                             * -- normally < peak_mu, that gap is what makes
                             * a slide, once started, want to keep going     */
} TyreSurfaceParams;

/* Inputs to a single wheel's tyre solve for one physics step. Slip ratio
 * and slip angle are computed by vehicle.c from wheel spin and
 * contact-patch velocity (step 4 of vehicle.h's canonical order) and handed
 * in here already computed -- tyre.h does not know about wheel spin or
 * rigid-body velocities at all, keeping it testable with plain numbers. */
typedef struct TyreSlipInput {
    f32 slip_ratio;    /* dimensionless, positive = wheel spinning faster
                         * than the ground (driving/accelerating slip),
                         * negative = wheel spinning slower (braking slip)  */
    f32 slip_angle;    /* radians, angle between the wheel's heading and its
                         * actual direction of travel at the contact patch  */
    f32 normal_load;   /* N, from suspension.h's SuspensionState.normal_load
                         * for THIS SAME STEP -- see vehicle.h's "stale
                         * normal load" ordering-bug note. Never a value
                         * left over from a previous step.                  */
} TyreSlipInput;

/* Output of a single wheel's tyre solve: a combined-slip force already
 * split into the wheel's own longitudinal (x, along the direction the wheel
 * is pointed) and lateral (y, perpendicular to that, in the ground plane)
 * axes. vehicle.c rotates this pair into world space using the wheel's
 * current steer/heading before applying it to the chassis (step 6). */
typedef struct TyreForceOutput {
    f32 fx;   /* N, longitudinal (+ forward)  */
    f32 fy;   /* N, lateral (+ defined consistently with slip_angle's sign) */
} TyreForceOutput;

/* Runs the combined-slip solve described in the header comment above:
 * normalise slip_ratio and slip_angle by the surface's peak_slip_ratio /
 * peak_slip_angle, combine into rho = sqrt(sr_norm^2 + alpha_norm^2), look
 * up ONE force-magnitude curve on rho, split the result back into fx/fy in
 * proportion to each normalised slip's share of rho.
 *
 * Degenerate case: if both slip_ratio and slip_angle are exactly zero (rho
 * == 0), returns {0, 0} rather than dividing 0/0 when splitting the
 * magnitude back into components -- a stationary, unslipping tyre applies
 * no force, which is physically correct and also sidesteps the singularity
 * at the origin of the fx/fy split. */
TyreForceOutput tyre_solve(const TyreSurfaceParams *surface, TyreSlipInput slip);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_VEHICLE_TYRE_H */
