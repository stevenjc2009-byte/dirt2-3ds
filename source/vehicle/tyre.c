/*---------------------------------------------------------------------------------
 * tyre.c -- combined-slip tyre force model. See tyre.h for the mandatory
 * single-curve contract this implements -- Fx/Fy are NEVER computed from
 * independent lookups, only ever split from one rho-indexed magnitude.
 *
 * SHAPE SOURCE.
 *   The rho -> normalised-force-magnitude curve is sampled from the copied
 *   lookup table (vehicle/tyre_lut.h/.c -- generated offline by
 *   scratchpad/tyre_lut/gen_tyre_lut.py from a Pacejka "Magic Formula" fit,
 *   see that folder's arcade_tuning_rationale.txt for the per-surface
 *   tuning reasoning). Phase 1's world (world/testground.c's
 *   testground_surface_at) reports exactly ONE tyre surface everywhere --
 *   there is no per-patch tarmac/gravel/mud selection yet -- so tyre_solve
 *   does not need to pick a surface row per call. TYRE_SHAPE_TUNING /
 *   TYRE_SHAPE_SURFACE below fix ONE representative row (arcade tuning,
 *   dry tarmac -- this is an arcade rally handling model, and tarmac_dry's
 *   mu is exactly 1.0) purely for its SHAPE: the raw table values for that
 *   row already run 0..1 with no rescale needed on the rising side. That
 *   shape is rescaled at runtime onto THIS vehicle instance's own peak_mu
 *   (value at rho == 1) and sliding_mu (value the curve is pulled toward
 *   as rho grows past the table's range) -- see TyreSurfaceParams in
 *   tyre.h. When a later phase wires up real per-patch surfaces, extend
 *   TyreSurfaceParams with a surface/tuning selector (additive field) and
 *   thread it through here instead of the fixed TYRE_SHAPE_* pair --
 *   out of scope for this single-surface phase, not done here.
 *
 * LOAD SENSITIVITY.
 *   Force scales with Fz^TYRE_LOAD_EXPONENT (~0.8), not linearly with Fz --
 *   real tyres gain progressively less grip per unit of extra load. To keep
 *   that exponent dimensionally sane (Fz is newtons, not a 0..1 fraction)
 *   it is expressed relative to TYRE_NOMINAL_LOAD_N, a reference load at
 *   which the scale factor is exactly 1.0 (so mu(rho)*Fz is exact at that
 *   one load and falls below straight-line scaling on either side) -- a
 *   placeholder in the same spirit as this project's other not-yet-tuned
 *   numbers (see vehicle_params.h's header comment), roughly a rally
 *   hatchback's front corner load with some static+transfer weight.
 *---------------------------------------------------------------------------------*/
#include "vehicle/tyre.h"
#include "vehicle/tyre_lut.h"
#include <math.h>

#define TYRE_SHAPE_TUNING    TYRE_TUNING_ARCADE
#define TYRE_SHAPE_SURFACE   TYRE_SURFACE_TARMAC_DRY
#define TYRE_SHAPE_PEAK      1.0f
/* Last entry (rho == TYRE_LUT_RHO_MAX) of the arcade/tarmac_dry row in
 * tyre_lut.c -- a fixed property of that shipped table row, not a per-call
 * value, so it is a compile-time constant here rather than something read
 * back out of the table at runtime. */
#define TYRE_SHAPE_FLOOR     0.927708918f

#define TYRE_LOAD_EXPONENT   0.8f
#define TYRE_NOMINAL_LOAD_N  4000.0f

#define TYRE_RHO_EPSILON     1e-6f

/* Precomputed reciprocals -- no runtime division for either. */
static const f32 kInvShapeRange    = 1.0f / (TYRE_SHAPE_PEAK - TYRE_SHAPE_FLOOR);
static const f32 kInvNominalLoadN  = 1.0f / TYRE_NOMINAL_LOAD_N;

TyreForceOutput tyre_solve(const TyreSurfaceParams *surface, TyreSlipInput slip) {
    TyreForceOutput out;

    /* rho = sqrt(sr_norm^2 + alpha_norm^2) -- tyre.h's own formula, using
     * THIS instance's own peak_slip_ratio/peak_slip_angle (radians), not
     * the LUT's per-named-surface sr_peak/alpha_peak_deg (those are a
     * separate, currently-unused-here axis -- see file header). */
    f32 sr_norm    = slip.slip_ratio / surface->peak_slip_ratio;
    f32 alpha_norm = slip.slip_angle / surface->peak_slip_angle;
    f32 rho = sqrtf(sr_norm * sr_norm + alpha_norm * alpha_norm);

    /* Degenerate case: no slip at all -- tyre.h's contract requires {0,0}
     * here rather than a 0/0 divide when splitting fx/fy below. */
    if (rho < TYRE_RHO_EPSILON) {
        out.fx = 0.0f;
        out.fy = 0.0f;
        return out;
    }

    /* ONE 1-D lookup on rho (tyre_lut_lookup itself holds the last table
     * value for rho beyond TYRE_LUT_RHO_MAX -- see tyre_lut.c), rescaled
     * onto this instance's own peak_mu/sliding_mu. */
    f32 shape = tyre_lut_lookup(TYRE_SHAPE_TUNING, TYRE_SHAPE_SURFACE, rho);

    f32 mu;
    if (rho <= 1.0f) {
        /* Rising side: this row's shape already runs 0 (rho==0) to 1
         * (rho==1), so peak_mu alone scales it. */
        mu = surface->peak_mu * shape;
    } else {
        /* Falling/plateau side: blend the shape's own [TYRE_SHAPE_FLOOR, 1]
         * range onto [sliding_mu, peak_mu]. */
        f32 decay_frac = (shape - TYRE_SHAPE_FLOOR) * kInvShapeRange;
        mu = surface->sliding_mu + (surface->peak_mu - surface->sliding_mu) * decay_frac;
    }

    /* Load sensitivity: Fz^0.8, not linear. Clamp negative load (should
     * never happen -- vehicle.h step 3 clamps normal_load at or above
     * zero -- but powf on a negative base with a non-integer exponent is
     * NaN, so guard defensively at this boundary anyway.) */
    f32 load_n = (slip.normal_load > 0.0f) ? slip.normal_load : 0.0f;
    f32 load_ratio = load_n * kInvNominalLoadN;
    f32 load_scale = powf(load_ratio, TYRE_LOAD_EXPONENT) * TYRE_NOMINAL_LOAD_N;
    f32 f_mag = mu * load_scale;

    /* Split back into fx/fy along the slip vector's own direction in
     * (sr_norm, alpha_norm) space -- the friction-ellipse split. */
    f32 inv_rho = 1.0f / rho;
    out.fx = f_mag * (sr_norm * inv_rho);
    out.fy = f_mag * (alpha_norm * inv_rho);
    return out;
}
