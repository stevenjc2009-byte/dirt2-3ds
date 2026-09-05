/*---------------------------------------------------------------------------------
 * testground.c -- the Phase 1 three-zone analytic heightfield described in
 * testground.h: flat, rolling hills, washboard, laid out in that order along
 * +Z so one forward test drive crosses all three.
 *
 * HOW THE ZONES ARE MADE CONTINUOUS.
 *   A naive "sine in zone 2, different sine in zone 3" heightfield has a
 *   CLIFF at each zone boundary whenever the zone length is not an exact
 *   whole number of wavelengths -- and with arbitrary config numbers it never
 *   is. A cliff is not a subtle artefact here: a suspension raycast crossing
 *   a vertical step gets a compression discontinuity, the damper sees an
 *   effectively infinite compression velocity for one step, and the car
 *   launches. That would be blamed on the damper, which would be the wrong
 *   place to look.
 *
 *   Two options were considered:
 *     (a) snap each zone's wavelength to length/round(length/wavelength) so a
 *         whole number of cycles fits exactly. Continuous, but it silently
 *         changes the wavelength the config asked for (40 m of washboard at
 *         1.2 m becomes 1.212 m), and the whole point of this world is that a
 *         test can say "the washboard is 1.2 m at 5 cm".
 *     (b) keep the config's exact wavelength and amplitude, and multiply each
 *         zone's bump term by a smooth envelope that reaches zero at that
 *         zone's own two boundaries.
 *
 *   (b) is what is implemented. Every zone's bump term and its slope are both
 *     zero at both of its edges, so height AND surface normal are continuous
 *     across every boundary with no special-casing, while the interior of
 *     each zone has exactly the amplitude and wavelength its config asked
 *     for. The taper costs one wavelength of ramp-in at each end of a zone,
 *     which is also what a real corrugated strip looks like where it starts.
 *
 * SHAPE OF EACH ZONE'S BUMP.
 *   bump(u) = amplitude * 0.5*(1 - cos(2*pi*u/wavelength)) * envelope(u)
 *   The 0.5*(1-cos) form (rather than a raw sine) rides on top of
 *   base_height instead of cutting below it, peaks at exactly `amplitude`,
 *   and is already zero with zero slope at u = 0, so the leading edge of
 *   every zone is continuous before the envelope does anything.
 *
 * NORMALS.
 *   Analytic, not finite-differenced: the derivative of the expression above
 *   is closed-form, and a finite difference would need a step size tuned
 *   against the washboard's short wavelength to avoid smoothing away the
 *   very corrugation this zone exists to test. Height varies only along Z
 *   (the zones are uniform across X), so dh/dx is exactly zero and the normal
 *   is normalize(0, 1, -dh/dz).
 *
 * SURFACE (GRIP) LOOKUP.
 *   Independent of the height zones above: zone 1 (flat) is tarmac, zones 2
 *   and 3 (hills, washboard) share one gravel surface. The boundary is
 *   blended the same way the height envelopes are -- smoothstep over
 *   config.surface_transition_length metres centred on the zone1/zone2 seam
 *   -- rather than switched in one query step. A single-sample cliff in mu
 *   would be a real, not cosmetic, problem: a car straddling the boundary
 *   (its four contact patches on both sides at once, or a single wheel
 *   crossing it between two adjacent physics steps at speed) would see one
 *   wheel's available grip jump discontinuously, which reads as a sudden,
 *   unexplained snap of grip or loss of it -- exactly the kind of surprise
 *   the height-continuity work above exists to avoid on the suspension
 *   side. A hard edge was considered and rejected for the same reason a
 *   hard height edge was rejected in that section, not out of caution for
 *   its own sake.
 *---------------------------------------------------------------------------------*/
#include "world/testground.h"
#include "core/vecmath.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TESTGROUND_TWO_PI 6.28318530717958647692f

/* Smoothstep and its derivative, on x in [0, 1]. Used as the zone-edge
 * envelope (height) AND as the surface grip blend (testground_surface_at):
 * value AND slope are zero at x = 0, so a zone -- or a surface -- fades in
 * without putting a kink in the height field or a step in the tyre's mu. */
static f32 testground_smoothstep(f32 x) {
    return x * x * (3.0f - 2.0f * x);
}
static f32 testground_smoothstep_derivative(f32 x) {
    return 6.0f * x * (1.0f - x);
}

/* Envelope over a zone of length `length` at offset `u`, tapering over
 * `taper` metres at each end. Writes d(envelope)/du through out_denvelope. */
static f32 testground_envelope(f32 u, f32 length, f32 taper, f32 *out_denvelope) {
    if (taper <= 0.0f) {
        *out_denvelope = 0.0f;
        return 1.0f;
    }
    if (u < taper) {
        f32 x = u / taper;
        *out_denvelope = testground_smoothstep_derivative(x) / taper;
        return testground_smoothstep(x);
    }
    if (u > length - taper) {
        f32 x = (length - u) / taper;
        *out_denvelope = -testground_smoothstep_derivative(x) / taper;
        return testground_smoothstep(x);
    }
    *out_denvelope = 0.0f;
    return 1.0f;
}

/* One zone's height contribution above base_height at offset `u` into the
 * zone, plus its slope d(height)/du. Returns 0 (flat) for a degenerate
 * config rather than dividing by a zero wavelength. */
static f32 testground_zone_bump(f32 u, f32 length, f32 amplitude, f32 wavelength,
                                 f32 *out_slope) {
    f32 phase, wave, wave_slope, envelope, envelope_slope, taper;

    *out_slope = 0.0f;
    if (length <= 0.0f || wavelength <= 0.0f || amplitude == 0.0f) return 0.0f;

    /* Taper is one full wavelength where the zone is long enough to spare it,
     * and at most a quarter of the zone otherwise, so a short zone still gets
     * a continuous fade in and out instead of being all taper. */
    taper = wavelength;
    if (taper > length * 0.25f) taper = length * 0.25f;

    phase = TESTGROUND_TWO_PI * u / wavelength;
    wave = 0.5f * (1.0f - cosf(phase));
    wave_slope = ((f32)M_PI / wavelength) * sinf(phase);

    envelope = testground_envelope(u, length, taper, &envelope_slope);

    *out_slope = amplitude * (wave_slope * envelope + wave * envelope_slope);
    return amplitude * wave * envelope;
}

void testground_generate(Testground *tg, const TestgroundConfig *config) {
    /* Phase 1's heightfield is fully analytic (see testground.h on why this
     * is named _generate anyway), so generation is storing the config. */
    if (!tg || !config) return;
    tg->config = *config;
}

bool testground_query(const Testground *tg, f32 world_x, f32 world_z,
                       f32 *out_height, Vec3 *out_normal) {
    const TestgroundConfig *c;
    f32 zone2_start, zone3_start, world_end;
    f32 height, slope = 0.0f;

    if (!tg) return false;
    c = &tg->config;

    zone2_start = c->flat_length;
    zone3_start = c->flat_length + c->hills_length;
    world_end   = zone3_start + c->washboard_length;

    /* Out of bounds is "no ground", never a clamped or extrapolated height --
     * see testground.h. Nothing is written to the out params in that case, so
     * a caller that ignores the return value gets its own uninitialised value
     * rather than a plausible-looking fake height it might trust. */
    if (world_x < -c->world_half_width || world_x > c->world_half_width) return false;
    if (world_z < 0.0f || world_z > world_end) return false;

    if (world_z < zone2_start) {
        /* zone 1 -- flat. */
        height = c->base_height;
        slope = 0.0f;
    } else if (world_z < zone3_start) {
        /* zone 2 -- rolling hills. */
        height = c->base_height + testground_zone_bump(world_z - zone2_start,
                                                        c->hills_length,
                                                        c->hills_amplitude,
                                                        c->hills_wavelength,
                                                        &slope);
    } else {
        /* zone 3 -- washboard. */
        height = c->base_height + testground_zone_bump(world_z - zone3_start,
                                                        c->washboard_length,
                                                        c->washboard_amplitude,
                                                        c->washboard_wavelength,
                                                        &slope);
    }

    if (out_height) *out_height = height;
    if (out_normal) {
        /* Height varies only along Z, so the surface tangent along Z is
         * (0, slope, 1) and along X is (1, 0, 0); their cross product gives
         * the upward normal (0, 1, -slope) before normalisation. */
        *out_normal = vec3_normalize(vec3_make(0.0f, 1.0f, -slope));
    }
    return true;
}

bool testground_height_query(void *userdata, f32 world_x, f32 world_z,
                              f32 *out_height, Vec3 *out_normal) {
    /* Signature-identical adapter so a Testground* drops straight into
     * vehicle_init as ground_userdata with no glue -- see testground.h. */
    return testground_query((const Testground *)userdata, world_x, world_z,
                             out_height, out_normal);
}

/* Linear interpolation between two surface params, componentwise. t is
 * expected pre-clamped to [0, 1] by the caller (testground_surface_at only
 * ever calls this from inside the blend band, where t already is). */
static TyreSurfaceParams testground_lerp_surface(const TyreSurfaceParams *a,
                                                  const TyreSurfaceParams *b,
                                                  f32 t) {
    TyreSurfaceParams out;
    out.peak_slip_ratio = a->peak_slip_ratio + t * (b->peak_slip_ratio - a->peak_slip_ratio);
    out.peak_slip_angle = a->peak_slip_angle + t * (b->peak_slip_angle - a->peak_slip_angle);
    out.peak_mu         = a->peak_mu         + t * (b->peak_mu         - a->peak_mu);
    out.sliding_mu      = a->sliding_mu      + t * (b->sliding_mu      - a->sliding_mu);
    return out;
}

const TyreSurfaceParams *testground_surface_at(const Testground *tg,
                                                f32 world_x, f32 world_z) {
    const TestgroundConfig *c;
    Testground *scratch_owner;
    f32 zone2_start, half_band, band_start, band_end, t;

    (void)world_x; /* surfaces are uniform across X, see testground.h */

    if (!tg) return (const TyreSurfaceParams *)0;
    c = &tg->config;
    zone2_start = c->flat_length;

    /* Casting away const here is well-defined, not a violation of it: every
     * Testground this project constructs (main.c, tests/test_physics.c) is
     * a plain non-const object, and the const on `tg` is only this
     * function's own promise not to modify config -- blended_surface is
     * scratch storage that exists solely so this function can return a
     * POINTER (the established signature) to a value it just computed. See
     * the Testground and testground_surface_at comments in testground.h for
     * the resulting "valid until the next call" lifetime. */
    scratch_owner = (Testground *)tg;

    half_band = c->surface_transition_length * 0.5f;
    if (half_band <= 0.0f) {
        /* No transition configured -- hard edge at the zone boundary. */
        scratch_owner->blended_surface =
            (world_z < zone2_start) ? c->surface_tarmac : c->surface_gravel;
        return &scratch_owner->blended_surface;
    }

    band_start = zone2_start - half_band;
    band_end   = zone2_start + half_band;

    if (world_z <= band_start) {
        scratch_owner->blended_surface = c->surface_tarmac;
    } else if (world_z >= band_end) {
        scratch_owner->blended_surface = c->surface_gravel;
    } else {
        t = testground_smoothstep((world_z - band_start) / (band_end - band_start));
        scratch_owner->blended_surface =
            testground_lerp_surface(&c->surface_tarmac, &c->surface_gravel, t);
    }
    return &scratch_owner->blended_surface;
}
