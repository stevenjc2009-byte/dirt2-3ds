/*---------------------------------------------------------------------------------
 * testground.h -- procedurally generated bumpy test plane for Phase 1:
 * flat regions, rolling hills, and a washboard bump strip, so suspension
 * compression/rebound/settle behaviour is both visible and testable.
 *
 * OWNER: world. Implements suspension.h's SuspensionGroundQuery callback
 * signature (see testground_height_query below) so vehicle.c can drive over
 * this world with zero #include dependency on world/testground.h itself --
 * only main.c (which owns picking which world to load) and this module know
 * about each other.
 *
 * LAYOUT (world-space X/Z, flat Y-up):
 *   Three deliberately distinct regions placed along +Z so a test drive
 *   forward crosses all three in order, one per named zone rather than
 *   randomly interspersed -- Phase 1 verification needs to be able to say
 *   "at Z = <N>, the car should be doing <X>", which a fully random
 *   heightfield would not support.
 *     zone 1 (flat):        z in [0, flat_length) -- height == base_height
 *                            everywhere. Baseline: confirms the suspension
 *                            settles to a stable rest compression with no
 *                            external excitation.
 *     zone 2 (rolling hills): z in [flat_length, flat_length+hills_length)
 *                            -- smooth low-frequency sine-based hills.
 *                            Confirms spring/damper compression and rebound
 *                            tracks a slowly-changing surface without
 *                            oscillating or lagging visibly.
 *     zone 3 (washboard):    z in [flat_length+hills_length, world end) --
 *                            a short-wavelength, small-amplitude repeating
 *                            ridge (a real washboard/corrugation is exactly
 *                            this: high-frequency, low-amplitude). Confirms
 *                            the damper can follow rapid small excitations
 *                            without either going numerically unstable at
 *                            120 Hz or filtering them out so much the car
 *                            feels disconnected from the surface.
 *   Beyond the world's X/Z bounds, testground_height_query returns false
 *   (see suspension.h's SuspensionGroundQuery contract) rather than
 *   extrapolating -- driving off the edge of the test plane is legitimately
 *   "no ground," not a wrapped or clamped surface.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_WORLD_TESTGROUND_H
#define DIRT2_WORLD_TESTGROUND_H

#include "core/types.h"
#include "vehicle/suspension.h"
#include "vehicle/tyre.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TestgroundConfig {
    f32 world_half_width;  /* metres, the plane extends
                             * [-world_half_width, +world_half_width] in X  */
    f32 base_height;       /* metres, the flat zone's height and the
                             * baseline every other zone's bumps are added
                             * on top of                                    */

    f32 flat_length;       /* metres of zone 1 along +Z, starting at Z=0    */

    f32 hills_length;      /* metres of zone 2, immediately after zone 1    */
    f32 hills_amplitude;   /* metres, peak height added by the rolling
                             * hills above base_height                      */
    f32 hills_wavelength;  /* metres per full sine cycle of the hills       */

    f32 washboard_length;      /* metres of zone 3, immediately after zone 2 */
    f32 washboard_amplitude;   /* metres, small -- this is a corrugation,
                                 * not a hill                                */
    f32 washboard_wavelength;  /* metres per ridge -- short, unlike
                                 * hills_wavelength                          */

    /* SURFACE (tyre grip), independent of the height zones above.
     * Zone 1 (flat) is a hard-packed strip -- tarmac -- so a test drive can
     * build speed before reaching the loose stuff; zones 2 and 3 (hills,
     * washboard) share one gravel surface, matching the real-world pairing
     * of "rolling terrain + corrugation" with a loose surface rather than
     * pavement. See testground_surface_at for the lookup and its blend
     * band; see testground.c for why these are the only two surfaces and
     * where the boundary sits. */
    TyreSurfaceParams surface_tarmac;   /* zone 1 (flat)                    */
    TyreSurfaceParams surface_gravel;   /* zones 2 + 3 (hills, washboard)   */

    /* Metres of smooth grip blend centred on the zone1/zone2 boundary
     * (world Z == flat_length). 0 (or negative) means a hard edge instead
     * -- see testground_surface_at's header comment for what that would
     * feel like and why it is not the default. */
    f32 surface_transition_length;
} TestgroundConfig;

/* Opaque-ish handle for the generated test ground. Kept as a plain struct
 * (not hidden behind a forward-declared pointer) because Phase 1's
 * heightfield is fully procedural/analytic -- there is no baked mesh data
 * to hide, just the config plus whatever small amount of derived state
 * generation produces.
 *
 * blended_surface is scratch storage owned by this struct so
 * testground_surface_at can hand back a POINTER to a possibly-blended
 * TyreSurfaceParams value without a static/global (which would not be
 * reentrant across multiple Testground instances). It holds only the
 * result of the MOST RECENT testground_surface_at call on this instance --
 * see that function's header comment. */
typedef struct Testground {
    TestgroundConfig config;
    TyreSurfaceParams blended_surface;
} Testground;

/* "Generates" the test ground -- for a fully analytic heightfield this
 * mostly just validates/stores `config`, but is named _generate rather than
 * _init to leave room for a future version that bakes a mesh (for rendering
 * -- see world.h's eventual real-terrain module) from the same analytic
 * definition without changing the call site. */
void testground_generate(Testground *tg, const TestgroundConfig *config);

/* Analytic height + normal at world-space (x, z). Returns false if (x, z)
 * is outside [-world_half_width, +world_half_width] in X or before Z=0 or
 * past the end of zone 3 in Z -- see header note on why out-of-bounds is
 * "no ground" rather than clamped. */
bool testground_query(const Testground *tg, f32 world_x, f32 world_z,
                       f32 *out_height, Vec3 *out_normal);

/* Adapter matching suspension.h's SuspensionGroundQuery signature exactly,
 * so a Testground* can be passed as vehicle_init's ground_userdata and this
 * function as its ground_query with no glue code at the call site:
 *
 *   vehicle_init(&car, &params, testground_height_query, &my_testground);
 */
bool testground_height_query(void *userdata, f32 world_x, f32 world_z,
                              f32 *out_height, Vec3 *out_normal);

/* Returns the tyre surface parameters at world-space (x, z): tarmac in
 * zone 1 (flat), gravel in zones 2/3 (hills, washboard), smoothly blended
 * across config.surface_transition_length metres centred on the zone1/zone2
 * boundary so a car crossing it sees grip change gradually rather than in
 * one query step -- see testground.c for why a one-sample cliff in mu is
 * worse here than it sounds (a suspension raycast already treats a height
 * cliff as a shock; a GRIP cliff mid-corner is the same problem one level
 * up, applied to the tyre solve instead of the spring). world_x is currently
 * unused (surfaces, like the height zones, are uniform across X) but stays
 * in the signature so no call site changes if that ever stops being true.
 *
 * Always returns a valid, non-NULL pointer for this project's Testground --
 * there is no "no surface data" case once TestgroundConfig carries the
 * fields above. The returned pointer is scratch storage owned by `tg`
 * (see Testground's own comment): it is valid to read only until the next
 * call to testground_surface_at on the SAME Testground instance, so a
 * caller must use it (e.g. pass straight to tyre_solve) before making
 * another such call, not hold onto it. */
const TyreSurfaceParams *testground_surface_at(const Testground *tg,
                                                f32 world_x, f32 world_z);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_WORLD_TESTGROUND_H */
