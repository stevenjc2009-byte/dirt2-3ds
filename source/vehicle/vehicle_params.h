/*---------------------------------------------------------------------------------
 * vehicle_params.h -- one place to tune a vehicle: mass, inertia, per-wheel
 * suspension, tyre surface defaults, and drivetrain config.
 *
 * OWNER: vehicle. This header defines the STRUCT shape only -- it declares
 * no constants and ships no default numbers, deliberately. Tuning numbers
 * are gameplay/feel decisions for the Phase 1 test-car work to make (and
 * re-make after playtesting on real hardware, per the project's
 * verify-before-claiming-done discipline), not something to bake into a
 * skeleton header sight-unseen. A future vehicle_params.c (or a
 * data-driven loader, if that's how content ends up shipping) is where the
 * actual test-car numbers belong.
 *
 * Every sub-struct referenced here is owned by its own module (suspension.h,
 * tyre.h, drivetrain.h) -- this header only aggregates them into one
 * loadable unit per vehicle, plus the whole-chassis values (mass, inertia,
 * dimensions) that don't belong to any single subsystem.
 *
 * ONE COHERENT RALLY-CAR NUMBER SET (input/vehicle_params owner, 2026-09-05).
 *   Every field below now carries a documented RECOMMENDED value in its
 *   comment, for one target car: a ~1230 kg Group-A/S2000-era gravel rally
 *   car (DiRT 2's own period), AWD, 2.0L turbo four. These are comments,
 *   not compiled constants -- consistent with this header's declared
 *   struct-only shape above -- because this file's owner does not own
 *   source/main.c or tests/test_physics.c, the two places that currently
 *   construct a VehicleParams with actual numbers (main.c's
 *   make_placeholder_vehicle_params, test_physics.c's
 *   make_test_vehicle_params). Both currently use numbers that don't trace
 *   to any real car; see this module's verification report for the exact
 *   values recommended for main.c to adopt.
 *
 *   Every number below is labelled by provenance:
 *     MEASURED -- taken from a real, cited spec.
 *     DERIVED  -- computed from other numbers here via a stated formula.
 *     CHOSEN   -- a feel/gameplay decision with no single correct answer,
 *                 with the reasoning that drove the choice.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VEHICLE_VEHICLE_PARAMS_H
#define DIRT2_VEHICLE_VEHICLE_PARAMS_H

#include "core/types.h"
#include "vehicle/suspension.h"
#include "vehicle/tyre.h"
#include "vehicle/drivetrain.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VehicleParams {
    /* Whole-chassis rigid body properties -- see core/rigidbody.h for what
     * these feed into.
     *
     * chassis_mass: RECOMMENDED 1230.0f kg.
     *   DERIVED -- FIA Group A / S2000-era gravel rally cars (DiRT 2's own
     *   period) run a homologated minimum weight in the 1200-1300 kg band;
     *   1230 sits in the middle of that band and is close to (but not
     *   copied from) the 1200 kg already used by main.c's placeholder and
     *   tests/test_physics.c's fixture, so a switch to this value would not
     *   be a large jump from what is already tuned against.
     *
     * chassis_inertia_diag: RECOMMENDED (460.0f, 1820.0f, 1760.0f) kg*m^2.
     *   DERIVED -- uniform rectangular-box approximation, the standard
     *   first-pass estimate when no CAD/measured inertia exists:
     *     Ixx (roll, about body x) = m/12 * (W^2 + H^2)
     *     Iyy (yaw,  about body y) = m/12 * (L^2 + W^2)
     *     Izz (pitch,about body z) = m/12 * (L^2 + H^2)
     *   using m=1230 kg, overall length L=3.9 m, track W=1.6 m, height
     *   H=1.4 m (rough gravel-spec rally hatchback dimensions). Roll
     *   inertia comes out much smaller than yaw/pitch, which is the
     *   physically-expected shape for a car (mass is spread far more along
     *   its length than its width) -- the currently-used test fixture
     *   value (600, 900, 1500) inverts that (pitch > yaw > roll) and was
     *   not re-derived here since tests/test_physics.c is owned by another
     *   agent; see this module's verification report.                     */
    f32 chassis_mass;             /* kg                                    */
    Vec3 chassis_inertia_diag;    /* body-space (Ixx, Iyy, Izz), kg*m^2     */

    /* One suspension config per wheel, indexed by WheelIndex
     * (core/types.h). Front/rear or left/right asymmetry (e.g. stiffer
     * rear springs) is expressed by giving different wheels different
     * entries here -- there is no separate "front config" / "rear config"
     * pair, to avoid two representations of the same four numbers.
     *
     * RECOMMENDED per-wheel values (front: WHEEL_FL/WHEEL_FR, rear:
     * WHEEL_RL/WHEEL_RR), all CHOSEN/DERIVED for a per-corner sprung mass
     * of ~307.5 kg (chassis_mass/4, Phase 1 has no front/rear weight
     * distribution split) targeting a rally-appropriate ride frequency of
     * ~2.1-2.2 Hz -- stiffer than a road car's ~1.2 Hz (needed so the body
     * doesn't wallow under the antiroll/tyre loads a rally pace demands)
     * but softer than a tarmac race car's ~3-4 Hz (a rally car needs the
     * wheel travel a stiff race setup would deny it, to keep tyres in
     * contact with rough, undulating ground):
     *   rest_length            0.36f m   (CHOSEN -- rally-spec ride height
     *                                     needs more static droop than a
     *                                     road car's ~0.30f to clear ruts)
     *   max_travel             0.22f m   (CHOSEN -- long-travel rally
     *                                     suspension; roughly double a
     *                                     tarmac car's, for jump landings)
     *   spring_rate            front 58000.0f N/m, rear 50000.0f N/m
     *                          (DERIVED -- k = m_corner*(2*pi*f)^2 with
     *                          m_corner=307.5 kg, f~2.1 Hz front / ~1.95 Hz
     *                          rear; front slightly stiffer than rear is a
     *                          CHOSEN trim -- it transfers a bit more
     *                          lateral load to the outside front under
     *                          cornering, which is what gives a rally
     *                          setup its characteristic loose, rotate-on-
     *                          throttle rear rather than push/understeer)
     *   damper_compression     front 2950.0f N*s/m, rear 2700.0f N*s/m
     *                          (DERIVED -- ~0.35x critical damping,
     *                          c_crit=2*sqrt(k*m_corner), left soft so a
     *                          hard hit doesn't spike chassis force)
     *   damper_rebound         front 5450.0f N*s/m, rear 5100.0f N*s/m
     *                          (DERIVED -- ~0.65x critical damping; firmer
     *                          than compression per this header's own
     *                          asymmetric-damper note, so the car doesn't
     *                          pogo back up after a landing)
     *   bottom_out_spring_rate front 580000.0f N/m, rear 500000.0f N/m
     *                          (CHOSEN -- 10x the main spring_rate, a
     *                          common progressive-bump-stop convention: a
     *                          firm but not infinitely hard limit)
     *   mount_point_body       CHOSEN geometry for a 2.55 m wheelbase /
     *                          1.6 m track: front z=+-0.8, x=+1.275;
     *                          rear z=+-0.8, x=-1.275 (y per ride height,
     *                          content decision left to whoever places the
     *                          chassis mesh's origin)                      */
    SuspensionConfig suspension[VEHICLE_WHEEL_COUNT];

    /* Anti-roll bar rate per axle, indexed by AxleIndex (core/types.h). See
     * suspension_antiroll.
     *
     * RECOMMENDED: AXLE_FRONT 12000.0f N/m, AXLE_REAR 9000.0f N/m.
     *   CHOSEN -- deliberately softer than a tarmac race setup (which
     *   would run this much higher) to preserve wheel articulation and
     *   mechanical grip over rough/bumpy loose surfaces. Front stiffer
     *   than rear reinforces the same loose-rear trim as the front-biased
     *   spring split above.                                               */
    f32 antiroll_rate[VEHICLE_AXLE_COUNT];

    /* Airborne auto-level tuning -- see suspension_airborne_autolevel.
     *
     * RECOMMENDED: autolevel_strength 350.0f, autolevel_damping 55.0f.
     *   CHOSEN -- this is a pure gameplay assist (suspension.h's own
     *   comment is explicit that no real car has this), so there is no
     *   measured or derived source; these values are close to the test
     *   fixture's (400, 60) with a slightly gentler pull, since a rally
     *   car catching air over crests is common enough that a strong
     *   correction would fight the player's own landing control.         */
    f32 autolevel_strength;
    f32 autolevel_damping;

    /* Default tyre surface, used when world/testground.h reports a contact
     * point with no more specific surface data attached. Per-surface
     * overrides (tarmac vs gravel vs mud patches on the test plane) are
     * looked up elsewhere and passed to tyre_solve directly; this is just
     * the fallback so the vehicle always has SOME tyre curve to solve
     * against.
     *
     * RECOMMENDED (gravel, the default surface for a rally car):
     *   peak_slip_ratio  0.20f   (CHOSEN -- higher than tarmac's ~0.10-0.12
     *                             because loose gravel lets a driven wheel
     *                             spin up noticeably before it stops
     *                             gaining forward force)
     *   peak_slip_angle  0.17f rad (~9.7 deg) (CHOSEN -- gravel's peak
     *                             cornering slip angle runs higher than
     *                             tarmac's ~6-7 deg)
     *   peak_mu          1.15f   (DERIVED/CHOSEN -- prepped rally gravel
     *                             tyres with aggressive tread can exceed
     *                             a naive "gravel is low grip" assumption;
     *                             1.15 is a plausible peak, a little above
     *                             tarmac-slick territory for a knobby tyre
     *                             biting into loose material)
     *   sliding_mu       0.65f   (CHOSEN -- a LARGE gap below peak_mu is
     *                             the deliberate lever for "loose, slidey"
     *                             rally handling: once a gravel tyre truly
     *                             breaks away the loose surface gives much
     *                             less grip back than tarmac would, so a
     *                             slide wants to keep going rather than
     *                             self-correct -- this is the single
     *                             number doing the most work toward the
     *                             v0.1 feel goal)                         */
    TyreSurfaceParams default_tyre_surface;

    /* Wheel physical properties needed by vehicle.c's step 8 (integrate
     * wheel spin) that don't belong to drivetrain.h's engine/gearing model
     * or to suspension.h's spring/damper model.
     *
     * RECOMMENDED: wheel_mass_moment_of_inertia 1.3f kg*m^2.
     *   DERIVED -- solid-disk approximation I = 0.5*m*r^2 for an ~18 kg
     *   wheel+tyre+brake assembly at r=0.30 m gives 0.81; nudged up to 1.3
     *   because a tyre's mass sits mostly at the outer radius (closer to a
     *   thin annulus, I = m*r^2) rather than a uniform disk, which pulls
     *   the true value above the solid-disk estimate.                     */
    f32 wheel_mass_moment_of_inertia; /* kg*m^2, about the spin axis, same
                                        * for all four wheels in Phase 1     */

    /* ADDITIVE (input/vehicle_params owner, 2026-09-05): three fields
     * vehicle.c currently fakes with its own file-local #defines because
     * this header shipped no field for them (see vehicle.c's "PHASE 1
     * PLACEHOLDER CONSTANTS" comment block). vehicle.c is owned by a
     * different agent and was NOT edited here to consume these -- see this
     * module's verification report for the exact three-line edit its
     * owner needs to make (replace each #define's use with
     * params->max_steer_angle / params->slip_speed_floor /
     * params->handbrake_torque, and thread `params` into
     * vehicle_phase_steering's parameter list, which does not currently
     * receive it). */

    /* RECOMMENDED: 0.5236f rad (~30 degrees).
     *   CHOSEN -- matches vehicle.c's current VEHICLE_MAX_STEER_ANGLE_RAD
     *   placeholder exactly; a rally car's ~500-560 degree lock-to-lock
     *   steering rack works out to roughly 25-30 degrees of front-wheel
     *   angle, so the existing placeholder was already in a sane range and
     *   is kept rather than changed for its own sake.                     */
    f32 max_steer_angle;      /* radians of front-wheel steer at full
                                * Circle Pad deflection (InputState.steer
                                * == +-1); see vehicle.c's
                                * vehicle_phase_steering.                   */

    /* RECOMMENDED: 1.0f m/s.
     *   CHOSEN -- matches vehicle.c's current VEHICLE_SLIP_SPEED_FLOOR
     *   placeholder; it is the cheapest denominator guard against the
     *   slip-ratio singularity at rest and needs no extra per-wheel state
     *   (see vehicle.c's comment on why a relaxation-length model was not
     *   used instead), so there was no reason found to change it.         */
    f32 slip_speed_floor;     /* m/s, floor on the |v_long| denominator used
                                * when computing slip ratio; see vehicle.c's
                                * vehicle_phase_tyres.                      */

    /* RECOMMENDED: 2600.0f N*m.
     *   CHOSEN -- vehicle.c currently reuses drivetrain.max_brake_torque
     *   (recommended 1900.0f, see DrivetrainConfig below) for the
     *   handbrake instead of a dedicated value; giving the handbrake its
     *   own, higher figure means a handbrake pull reliably locks the rear
     *   wheels for a rally-style pendulum turn even in a situation (high
     *   speed, high grip patch) where the regular brake alone would not
     *   have locked them -- a handbrake that only ever does what the
     *   footbrake already does is a wasted input on this car.             */
    f32 handbrake_torque;     /* N*m, applied to each rear wheel
                                * (WHEEL_RL/WHEEL_RR) in addition to normal
                                * brake torque when InputState.handbrake is
                                * true; see vehicle.c's
                                * vehicle_phase_wheel_spin.                 */

    /* RECOMMENDED drivetrain values (see drivetrain.h's DrivetrainConfig):
     *   layout             DRIVE_AWD (CHOSEN -- period-correct for a
     *                       Group A/S2000-era rally car and for DiRT 2's
     *                       own cars; also the test fixture's choice)
     *   gear_ratio         1.55f (CHOSEN -- combined with final_drive
     *                       below, gives ~134 km/h at the 7500 rpm
     *                       limiter in this single fixed "gear", a
     *                       plausible top speed for one mid/high gear on
     *                       a rally stage)
     *   final_drive_ratio  4.1f (unchanged from the test fixture/main.c
     *                       placeholder -- already a plausible value, no
     *                       reason found to change it)
     *   max_engine_torque  300.0f N*m (MEASURED -- in the ballpark
     *                       commonly cited for 2.0L turbo Group A/WRC-era
     *                       rally engines of DiRT 2's period)
     *   peak_torque_rpm    4500.0f (MEASURED -- typical for the same
     *                       turbo four-cylinder class)
     *   max_rpm            7500.0f (MEASURED -- typical rev-limiter for
     *                       the same class)
     *   idle_rpm           900.0f (MEASURED -- typical idle for the same
     *                       class)
     *   max_brake_torque   1900.0f N*m (CHOSEN -- strong four-wheel
     *                       braking appropriate for a rally stage, without
     *                       being so far beyond the tyre model's peak
     *                       grip that the front locks instantly at any
     *                       brake input)
     *   wheel_radius       0.30f m (CHOSEN -- typical effective rolling
     *                       radius for a 15-16in rally wheel plus gravel
     *                       tyre; unchanged from the test fixture)        */
    DrivetrainConfig drivetrain;
} VehicleParams;

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_VEHICLE_VEHICLE_PARAMS_H */
