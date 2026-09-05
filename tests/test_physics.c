/*---------------------------------------------------------------------------------
 * test_physics.c -- host-side physics test suite for the dirt2 core.
 *
 * THIS FILE IS EXPECTED TO FAIL RIGHT NOW, IN PART.
 *   core/vecmath.c, core/rigidbody.c and core/timestep.c are genuinely
 *   implemented as of this writing; vehicle/suspension.c, vehicle/tyre.c,
 *   vehicle/drivetrain.c, vehicle/vehicle.c and world/testground.c are still
 *   compiling STUBS (see each .c file's own header comment) being filled in
 *   by parallel agents. Every assertion below is written against the
 *   HEADER CONTRACT (the interface, not any particular .c's current body),
 *   so tests that depend on a still-stubbed module are SUPPOSED to fail
 *   until that module lands -- a test that already passes against a stub is
 *   testing nothing. Do not weaken a tolerance or skip a check to make a
 *   stub pass.
 *
 * FAULT INJECTION (proving a check can go red).
 *   Every numbered assertion below that exercises ALREADY-REAL code
 *   (rigidbody/vecmath/timestep) or that produces a definite expected VALUE
 *   (not just "stub returns zero") can be independently corrupted by
 *   building with -DDIRT2_INJECT_FAULT=<N>, which deliberately breaks the
 *   Nth check's expected value/threshold so its own CHECK is proven capable
 *   of failing on real data, not just on an absent implementation. Default
 *   (no -D) is 0 == no injection, the normal test build.
 *
 * RATE INDEPENDENCE.
 *   All step counts below are expressed as "N simulated seconds" via
 *   PHYSICS_HZ (core/timestep.h's compile-time constant), never as a raw
 *   step count -- so the SAME source, compiled three times by
 *   `make -f Makefile.host test-rates` at 60/100/120 Hz, exercises the same
 *   real-time window at each rate. See
 *   test_suspension_stable_across_all_physics_rates for a check
 *   specifically designed to FAIL at 60 Hz and PASS at 100/120 Hz against a
 *   naive (rate-dependent-unstable) damper implementation -- the "stiff
 *   springs at 60 Hz" classic failure this suite must be able to catch.
 *---------------------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

#include "core/types.h"
#include "core/vecmath.h"
#include "core/rigidbody.h"
#include "core/timestep.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicle_params.h"
#include "vehicle/suspension.h"
#include "vehicle/tyre.h"
#include "vehicle/tyre_lut.h"   /* tyre_lut_lookup -- probed directly by the
                                  * non-finite-rho regression test           */
#include "vehicle/drivetrain.h"
#include "world/testground.h"
#include "input/input.h"

#ifndef DIRT2_INJECT_FAULT
#define DIRT2_INJECT_FAULT 0
#endif

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) \
    do { \
        g_checks++; \
        if (!(cond)) { \
            g_failures++; \
            fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
        } \
    } while (0)

/* Physical constants this test file needs that the project itself does not
 * (yet) define anywhere -- grep confirms no GRAVITY/9.81 constant exists
 * under source/, and vehicle.h's canonical per-frame order does not mention
 * where gravity gets applied at all. See this suite's final report for why
 * that is flagged as a header-completeness question rather than something
 * fixed here. */
#define TEST_GRAVITY 9.81f
#define PI_F 3.14159265358979323846f

/* ---- generic numeric helpers ------------------------------------------- */

static int nearly_equal_f32(f32 a, f32 b, f32 abs_tol, f32 rel_tol) {
    f32 diff = fabsf(a - b);
    if (diff <= abs_tol) return 1;
    f32 scale = fabsf(a) > fabsf(b) ? fabsf(a) : fabsf(b);
    return diff <= rel_tol * scale;
}

static int vec3_is_finite(Vec3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static int quat_is_finite(Quat q) {
    return isfinite(q.x) && isfinite(q.y) && isfinite(q.z) && isfinite(q.w);
}

static f32 quat_length(Quat q) {
    return sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

/* ---- shared test fixtures ------------------------------------------------
 * One place each subsystem's "reasonable tuning" numbers live, reused
 * across every test in this file so the physical scenario (a ~1200 kg test
 * car, 300 kg/wheel) stays consistent no matter which module a given test
 * is exercising. Values match tests/test_physics.c's own former smoke test
 * / source/main.c's placeholder numbers, not anything vehicle_params.h
 * itself ships (it deliberately ships none -- see that header). */

static SuspensionConfig make_test_suspension_config(void) {
    SuspensionConfig cfg;
    cfg.rest_length = 0.30f;
    cfg.max_travel = 0.15f;
    cfg.spring_rate = 35000.0f;
    cfg.damper_compression = 3000.0f;
    cfg.damper_rebound = 4500.0f;
    cfg.bottom_out_spring_rate = 200000.0f;
    cfg.mount_point_body = vec3_zero();
    return cfg;
}

static TyreSurfaceParams make_test_tyre_surface(void) {
    TyreSurfaceParams s;
    s.peak_slip_ratio = 0.15f;
    s.peak_slip_angle = 0.12f;
    s.peak_mu = 1.1f;
    s.sliding_mu = 0.8f;
    return s;
}

static DrivetrainConfig make_test_drivetrain_config(void) {
    DrivetrainConfig cfg;
    cfg.layout = DRIVE_AWD;
    cfg.gear_ratio = 3.5f;
    cfg.final_drive_ratio = 4.1f;
    cfg.max_engine_torque = 300.0f;
    cfg.peak_torque_rpm = 4500.0f;
    cfg.max_rpm = 7500.0f;
    cfg.idle_rpm = 900.0f;
    cfg.max_brake_torque = 1800.0f;
    cfg.wheel_radius = 0.30f;
    return cfg;
}

static VehicleParams make_test_vehicle_params(void) {
    VehicleParams p;
    int i;
    p.chassis_mass = 1200.0f;
    p.chassis_inertia_diag = vec3_make(600.0f, 900.0f, 1500.0f);
    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        p.suspension[i] = make_test_suspension_config();
    }
    p.antiroll_rate[AXLE_FRONT] = 15000.0f;
    p.antiroll_rate[AXLE_REAR] = 10000.0f;
    p.autolevel_strength = 400.0f;
    p.autolevel_damping = 60.0f;
    p.default_tyre_surface = make_test_tyre_surface();
    p.wheel_mass_moment_of_inertia = 1.2f;
    p.drivetrain = make_test_drivetrain_config();
    /* Leaving these three unset is a CRASH, not a wrong number: a garbage
     * slip_speed_floor makes the slip denominator NaN, rho NaN, and the tyre
     * table index out of bounds. AddressSanitizer caught exactly that here,
     * as a SEGV inside tyre_lut_lookup, the moment vehicle.c started reading
     * these from VehicleParams instead of from its own local #defines. */
    p.max_steer_angle  = 0.5236f;
    p.slip_speed_floor = 1.0f;
    p.handbrake_torque = 2600.0f;
    return p;
}

/* Fills the surface fields of a TestgroundConfig. Factored out so the two
 * constructors below cannot diverge, and so that a THIRD constructor added
 * later has an obvious thing to call. Harmless to omit today only because
 * nothing in the sim calls testground_surface_at through these grounds --
 * the moment one does, an omission here is uninitialised stack reaching the
 * tyre model, which is the same class of bug that put a SEGV in
 * tyre_lut_lookup during v0.1 integration. */
static void fill_test_surfaces(TestgroundConfig *cfg) {
    cfg->surface_tarmac.peak_mu         = 1.00f;
    cfg->surface_tarmac.sliding_mu      = 0.75f;
    cfg->surface_tarmac.peak_slip_ratio = 0.20f;
    cfg->surface_tarmac.peak_slip_angle = 0.1396f;  /* 8 deg  */
    cfg->surface_gravel.peak_mu         = 0.60f;
    cfg->surface_gravel.sliding_mu      = 0.51f;
    cfg->surface_gravel.peak_slip_ratio = 0.25f;
    cfg->surface_gravel.peak_slip_angle = 0.2618f;  /* 15 deg */
    cfg->surface_transition_length      = 6.0f;
}

static Testground make_flat_testground(f32 flat_length) {
    Testground tg;
    TestgroundConfig cfg;
    fill_test_surfaces(&cfg);
    cfg.world_half_width = 25.0f;
    cfg.base_height = 0.0f;
    cfg.flat_length = flat_length;
    cfg.hills_length = 0.0f;
    cfg.hills_amplitude = 0.0f;
    cfg.hills_wavelength = 20.0f;
    cfg.washboard_length = 0.0f;
    cfg.washboard_amplitude = 0.0f;
    cfg.washboard_wavelength = 1.2f;
    testground_generate(&tg, &cfg);
    return tg;
}

static Testground make_washboard_testground(void) {
    Testground tg;
    TestgroundConfig cfg;
    fill_test_surfaces(&cfg);
    cfg.world_half_width = 25.0f;
    cfg.base_height = 0.0f;
    cfg.flat_length = 10.0f;
    cfg.hills_length = 0.0f;
    cfg.hills_amplitude = 0.0f;
    cfg.hills_wavelength = 20.0f;
    cfg.washboard_length = 40.0f;
    cfg.washboard_amplitude = 0.05f;
    cfg.washboard_wavelength = 1.2f;
    testground_generate(&tg, &cfg);
    return tg;
}

/* ---- suspension test harness: a "quarter car" ----------------------------
 * One RigidBody standing in for the portion of chassis mass carried by a
 * single wheel, one SuspensionConfig/State, gravity applied by the test
 * itself. vehicle.c (which would normally apply gravity and drive the
 * eight-step order) is a stub owned by a different agent and out of scope
 * for tests/test_physics.c, so this harness re-creates only the SLICE of
 * vehicle.h's canonical order that exercises suspension.c in isolation
 * (raycast/spring/damper, apply force, integrate) with the test's OWN
 * ground query -- not world/testground.h, which is a separate module owned
 * by a different agent again. */
typedef struct QuarterCar {
    RigidBody body;
    SuspensionConfig cfg;
    SuspensionState state;
} QuarterCar;

static bool flat_ground_query(void *userdata, f32 world_x, f32 world_z,
                               f32 *out_height, Vec3 *out_normal) {
    (void)userdata; (void)world_x; (void)world_z;
    *out_height = 0.0f;
    *out_normal = vec3_make(0.0f, 1.0f, 0.0f);
    return true;
}

static bool no_ground_query(void *userdata, f32 world_x, f32 world_z,
                             f32 *out_height, Vec3 *out_normal) {
    (void)userdata; (void)world_x; (void)world_z; (void)out_height; (void)out_normal;
    return false;
}

static void quarter_car_init(QuarterCar *qc, f32 mass, SuspensionConfig cfg, f32 start_height) {
    rigidbody_init(&qc->body, mass, vec3_make(1.0f, 1.0f, 1.0f)); /* inertia irrelevant: no torque exercised */
    qc->body.position = vec3_make(0.0f, start_height, 0.0f);
    qc->cfg = cfg;
    qc->state.compression = 0.0f;
    qc->state.compression_prev = 0.0f;
    qc->state.compression_velocity = 0.0f;
    qc->state.grounded = false;
    qc->state.contact_point = vec3_zero();
    qc->state.contact_normal = vec3_make(0.0f, 1.0f, 0.0f);
    qc->state.normal_load = 0.0f;
}

static void quarter_car_step(QuarterCar *qc, SuspensionGroundQuery gq, void *ud, f32 dt) {
    Vec3 force;
    rigidbody_clear_accumulators(&qc->body);
    force = suspension_solve(&qc->cfg, &qc->state, &qc->body, qc->body.position, gq, ud, dt);
    rigidbody_apply_force_at_point(&qc->body, force, qc->body.position);
    rigidbody_apply_force_at_point(&qc->body,
        vec3_make(0.0f, -qc->body.mass * TEST_GRAVITY, 0.0f), qc->body.position);
    rigidbody_step(&qc->body, dt);
}

/* ---- vehicle test harness -------------------------------------------- */

static void run_vehicle_settle(Vehicle *car, Testground *ground, const VehicleParams *params,
                                Vec3 start_pos, Vec3 start_vel, int steps) {
    InputState input;
    int i;
    input_init(&input);
    vehicle_init(car, params, testground_height_query, ground);
    car->chassis.position = start_pos;
    car->chassis.linear_velocity = start_vel;
    for (i = 0; i < steps; i++) {
        vehicle_step(car, &input, PHYSICS_DT);
    }
}

/*===================================================================================
 * timestep -- kept from the original smoke suite; already real and correct,
 * no change needed to reach "the real thing" for this tiny module.
 *===================================================================================*/

static void test_timestep_rate_is_compiled_in(void) {
    f32 expected_dt = 1.0f / (f32)PHYSICS_HZ;
    CHECK(PHYSICS_DT == expected_dt, "PHYSICS_DT must equal 1/PHYSICS_HZ");
    CHECK(PHYSICS_HZ == 60 || PHYSICS_HZ == 100 || PHYSICS_HZ == 120,
          "PHYSICS_HZ should be one of the three rates this project tests");
}

static void test_timestep_init_zeroes_state(void) {
    Timestep ts;
    timestep_init(&ts);
    CHECK(ts.accumulator == 0.0f, "timestep_init must zero accumulator");
    CHECK(ts.steps_this_frame == 0, "timestep_init must zero steps_this_frame");
}

/*===================================================================================
 * rigid body -- core/rigidbody.c is REAL as of this writing, so these are
 * expected to PASS. Kept honest by the fault-injection hooks (#1-#4).
 *===================================================================================*/

static void test_rigidbody_freefall_matches_analytic(void) {
    /* Semi-implicit Euler under a CONSTANT force has an EXACT closed form,
     * not just an approximation to compare against:
     *   v_n = -g*dt*n                         (each step adds the same
     *                                           constant acceleration)
     *   y_n = -g*dt^2 * n*(n+1)/2              (position uses the NEW
     *          velocity each step -- sum of an arithmetic series)
     * Tolerance is therefore tight: this is float32 round-off only, not
     * physics-model error. */
    RigidBody rb;
    const f32 mass = 1000.0f;
    const f32 dt = PHYSICS_DT;
    const int n = 200;
    f32 expected_vy, expected_y;
    int i;

    rigidbody_init(&rb, mass, vec3_make(600.0f, 900.0f, 1500.0f));
    rb.position = vec3_zero();

    for (i = 0; i < n; i++) {
        rigidbody_clear_accumulators(&rb);
        rigidbody_apply_force_at_point(&rb, vec3_make(0.0f, -mass * TEST_GRAVITY, 0.0f), rb.position);
        rigidbody_step(&rb, dt);
    }

    expected_vy = -TEST_GRAVITY * dt * (f32)n;
    expected_y  = -TEST_GRAVITY * dt * dt * ((f32)n * ((f32)n + 1.0f)) * 0.5f;

#if DIRT2_INJECT_FAULT == 1
    expected_vy *= 0.5f; /* deliberately wrong -- proves this CHECK can fail */
#endif
    CHECK(isfinite(rb.linear_velocity.y) && isfinite(rb.position.y), "free-fall state must stay finite");
    CHECK(nearly_equal_f32(rb.linear_velocity.y, expected_vy, 1e-3f, 1e-4f),
          "free-fall velocity after N steps must equal -g*dt*N exactly (semi-implicit Euler, constant force)");
    CHECK(nearly_equal_f32(rb.position.y, expected_y, 1e-2f, 1e-4f),
          "free-fall position after N steps must equal the exact discrete sum -g*dt^2*N*(N+1)/2");
}

static void test_rigidbody_offcentre_impulse_angular_velocity(void) {
    /* At identity orientation, inertia_world_inv reduces to the plain
     * diagonal inverse of inertia_body_diag (see rigidbody.c's own
     * R*I^-1*R^T formula with R == identity), so the expected angular
     * velocity after one step of a single off-centre force has an exact
     * closed form: omega = dt * (torque / inertia_diag) component-wise,
     * torque = r x F. */
    RigidBody rb;
    const Vec3 inertia = vec3_make(600.0f, 900.0f, 1500.0f);
    const f32 dt = PHYSICS_DT;
    Vec3 force, point, r, torque, expected_omega;

    rigidbody_init(&rb, 1000.0f, inertia);
    rb.position = vec3_zero();

    force = vec3_make(0.0f, 0.0f, 1000.0f);
    point = vec3_make(1.0f, 0.0f, 0.0f); /* 1m off-centre along +x */

    rigidbody_clear_accumulators(&rb);
    rigidbody_apply_force_at_point(&rb, force, point);

    r = vec3_sub(point, rb.position);
    torque = vec3_cross(r, force);
    expected_omega = vec3_make(dt * torque.x / inertia.x,
                                dt * torque.y / inertia.y,
                                dt * torque.z / inertia.z);

    rigidbody_step(&rb, dt);

#if DIRT2_INJECT_FAULT == 2
    expected_omega.y *= 2.0f;
#endif
    CHECK(vec3_is_finite(rb.angular_velocity), "off-centre impulse angular velocity must be finite");
    CHECK(nearly_equal_f32(rb.angular_velocity.x, expected_omega.x, 1e-6f, 1e-4f),
          "off-centre impulse: angular_velocity.x must match dt*(r x F)/Ixx");
    CHECK(nearly_equal_f32(rb.angular_velocity.y, expected_omega.y, 1e-6f, 1e-4f),
          "off-centre impulse: angular_velocity.y must match dt*(r x F)/Iyy");
    CHECK(nearly_equal_f32(rb.angular_velocity.z, expected_omega.z, 1e-6f, 1e-4f),
          "off-centre impulse: angular_velocity.z must match dt*(r x F)/Izz");
}

static void test_rigidbody_quaternion_stays_unit_over_10000_steps(void) {
    /* No external torque -- angular_velocity stays exactly constant with
     * this integrator (rigidbody.h step 2 has no gyroscopic coupling term).
     * quat_integrate deliberately does NOT renormalise internally (see
     * vecmath.h) -- rigidbody.c is contractually required to call
     * quat_normalize itself after every step. This test exists to catch a
     * missing/misplaced normalise: at a believable tumble rate, |q| would
     * visibly drift from 1.0 well before 10000 steps if that call were
     * skipped or only run every-other-step. */
    RigidBody rb;
    const int n = 10000;
    f32 max_dev = 0.0f;
    int i;

    rigidbody_init(&rb, 1000.0f, vec3_make(600.0f, 900.0f, 1500.0f));
    rb.position = vec3_zero();
    rb.angular_velocity = vec3_make(2.0f, 3.0f, -1.5f); /* rad/s, a believable-fast tumble */

    for (i = 0; i < n; i++) {
        f32 dev;
        rigidbody_clear_accumulators(&rb);
        rigidbody_step(&rb, PHYSICS_DT);
        dev = fabsf(quat_length(rb.orientation) - 1.0f);
        if (dev > max_dev) max_dev = dev;
    }

#if DIRT2_INJECT_FAULT == 3
    max_dev += 1.0f;
#endif
    /* Tolerance: quat_normalize is one sqrt+reciprocal, so renormalising
     * EVERY step leaves only ONE call's worth of float32 round-off
     * (epsilon ~1.19e-7) as residual, not 10000 steps of accumulated drift.
     * 1e-5 is ~100x that epsilon -- generous for the rsqrt path, but a
     * missing/skipped normalise call would produce deviations of 1e-2 or
     * larger within a few hundred steps at this angular rate, nowhere near
     * this threshold. */
    CHECK(max_dev < 1e-5f, "quaternion must stay unit-length (within float round-off) over 10000 steps");
}

static void test_rigidbody_asymmetric_inertia_spin_bounded(void) {
    /* With torque_accum == 0 every step, angular_velocity is exactly
     * CONSERVED by this integrator (no -omega x I*omega gyroscopic term is
     * modelled -- see the note in the previous test). So for THIS
     * integrator, "a freely-spinning body with unequal inertia stays
     * bounded" reduces to "angular speed does not drift" -- which is still
     * a real, useful check: it catches a bug where inertia_world_inv is
     * rebuilt incorrectly after quat_normalize (wrong rotation order,
     * forgetting to recompute it from the NEW orientation) and ends up
     * injecting or removing rotational energy over many steps even with
     * zero applied torque. */
    RigidBody rb;
    const int n = 5000;
    f32 speed0, speed_end;
    int i;

    rigidbody_init(&rb, 1000.0f, vec3_make(400.0f, 900.0f, 2200.0f)); /* deliberately unequal */
    rb.position = vec3_zero();
    rb.angular_velocity = vec3_make(1.0f, 4.0f, 0.3f);
    speed0 = vec3_length(rb.angular_velocity);

    for (i = 0; i < n; i++) {
        rigidbody_clear_accumulators(&rb);
        rigidbody_step(&rb, PHYSICS_DT);
    }
    speed_end = vec3_length(rb.angular_velocity);

#if DIRT2_INJECT_FAULT == 4
    speed_end *= 3.0f;
#endif
    CHECK(isfinite(speed_end), "free-spinning body: angular speed must stay finite");
    CHECK(nearly_equal_f32(speed_end, speed0, 1e-4f, 1e-3f),
          "free-spinning unequal-inertia body: angular speed must not drift with zero applied torque");
}

/*===================================================================================
 * suspension -- vehicle/suspension.c is currently a STUB (always returns
 * zero force, grounded=false). Every test below is expected to FAIL until
 * it is implemented; the assertions are written against suspension.h alone.
 *===================================================================================*/

static void test_suspension_settled_ride_height_matches_mg_over_k(void) {
    /* Static equilibrium: spring force == weight carried by the wheel,
     * k*compression_eq = m*g -> compression_eq = m*g/k. Exact regardless of
     * damping (damping only affects HOW it gets there). */
    QuarterCar qc;
    const f32 mass = 300.0f; /* one quarter of the 1200 kg test chassis used elsewhere in this file */
    SuspensionConfig cfg = make_test_suspension_config();
    const f32 expected_compression = mass * TEST_GRAVITY / cfg.spring_rate; /* ~0.0841 m, well under max_travel=0.15 m -- no bottom-out */
    const int settle_steps = (int)(5.0f * PHYSICS_HZ);
    const int window_steps = (int)(1.0f * PHYSICS_HZ);
    f32 min_c = 1e9f, max_c = -1e9f, sum_c = 0.0f, mean_c;
    int i;

    quarter_car_init(&qc, mass, cfg, cfg.rest_length + 0.5f); /* start well above rest_length+max_travel -> airborne, free-falls first */

    for (i = 0; i < settle_steps; i++) {
        quarter_car_step(&qc, flat_ground_query, NULL, PHYSICS_DT);
        if (i >= settle_steps - window_steps) {
            f32 c = qc.state.compression;
            if (c < min_c) min_c = c;
            if (c > max_c) max_c = c;
            sum_c += c;
        }
    }
    mean_c = sum_c / (f32)window_steps;

#if DIRT2_INJECT_FAULT == 5
    mean_c *= 2.0f;
#endif
    CHECK(qc.state.grounded, "settled wheel must report grounded == true");
    /* 2% relative tolerance: the analytic equilibrium is exact, but reaching
     * it takes finite simulated time. zeta = c/(2*sqrt(k*m)): compression
     * damper zeta ~= 3000/(2*sqrt(35000*300)) ~= 0.463, rebound
     * zeta ~= 4500/6480.7 ~= 0.694 -- time constant tau = 1/(zeta*omega_n),
     * omega_n = sqrt(k/m) ~= 10.8 rad/s, so tau is at most ~0.20s; 5
     * simulated seconds is >20 time constants, leaving a negligible
     * residual transient. 2% covers float32 round-off plus that sliver,
     * not model error. */
    CHECK(nearly_equal_f32(mean_c, expected_compression, 0.0005f, 0.02f),
          "settled compression must equal mass*g/spring_rate");
    /* "Settled" additionally means STOPPED oscillating: compression range
     * across the final simulated second must be under 0.1mm, an
     * arbitrary-but-tiny (~0.1%) slice of the 84mm equilibrium compression
     * that a suspension still visibly bouncing would not meet. */
    CHECK((max_c - min_c) < 0.0001f, "settled compression must stop changing (no residual oscillation)");
}

static void test_suspension_damped_frequency_matches_sqrt_k_over_m(void) {
    /* Symmetric, LIGHT damping used ONLY for this test (see comment below)
     * so the motion is close to a clean single-zeta SHO and the damped
     * frequency omega_n*sqrt(1-zeta^2) sits close to the plain undamped
     * natural frequency sqrt(k/m) the task asks to check against.
     * zeta = c/(2*sqrt(k*m)) = 800/(2*sqrt(35000*300)) ~= 0.1234, so
     * sqrt(1-zeta^2) ~= 0.9924 -- a 0.76% difference from the undamped
     * value, comfortably inside the 5% tolerance below. The production-like
     * ASYMMETRIC damping used in the settle test (damper_rebound !=
     * damper_compression) is deliberately not used here: it turns the
     * motion into a non-sinusoidal asymmetric oscillation with no single
     * clean frequency to compare against a closed-form formula. */
    QuarterCar qc;
    SuspensionConfig cfg = make_test_suspension_config();
    const f32 mass = 300.0f;
    const f32 k = cfg.spring_rate;
    const f32 compression_eq = mass * TEST_GRAVITY / k;
    const f32 perturb = 0.02f; /* 20mm displaced from equilibrium, released from rest */
    const f32 expected_freq_hz = sqrtf(k / mass) / (2.0f * PI_F);
    const int max_steps = (int)(3.0f * PHYSICS_HZ);
    f32 prev_dev, prev_t;
    f32 crossing_times[3];
    int crossings_found = 0;
    int i;

    cfg.damper_compression = 800.0f;
    cfg.damper_rebound = 800.0f;

    /* Start already displaced (rest_length - (compression_eq+perturb) above
     * ground) rather than dropping from height, so the first sample is a
     * clean "released from rest at a known offset" start, not an
     * uncontrolled fall transient. */
    quarter_car_init(&qc, mass, cfg, cfg.rest_length - (compression_eq + perturb));
    /* Prime compression_prev to the TRUE initial compression (rest_length
     * minus height above ground) so the first step's finite-difference
     * compression_velocity is ~0 (released from rest), not an artificial
     * spike from comparing against quarter_car_init's default 0. */
    qc.state.compression_prev = compression_eq + perturb;

    prev_dev = qc.state.compression_prev - compression_eq; /* == perturb, i.e. above equilibrium */
    prev_t = 0.0f;

    for (i = 0; i < max_steps && crossings_found < 3; i++) {
        f32 t, dev;
        quarter_car_step(&qc, flat_ground_query, NULL, PHYSICS_DT);
        t = (f32)(i + 1) * PHYSICS_DT;
        dev = qc.state.compression - compression_eq;
        if (prev_dev < 0.0f && dev >= 0.0f) {
            /* Rising zero-crossing: linearly interpolate the exact time. */
            f32 frac = -prev_dev / (dev - prev_dev);
            crossing_times[crossings_found++] = prev_t + frac * (t - prev_t);
        }
        prev_dev = dev;
        prev_t = t;
    }

    CHECK(crossings_found >= 3, "damped-frequency test must observe at least two full oscillation periods");
    if (crossings_found >= 3) {
        f32 period = (crossing_times[2] - crossing_times[0]) / 2.0f; /* average of 2 periods */
        f32 observed_freq_hz = (period > 0.0f) ? (1.0f / period) : -1.0f;
#if DIRT2_INJECT_FAULT == 6
        observed_freq_hz *= 1.5f;
#endif
        CHECK(nearly_equal_f32(observed_freq_hz, expected_freq_hz, 0.0f, 0.05f),
              "observed damped oscillation frequency must be within 5% of sqrt(k/m)/(2*pi)");
    }
}

static void test_suspension_normal_load_never_negative(void) {
    /* Sweep a wide range of penetration depths (near full extension through
     * well past bottom-out) and BOTH stroke directions (fast rebound vs
     * fast compression, via compression_prev bias) and confirm normal_load
     * is never negative for any of them -- a suspension can only PUSH the
     * chassis away from the ground (suspension.h: "clamped ... >= 0"). A
     * naive F = -k*x - c*v with no clamp WOULD go negative during a fast
     * rebound stroke on a lightly-compressed spring -- exactly the bug this
     * sweep exists to catch. */
    SuspensionConfig cfg = make_test_suspension_config();
    RigidBody dummy_chassis;
    int all_nonneg = 1;
    int i;
    const int n = 21;
    const f32 top = cfg.rest_length + 0.02f;                    /* near full extension */
    const f32 bottom = cfg.rest_length - cfg.max_travel - 0.05f; /* well past bottom-out */

    rigidbody_init(&dummy_chassis, 1000.0f, vec3_make(1.0f, 1.0f, 1.0f));
    dummy_chassis.position = vec3_zero();

    for (i = 0; i < n; i++) {
        SuspensionState state;
        Vec3 force;
        f32 t = (f32)i / (f32)(n - 1);
        f32 wheel_height = top + t * (bottom - top);
        int rebound_case = (i % 2 == 0);

        dummy_chassis.position = vec3_make(0.0f, wheel_height, 0.0f);
        state.compression = 0.0f;
        /* Bias compression_prev so the finite-difference velocity alternates
         * between a fast REBOUND stroke and a fast COMPRESSION stroke,
         * covering both directions of the asymmetric damper. */
        state.compression_prev = (cfg.rest_length - wheel_height) + (rebound_case ? 0.05f : -0.05f);
        state.compression_velocity = 0.0f;
        state.grounded = false;
        state.normal_load = 0.0f;
        state.contact_point = vec3_zero();
        state.contact_normal = vec3_make(0.0f, 1.0f, 0.0f);

        force = suspension_solve(&cfg, &state, &dummy_chassis, dummy_chassis.position,
                                  flat_ground_query, NULL, PHYSICS_DT);
        (void)force;
        if (state.normal_load < 0.0f) {
            all_nonneg = 0;
            fprintf(stderr, "  (sample %d: wheel_height=%.4f normal_load=%.4f)\n", i, wheel_height, state.normal_load);
        }
    }

#if DIRT2_INJECT_FAULT == 7
    all_nonneg = 0;
#endif
    CHECK(all_nonneg, "normal_load must never be negative, across a sweep of compression depths and both stroke directions");
}

static void test_suspension_airborne_reports_zero_load(void) {
    SuspensionConfig cfg = make_test_suspension_config();
    RigidBody dummy_chassis;
    SuspensionState state;
    Vec3 force;

    rigidbody_init(&dummy_chassis, 1000.0f, vec3_make(1.0f, 1.0f, 1.0f));
    dummy_chassis.position = vec3_zero();

    state.compression = 0.0f;
    state.compression_prev = 0.0f;
    state.compression_velocity = 0.0f;
    state.grounded = true;      /* deliberately wrong initial value, to prove solve() actually sets it */
    state.normal_load = 999.0f; /* deliberately wrong sentinel */
    state.contact_point = vec3_zero();
    state.contact_normal = vec3_make(0.0f, 1.0f, 0.0f);

    force = suspension_solve(&cfg, &state, &dummy_chassis, dummy_chassis.position,
                              no_ground_query, NULL, PHYSICS_DT);

#if DIRT2_INJECT_FAULT == 8
    state.normal_load = 1.0f;
#endif
    CHECK(state.grounded == false, "a wheel with no ground in range must report grounded == false");
    CHECK(state.normal_load == 0.0f, "an airborne wheel must report EXACTLY zero normal load");
    CHECK(vec3_length(force) == 0.0f, "an airborne wheel must apply exactly zero suspension force");
}

static void test_suspension_bottom_out_is_progressive(void) {
    /* Sweep penetration in small, EQUAL steps from before max_travel to
     * well past it, at a slow constant closing rate (quasi-static), and
     * confirm the force curve's slope increases at max_travel but never
     * JUMPS discontinuously. A hard clamp (a common wrong shortcut) would
     * instead show one huge one-sample delta right at the boundary. */
    SuspensionConfig cfg = make_test_suspension_config();
    RigidBody dummy_chassis;
    const int n = 61;
    const f32 step_depth = 0.005f; /* 5mm per sample */
    f32 forces[61];
    f32 max_delta = 0.0f, sum_delta = 0.0f, mean_delta;
    int i, delta_count = 0;
    int crossed_max_travel = 0;
    int monotonic = 1;

    rigidbody_init(&dummy_chassis, 1000.0f, vec3_make(1.0f, 1.0f, 1.0f));
    dummy_chassis.position = vec3_zero();

    for (i = 0; i < n; i++) {
        SuspensionState state;
        Vec3 force;
        f32 compression_target = 0.10f + (f32)i * step_depth; /* sweeps 0.10m..0.40m; max_travel=0.15m sits inside */
        f32 wheel_height = cfg.rest_length - compression_target;

        dummy_chassis.position = vec3_make(0.0f, wheel_height, 0.0f);
        state.compression = 0.0f;
        state.compression_prev = compression_target - step_depth; /* small, constant closing rate */
        state.compression_velocity = 0.0f;
        state.grounded = false;
        state.normal_load = 0.0f;
        state.contact_point = vec3_zero();
        state.contact_normal = vec3_make(0.0f, 1.0f, 0.0f);

        force = suspension_solve(&cfg, &state, &dummy_chassis, dummy_chassis.position,
                                  flat_ground_query, NULL, PHYSICS_DT);
        forces[i] = vec3_length(force);
        if (compression_target >= cfg.max_travel && (compression_target - step_depth) < cfg.max_travel) {
            crossed_max_travel = 1;
        }
    }

    for (i = 1; i < n; i++) {
        f32 delta = fabsf(forces[i] - forces[i - 1]);
        sum_delta += delta;
        if (delta > max_delta) max_delta = delta;
        delta_count++;
        if (forces[i] < forces[i - 1] - 1e-3f) monotonic = 0;
    }
    mean_delta = sum_delta / (f32)delta_count;

#if DIRT2_INJECT_FAULT == 9
    max_delta *= 20.0f;
#endif
    if (!monotonic || max_delta >= mean_delta * 8.0f) {
        for (i = 1; i < n; i++) {
            f32 compression_target_i = 0.10f + (f32)i * step_depth;
            if (forces[i] < forces[i - 1] - 1e-3f || fabsf(forces[i] - forces[i - 1]) > mean_delta * 8.0f) {
                fprintf(stderr, "  (i=%d compression=%.4f force[i-1]=%.2f force[i]=%.2f delta=%.2f mean_delta=%.4f)\n",
                        i, compression_target_i, forces[i - 1], forces[i], forces[i] - forces[i - 1], mean_delta);
            }
        }
    }
    CHECK(crossed_max_travel, "bottom-out sweep must actually cross max_travel (test setup sanity)");
    /* No single step's force delta may exceed 8x the sweep's mean delta --
     * generous enough that the genuinely stiffer slope past max_travel
     * (bottom_out_spring_rate is ~5.7x spring_rate in this config) doesn't
     * itself trip the check, while a true discontinuity (a hard clamp, or
     * an off-by-one at the boundary) -- which produces a single-sample
     * spike much larger than even the steepest continuous slope in this
     * sweep -- still does. */
    CHECK(max_delta < mean_delta * 8.0f, "bottom-out transition must be progressive, not a discontinuous force jump");
    CHECK(monotonic, "suspension force must increase monotonically with penetration depth");
}

static void test_suspension_stable_across_all_physics_rates(void) {
    /* A deliberately stiff damper relative to mass and PHYSICS_DT, chosen
     * so an EXPLICIT (current-velocity) damper-force discretisation
     * overshoots and flips sign every step at 60 Hz specifically, while
     * staying stable at 100/120 Hz -- the classic "stiff spring/damper
     * unstable at low physics rate" failure this project's rate-
     * independence requirement exists to catch (docs/CODEMAP.md,
     * core/timestep.h).
     *
     * DERIVATION: for a damper force F = -c*v applied via explicit
     * (current-velocity) discretisation, each step's velocity update is
     * v_new = v - (c/m)*v*dt = v*(1 - c*dt/m). This is non-oscillating-
     * decaying only while |1 - c*dt/m| < 1, i.e. 0 < c*dt/m < 2 -- past
     * c*dt/m == 2 the damper overshoots zero velocity and grows in
     * magnitude every step. Solving c > 2m/dt for dt=1/60 and this file's
     * 300 kg quarter-car mass: c > 2*300*60 = 36000 N*s/m. This test uses
     * damper_compression = damper_rebound = 50000 N*s/m: c*dt/m = 2.78 at
     * 60 Hz (past the threshold), 1.67 at 100 Hz, 1.39 at 120 Hz (both
     * inside it). So this SAME test, compiled at all three PHYSICS_HZ
     * values by `make test-rates`, is expected to FAIL at 60 Hz and PASS at
     * 100/120 Hz against a naive explicit damper -- exactly the rate-
     * dependent bug this suite must be able to catch. */
    QuarterCar qc;
    SuspensionConfig cfg = make_test_suspension_config();
    const f32 mass = 300.0f;
    const int n = (int)(3.0f * PHYSICS_HZ);
    f32 max_abs_compression = 0.0f;
    int i;

    cfg.damper_compression = 50000.0f;
    cfg.damper_rebound = 50000.0f;

    quarter_car_init(&qc, mass, cfg, cfg.rest_length - 0.05f); /* start already 50mm compressed, at rest */
    qc.state.compression_prev = 0.05f;

    for (i = 0; i < n; i++) {
        f32 ac;
        quarter_car_step(&qc, flat_ground_query, NULL, PHYSICS_DT);
        ac = fabsf(qc.state.compression);
        if (ac > max_abs_compression) max_abs_compression = ac;
    }

#if DIRT2_INJECT_FAULT == 10
    max_abs_compression = 1000.0f;
#endif
    /* A stable suspension settles to compression ~=84mm (mass*g/k) and
     * never revisits anything close to max_travel (150mm) from this mild a
     * 50mm starting perturbation. An unstable (overshooting) damper
     * diverges geometrically and blows well past max_travel within a
     * handful of steps -- 0.5m is ~3.3x max_travel, unreachable by any
     * stable response to a 50mm offset but trivially exceeded by an
     * exponentially growing oscillation within 3 simulated seconds. */
    CHECK(max_abs_compression < 0.5f,
          "suspension must remain numerically stable (bounded compression) at the compiled PHYSICS_HZ, even with a stiff damper");
}

/*===================================================================================
 * tyre -- vehicle/tyre.c is currently a STUB (always returns {0,0}). Every
 * test below is expected to FAIL until it is implemented.
 *===================================================================================*/

static void test_tyre_peak_force_matches_mu_times_fz(void) {
    TyreSurfaceParams surf = make_test_tyre_surface();
    TyreSlipInput in;
    TyreForceOutput out;
    const f32 fz = 4000.0f;
    f32 mag;

    in.slip_ratio = surf.peak_slip_ratio; /* rho == 1, pure longitudinal */
    in.slip_angle = 0.0f;
    in.normal_load = fz;

    out = tyre_solve(&surf, in);
    mag = sqrtf(out.fx * out.fx + out.fy * out.fy);

#if DIRT2_INJECT_FAULT == 11
    mag *= 0.3f;
#endif
    CHECK(isfinite(out.fx) && isfinite(out.fy), "tyre force at peak slip must be finite");
    /* tyre.h: "peak_mu: peak combined friction coefficient AT rho == 1
     * (force / normal_load at the curve's maximum)" -- a contractual exact
     * value, so a tight 5% tolerance is appropriate (only the lookup
     * table's lerp quantisation between entries, per tyre.h's note on why a
     * small table + lerp is used instead of an exact closed form). */
    CHECK(nearly_equal_f32(mag, surf.peak_mu * fz, 0.0f, 0.05f),
          "force magnitude at rho==1 (pure slip_ratio at peak_slip_ratio) must be close to peak_mu * Fz");
}

static void test_tyre_combined_slip_stays_inside_friction_ellipse(void) {
    /* THE critical regression test for tyre.h's reason for existing: feed
     * BOTH slip_ratio and slip_angle at a fraction of their own peaks
     * SIMULTANEOUSLY. Two independently-summed 1-D curves (the bug this
     * single-curve interface exists to prevent) would return up to
     * sqrt(2)*peak_mu*Fz at rho~=1.41 -- physically impossible. A correct
     * combined-slip solve never exceeds peak_mu*Fz (the curve's global
     * maximum) for ANY input combination. */
    TyreSurfaceParams surf = make_test_tyre_surface();
    const f32 fz = 4000.0f;
    const f32 max_allowed = surf.peak_mu * fz * 1.05f; /* 5% margin for lookup-table interpolation overshoot near the peak */
    int all_inside = 1;
    int i, j;
    const int n = 9;
    TyreSlipInput half_in;
    TyreForceOutput half_out;
    f32 half_mag;

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            TyreSlipInput in;
            TyreForceOutput out;
            f32 mag;
            f32 sr_frac = -1.0f + 2.0f * (f32)i / (f32)(n - 1); /* -1..+1 of peak_slip_ratio */
            f32 sa_frac = -1.0f + 2.0f * (f32)j / (f32)(n - 1); /* -1..+1 of peak_slip_angle */

            in.slip_ratio = sr_frac * surf.peak_slip_ratio;
            in.slip_angle = sa_frac * surf.peak_slip_angle;
            in.normal_load = fz;
            out = tyre_solve(&surf, in);
            mag = sqrtf(out.fx * out.fx + out.fy * out.fy);
            if (!isfinite(mag) || mag > max_allowed) {
                all_inside = 0;
                fprintf(stderr, "  (sr_frac=%.2f sa_frac=%.2f mag=%.2f max_allowed=%.2f)\n",
                        sr_frac, sa_frac, mag, max_allowed);
            }
        }
    }

#if DIRT2_INJECT_FAULT == 12
    all_inside = 0;
#endif
    CHECK(all_inside, "combined-slip force must never exceed peak_mu * Fz for ANY slip_ratio/slip_angle combination (friction ellipse)");

    /* Specifically the double-counting failure mode: at HALF of each peak
     * simultaneously (rho = sqrt(0.5^2+0.5^2) ~= 0.707, still under the
     * rho==1 peak), two independently-summed curves would already be close
     * to double the correct magnitude at that point. Sharpest single sample
     * for catching that specific bug. */
    half_in.slip_ratio = 0.5f * surf.peak_slip_ratio;
    half_in.slip_angle = 0.5f * surf.peak_slip_angle;
    half_in.normal_load = fz;
    half_out = tyre_solve(&surf, half_in);
    half_mag = sqrtf(half_out.fx * half_out.fx + half_out.fy * half_out.fy);
#if DIRT2_INJECT_FAULT == 13
    half_mag = max_allowed * 2.0f;
#endif
    CHECK(half_mag <= max_allowed, "half-peak combined slip must not double-count grip");
}

static void test_tyre_force_scales_with_fz_pow_0_8(void) {
    /* A real tyre's force does not scale linearly with load -- doubling Fz
     * does not double peak force (load sensitivity). Fz^0.8 is the
     * standard vehicle-dynamics approximation for this. At fixed slip
     * (rho==1), force(Fz2)/force(Fz1) should be close to (Fz2/Fz1)^0.8, NOT
     * (Fz2/Fz1)^1.0 -- this is the check that fails if tyre_solve just does
     * force = mu*Fz linearly with no load sensitivity at all. */
    TyreSurfaceParams surf = make_test_tyre_surface();
    TyreSlipInput in;
    TyreForceOutput out1, out2;
    const f32 fz1 = 2000.0f, fz2 = 8000.0f; /* 4x load */
    f32 mag1, mag2, ratio, expected_ratio;

    in.slip_ratio = surf.peak_slip_ratio;
    in.slip_angle = 0.0f;

    in.normal_load = fz1;
    out1 = tyre_solve(&surf, in);
    mag1 = sqrtf(out1.fx * out1.fx + out1.fy * out1.fy);

    in.normal_load = fz2;
    out2 = tyre_solve(&surf, in);
    mag2 = sqrtf(out2.fx * out2.fx + out2.fy * out2.fy);

    expected_ratio = powf(fz2 / fz1, 0.8f); /* 4^0.8 ~= 3.0314 */
    ratio = (mag1 > 1e-6f) ? (mag2 / mag1) : -1.0f;

#if DIRT2_INJECT_FAULT == 14
    ratio = fz2 / fz1; /* deliberately wrong: linear scaling */
#endif
    /* 10% tolerance: Fz^0.8 is the textbook APPROXIMATION this project's
     * peak_mu curve is expected to roughly follow, not a value tyre.h
     * mandates to the decimal place -- unlike peak_mu itself, this is a
     * real physical property (load sensitivity), not an exact contractual
     * number. */
    CHECK(nearly_equal_f32(ratio, expected_ratio, 0.0f, 0.10f),
          "force must scale roughly with Fz^0.8 (load sensitivity), not linearly with Fz");
}

static void test_tyre_zero_slip_gives_zero_force_no_nan(void) {
    TyreSurfaceParams surf = make_test_tyre_surface();
    TyreSlipInput in;
    TyreForceOutput out;

    in.slip_ratio = 0.0f;
    in.slip_angle = 0.0f;
    in.normal_load = 5000.0f;

    out = tyre_solve(&surf, in);

#if DIRT2_INJECT_FAULT == 15
    out.fx = NAN;
#endif
    CHECK(isfinite(out.fx) && isfinite(out.fy), "zero slip must not produce NaN/Inf (rho==0 degenerate case)");
    /* Exact equality is intentional here, not a float anti-pattern: tyre.h
     * explicitly promises {0, 0} exactly for this degenerate case, not
     * "close to zero". */
    CHECK(out.fx == 0.0f, "zero slip_ratio and slip_angle must give exactly zero fx");
    CHECK(out.fy == 0.0f, "zero slip_ratio and slip_angle must give exactly zero fy");
}

/*===================================================================================
 * drivetrain -- vehicle/drivetrain.c is currently a STUB (always returns
 * zero drive/brake torque, never touches engine_rpm). Every test below is
 * expected to FAIL until it is implemented.
 *===================================================================================*/

static void test_drivetrain_rpm_tracks_wheel_speed_through_ratio(void) {
    /* Phase 1 has no clutch/gearbox model (drivetrain.h: "a single fixed
     * gear ratio ... enough to drive the test plane"), so the engine is
     * effectively rigidly coupled to the driven wheels through
     * gear_ratio*final_drive_ratio: once any startup transient has settled,
     * engine_rpm must equal driven_wheel_omega_avg (rad/s) converted
     * through that ratio to RPM. Driving at a constant wheel speed for 2
     * simulated seconds is long enough for any reasonable smoothing/ramp to
     * converge, at any of the three tested PHYSICS_HZ rates. */
    DrivetrainConfig cfg = make_test_drivetrain_config();
    DrivetrainState state;
    DrivetrainWheelTorque torque[VEHICLE_WHEEL_COUNT];
    const f32 wheel_omega = 40.0f; /* rad/s -- see expected_rpm comment for why this is a safe mid-range pick */
    const int n = (int)(2.0f * PHYSICS_HZ);
    f32 expected_rpm;
    int i;

    state.engine_rpm = cfg.idle_rpm;

    for (i = 0; i < n; i++) {
        drivetrain_update(&cfg, &state, 0.5f, 0.0f, wheel_omega, PHYSICS_DT, torque);
    }

    /* 40 * 3.5 * 4.1 * (60/2pi) ~= 5479 RPM -- comfortably between idle_rpm
     * (900) and max_rpm (7500), so no clamping ambiguity. */
    expected_rpm = wheel_omega * cfg.gear_ratio * cfg.final_drive_ratio * (60.0f / (2.0f * PI_F));

#if DIRT2_INJECT_FAULT == 16
    expected_rpm *= 2.0f;
#endif
    CHECK(isfinite(state.engine_rpm), "engine_rpm must stay finite");
    CHECK(nearly_equal_f32(state.engine_rpm, expected_rpm, 0.0f, 0.05f),
          "engine_rpm must track driven wheel speed through gear_ratio*final_drive_ratio");
}

static void test_drivetrain_brake_torque_scales_with_brake_input(void) {
    /* brake_torque must be a MAGNITUDE (>=0, drivetrain.h) that scales with
     * the brake input -- checked at two brake levels across all four wheels
     * to catch a stub or a saturated-at-one-value implementation. */
    DrivetrainConfig cfg = make_test_drivetrain_config();
    DrivetrainState state;
    DrivetrainWheelTorque torque_half[VEHICLE_WHEEL_COUNT];
    DrivetrainWheelTorque torque_full[VEHICLE_WHEEL_COUNT];
    int w;
    int all_nonneg = 1, scales_up = 1;

    for (w = 0; w < VEHICLE_WHEEL_COUNT; w++) {
        state.engine_rpm = cfg.idle_rpm;
        drivetrain_update(&cfg, &state, 0.0f, 0.5f, 20.0f, PHYSICS_DT, torque_half);
        state.engine_rpm = cfg.idle_rpm;
        drivetrain_update(&cfg, &state, 0.0f, 1.0f, 20.0f, PHYSICS_DT, torque_full);

        if (torque_half[w].brake_torque < 0.0f || torque_full[w].brake_torque < 0.0f) all_nonneg = 0;
        if (!(torque_full[w].brake_torque > torque_half[w].brake_torque)) scales_up = 0;
    }

#if DIRT2_INJECT_FAULT == 17
    scales_up = 0;
#endif
    CHECK(all_nonneg, "brake_torque must always be a non-negative magnitude");
    CHECK(scales_up, "brake_torque at brake=1.0 must exceed brake_torque at brake=0.5 for every wheel");
}

/* Mirrors vehicle.h step 8's DOCUMENTED brake-opposition semantics (brake
 * torque is a magnitude; the CALLER opposes it to the wheel's current spin
 * direction and must not let it overshoot past zero into reverse spin).
 * Lives HERE, not in vehicle.c, because vehicle.c belongs to a different
 * agent and is out of scope for tests/test_physics.c -- this function only
 * gives drivetrain_update's brake_torque OUTPUT a well-defined consumer so
 * "decelerates a wheel to zero and does not reverse it" is testable against
 * drivetrain.h's contract alone. */
static void integrate_wheel_with_brake_no_reverse(f32 *omega, f32 brake_torque_mag,
                                                    f32 wheel_inertia, f32 dt) {
    f32 sign = (*omega > 0.0f) ? 1.0f : ((*omega < 0.0f) ? -1.0f : 0.0f);
    f32 opposing_torque = -sign * brake_torque_mag;
    f32 new_omega = *omega + (opposing_torque / wheel_inertia) * dt;
    if ((*omega > 0.0f && new_omega < 0.0f) || (*omega < 0.0f && new_omega > 0.0f)) {
        new_omega = 0.0f; /* a brake stops a wheel, never spins it the other way */
    }
    *omega = new_omega;
}

static void test_drivetrain_brake_decelerates_wheel_to_zero_without_reversing(void) {
    DrivetrainConfig cfg = make_test_drivetrain_config();
    DrivetrainState state;
    DrivetrainWheelTorque torque[VEHICLE_WHEEL_COUNT];
    const f32 wheel_inertia = 1.2f; /* matches this file's vehicle_params wheel_mass_moment_of_inertia */
    f32 omega = 80.0f; /* rad/s, a believable rolling speed */
    int reversed = 0;
    int i;
    const int n = (int)(3.0f * PHYSICS_HZ); /* generous -- must reach zero well before this */
    int reached_zero_step = -1;

    state.engine_rpm = cfg.idle_rpm;

    for (i = 0; i < n; i++) {
        drivetrain_update(&cfg, &state, 0.0f, 1.0f, omega, PHYSICS_DT, torque);
        integrate_wheel_with_brake_no_reverse(&omega, torque[WHEEL_FL].brake_torque, wheel_inertia, PHYSICS_DT);
        if (omega < -1e-6f) reversed = 1;
        if (reached_zero_step < 0 && omega == 0.0f) reached_zero_step = i;
    }

#if DIRT2_INJECT_FAULT == 18
    reached_zero_step = -1;
#endif
    CHECK(!reversed, "a braked wheel must never spin in reverse");
    CHECK(reached_zero_step >= 0, "full brake must bring the wheel to a complete stop within 3 simulated seconds");
}

/*===================================================================================
 * vehicle -- the whole car. vehicle.c's step ORDER is real (see vehicle.h's
 * canonical order comment) but its per-step MATH is still mostly a stub
 * (zero slip, wheel spin never integrated, gravity never applied anywhere
 * in the current vehicle_step), and it depends on suspension.c/tyre.c/
 * testground.c, all stubs too -- every test below is expected to FAIL until
 * the whole chain is implemented.
 *===================================================================================*/

static void test_vehicle_settles_on_flat_ground_and_stays(void) {
    /* Gravity is NOT part of vehicle.h's numbered per-frame contract (see
     * this suite's final report) -- wherever vehicle.c ends up applying it,
     * the OBSERVABLE requirement is unambiguous regardless of mechanism: a
     * car dropped onto flat ground settles to a steady ride height and
     * stays there. This test only depends on that observable outcome. */
    Testground ground = make_flat_testground(200.0f);
    VehicleParams params = make_test_vehicle_params();
    Vehicle car;
    InputState input;
    Vec3 start = vec3_make(0.0f, params.suspension[WHEEL_FL].rest_length + 0.5f, 5.0f);
    const int pre_steps = (int)(4.0f * PHYSICS_HZ);
    const int window_steps = (int)(1.0f * PHYSICS_HZ);
    f32 min_y = 1e9f, max_y = -1e9f;
    f32 per_wheel_weight, expected_compression, expected_y;
    int i;

    run_vehicle_settle(&car, &ground, &params, start, vec3_zero(), pre_steps);
    input_init(&input);

    for (i = 0; i < window_steps; i++) {
        vehicle_step(&car, &input, PHYSICS_DT);
        if (car.chassis.position.y < min_y) min_y = car.chassis.position.y;
        if (car.chassis.position.y > max_y) max_y = car.chassis.position.y;
    }

    /* Expected settled ride height: mount_point_body is zero for every
     * wheel in this file's test config, so the chassis origin sits at
     * rest_length minus the per-wheel compression that carries a quarter
     * of the chassis weight. */
    per_wheel_weight = params.chassis_mass * TEST_GRAVITY / 4.0f;
    expected_compression = per_wheel_weight / params.suspension[WHEEL_FL].spring_rate;
    expected_y = ground.config.base_height + params.suspension[WHEEL_FL].rest_length - expected_compression;

#if DIRT2_INJECT_FAULT == 19
    expected_y += 1.0f;
#endif
    fprintf(stderr, "  (vehicle settle: actual_y=%.5f expected_y=%.5f diff=%.5f min_y=%.5f max_y=%.5f vy=%.5f)\n",
            car.chassis.position.y, expected_y, car.chassis.position.y - expected_y, min_y, max_y, car.chassis.linear_velocity.y);
    CHECK(isfinite(car.chassis.position.y), "settled chassis height must be finite");
    CHECK(nearly_equal_f32(car.chassis.position.y, expected_y, 0.01f, 0.05f),
          "vehicle must settle to a ride height matching per-wheel mass*g/spring_rate");
    CHECK((max_y - min_y) < 0.005f, "settled ride height must stop changing (no residual bounce/jitter/sink) over the final second");
    CHECK(fabsf(car.chassis.position.x - start.x) < 0.05f, "vehicle must not drift sideways under zero input while settling");
    CHECK(fabsf(car.chassis.position.z - start.z) < 0.05f, "vehicle must not drift forward/back under zero input while settling");
    CHECK(fabsf(car.chassis.linear_velocity.y) < 0.02f, "settled vertical velocity must be near zero");
}

static void test_vehicle_sum_of_normal_loads_equals_weight_at_rest(void) {
    Testground ground = make_flat_testground(200.0f);
    VehicleParams params = make_test_vehicle_params();
    Vehicle car;
    Vec3 start = vec3_make(0.0f, params.suspension[WHEEL_FL].rest_length + 0.5f, 5.0f);
    const int steps = (int)(5.0f * PHYSICS_HZ);
    f32 total_load = 0.0f;
    f32 expected_weight;
    int i;

    run_vehicle_settle(&car, &ground, &params, start, vec3_zero(), steps);

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        total_load += car.wheels[i].suspension.normal_load;
    }

    expected_weight = params.chassis_mass * TEST_GRAVITY;
#if DIRT2_INJECT_FAULT == 20
    expected_weight *= 3.0f;
#endif
    fprintf(stderr, "  (sum of loads: total_load=%.3f expected_weight=%.3f per_wheel=[%.3f %.3f %.3f %.3f])\n",
            total_load, expected_weight,
            car.wheels[0].suspension.normal_load, car.wheels[1].suspension.normal_load,
            car.wheels[2].suspension.normal_load, car.wheels[3].suspension.normal_load);
    CHECK(isfinite(total_load), "summed normal loads must be finite");
    /* 5%: allows for antiroll/auto-level interactions and the settle window
     * not being perfectly at equilibrium, without masking a genuinely wrong
     * total (e.g. only summing 2 of 4 wheels, or double-counting an axle). */
    CHECK(nearly_equal_f32(total_load, expected_weight, 0.0f, 0.05f),
          "sum of all four wheels' normal_load must equal chassis weight (mass * g) at rest");
}

static void test_vehicle_kinetic_energy_does_not_grow_with_zero_input(void) {
    /* Drop from height so the car actually HAS motion/energy to dissipate
     * (sitting at KE==0 the whole time would make "does not grow" pass even
     * for a completely frozen vehicle, proving nothing). Sample kinetic
     * energy at checkpoints AFTER the initial fall/impact transient and
     * confirm it never grows beyond a small margin of the previous
     * checkpoint -- catches the classic numerical-instability failure mode
     * where an under-damped step pumps energy into the system over time
     * instead of dissipating it. */
    Testground ground = make_flat_testground(200.0f);
    VehicleParams params = make_test_vehicle_params();
    Vehicle car;
    InputState input;
    const int checkpoint_steps[4] = {
        (int)(1.0f * PHYSICS_HZ), (int)(2.0f * PHYSICS_HZ),
        (int)(4.0f * PHYSICS_HZ), (int)(8.0f * PHYSICS_HZ)
    };
    const f32 drop_height = 0.4f;
    f32 ke[4];
    f32 max_ang_speed = 0.0f;
    f32 initial_pe;
    int cp = 0;
    int i;
    int non_growing = 1;
    int total_steps = checkpoint_steps[3];

    vehicle_init(&car, &params, testground_height_query, &ground);
    car.chassis.position = vec3_make(0.0f, params.suspension[WHEEL_FL].rest_length + drop_height, 5.0f);
    input_init(&input);

    for (i = 0; i < total_steps; i++) {
        f32 ang_speed;
        vehicle_step(&car, &input, PHYSICS_DT);
        ang_speed = vec3_length(car.chassis.angular_velocity);
        if (ang_speed > max_ang_speed) max_ang_speed = ang_speed;
        if (cp < 4 && i + 1 == checkpoint_steps[cp]) {
            ke[cp] = 0.5f * car.chassis.mass * vec3_length_sq(car.chassis.linear_velocity);
            cp++;
        }
    }

    for (i = 1; i < 4; i++) {
        /* 20% margin per checkpoint (each covers double the previous
         * checkpoint's simulated time): generous enough to absorb sampling
         * noise near a settle window, tight enough that a genuine runaway
         * (an unstable integrator grows each checkpoint by a large
         * multiple, not 20%) still trips it. +1.0 J floor avoids penalising
         * near-zero settled KE for relative-noise reasons. */
        if (ke[i] > ke[i - 1] * 1.2f + 1.0f) non_growing = 0;
    }
    initial_pe = params.chassis_mass * TEST_GRAVITY * drop_height; /* rough energy budget the drop actually released */

#if DIRT2_INJECT_FAULT == 21
    non_growing = 0;
#endif
    CHECK(isfinite(ke[3]) && isfinite(max_ang_speed), "kinetic energy and angular speed must stay finite throughout");
    CHECK(non_growing, "kinetic energy must not grow across checkpoints with zero driver input (no energy source)");
    CHECK(ke[3] < initial_pe * 2.0f + 10.0f, "kinetic energy must stay within a sane bound of the energy the drop actually released");
}

static void test_vehicle_stable_crossing_washboard_zone(void) {
    /* Given directly to the chassis rather than produced by the (still
     * stubbed) drivetrain, so this test isolates suspension/washboard
     * stability specifically -- combining it with an also-stubbed
     * drivetrain path would make a failure ambiguous about which subsystem
     * broke. */
    Testground ground = make_washboard_testground();
    VehicleParams params = make_test_vehicle_params();
    Vehicle car;
    InputState input;
    const f32 forward_speed = 6.0f; /* m/s, a plausible test-plane crawl speed */
    const f32 washboard_start_z = 10.0f; /* == make_washboard_testground's flat_length */
    const int steps = (int)((((ground.config.washboard_length + 10.0f) / forward_speed) + 1.0f) * PHYSICS_HZ);
    int all_finite = 1;
    int all_bounded = 1;
    f32 max_quat_dev = 0.0f;
    int i;

    vehicle_init(&car, &params, testground_height_query, &ground);
    car.chassis.position = vec3_make(0.0f, params.suspension[WHEEL_FL].rest_length + 0.05f, washboard_start_z);
    car.chassis.linear_velocity = vec3_make(0.0f, 0.0f, forward_speed);
    input_init(&input);

    for (i = 0; i < steps; i++) {
        RigidBody *rb = &car.chassis;
        f32 qdev;
        vehicle_step(&car, &input, PHYSICS_DT);

        if (!vec3_is_finite(rb->position) || !vec3_is_finite(rb->linear_velocity) ||
            !vec3_is_finite(rb->angular_velocity) || !quat_is_finite(rb->orientation)) {
            all_finite = 0;
            break;
        }
        if (fabsf(rb->position.x) > ground.config.world_half_width + 5.0f ||
            rb->position.y < -5.0f || rb->position.y > 10.0f ||
            rb->position.z > washboard_start_z + ground.config.washboard_length + 20.0f) {
            all_bounded = 0;
            break;
        }
        qdev = fabsf(quat_length(rb->orientation) - 1.0f);
        if (qdev > max_quat_dev) max_quat_dev = qdev;
    }

#if DIRT2_INJECT_FAULT == 22
    all_finite = 0;
#endif
    CHECK(all_finite, "driving across the washboard zone must never produce NaN/Inf in position, velocity or orientation");
    CHECK(all_bounded, "driving across the washboard zone must keep the chassis within sane world bounds (no launching to infinity)");
    CHECK(max_quat_dev < 1e-4f, "orientation quaternion must stay unit-length while crossing the washboard");
}

/* A NaN rho must NOT index the tyre table. This is the regression test for a
 * real SEGV found during v0.1 integration: NaN fails every comparison, so it
 * passed straight through both of tyre_lut_lookup's range guards and reached
 * (int)NaN, indexing catastrophically out of bounds. AddressSanitizer
 * reported "SEGV on unknown address ... READ memory access" at
 * tyre_lut.c:593. Without the guard this test does not fail politely -- it
 * takes the whole suite down, which is exactly the point. */
static void test_tyre_lut_survives_nonfinite_rho(void) {
    float zero = 0.0f;
    float nan_rho = zero / zero;          /* runtime NaN, not a constant the
                                            * optimiser can fold away        */
    float inf_rho = 1.0f / zero;
    float v_nan = tyre_lut_lookup(TYRE_TUNING_ARCADE, TYRE_SURFACE_TARMAC_DRY, nan_rho);
    float v_inf = tyre_lut_lookup(TYRE_TUNING_ARCADE, TYRE_SURFACE_TARMAC_DRY, inf_rho);
    float v_neg = tyre_lut_lookup(TYRE_TUNING_ARCADE, TYRE_SURFACE_TARMAC_DRY, -5.0f);

    CHECK(isfinite(v_nan), "tyre_lut_lookup must return a finite value for a NaN rho, not read out of bounds");
    CHECK(isfinite(v_inf), "tyre_lut_lookup must return a finite value for an infinite rho");
    CHECK(isfinite(v_neg), "tyre_lut_lookup must return a finite value for a negative rho");
    CHECK(v_nan >= 0.0f && v_nan <= 1.0f, "tyre_lut_lookup's NaN fallback must be a real table value in 0..1");
}

/* Every field vehicle.c reads out of VehicleParams must actually be set by
 * the constructors. Reads a fresh params through the same path the sim does
 * and drives it -- an unset slip_speed_floor here reappears as the NaN that
 * crashed the suite. */
static void test_vehicle_params_are_fully_initialised(void) {
    VehicleParams p = make_test_vehicle_params();

    CHECK(isfinite(p.max_steer_angle) && p.max_steer_angle > 0.0f,
          "VehicleParams.max_steer_angle must be set to a positive finite value");
    CHECK(isfinite(p.slip_speed_floor) && p.slip_speed_floor > 0.0f,
          "VehicleParams.slip_speed_floor must be set to a positive finite value (a zero or garbage floor makes slip NaN)");
    CHECK(isfinite(p.handbrake_torque) && p.handbrake_torque > 0.0f,
          "VehicleParams.handbrake_torque must be set to a positive finite value");
    CHECK(p.handbrake_torque >= p.drivetrain.max_brake_torque,
          "handbrake torque must be at least the footbrake's, or a handbrake pull cannot lock the rear");
}

/* The surface lookup must be TOTAL -- every query returns a usable surface,
 * including out of bounds -- and must not step in grip. Before this existed,
 * testground_surface_at was fully implemented, fully tested by its author,
 * and reachable from nothing in the running game. */
static void test_testground_surface_lookup_is_total_and_smooth(void) {
    Testground ground = make_flat_testground(200.0f);
    const TyreSurfaceParams *s;
    f32 prev_mu = -1.0f;
    f32 max_step = 0.0f;
    int null_count = 0;
    int z_i;

    /* 0.01 m steps across the whole world plus well past both ends. */
    for (z_i = -2000; z_i <= 25000; z_i += 1) {
        f32 z = (f32)z_i * 0.01f;
        s = testground_surface_at(&ground, 0.0f, z);
        if (!s) { null_count++; continue; }
        if (!isfinite(s->peak_mu) || s->peak_mu <= 0.0f) { null_count++; continue; }
        if (prev_mu >= 0.0f) {
            f32 step = fabsf(s->peak_mu - prev_mu);
            if (step > max_step) max_step = step;
        }
        prev_mu = s->peak_mu;
    }

#if DIRT2_INJECT_FAULT == 30
    null_count = 1;
#endif
    fprintf(stderr, "  (surface lookup: null_or_invalid=%d max_mu_step_per_cm=%.6f)\n",
            null_count, (double)max_step);
    CHECK(null_count == 0,
          "testground_surface_at must return a valid surface for every query, including out of bounds");
    /* A hard tarmac/gravel edge would be a 0.4 step in one sample. Anything
     * under 0.01 per centimetre is a blend, not a cliff. */
    CHECK(max_step < 0.01f,
          "grip must blend across a surface boundary, not change in a single sample");
}

/* Steering direction. This is the one defect in v1.0.0 that steve found on
 * hardware in under a minute and that every one of the 65 checks here missed:
 * the car steered exactly backwards. Nothing in the suite asserted WHICH WAY
 * it turned, only that it turned without exploding.
 *
 * "Right" is not asserted from a convention written in a comment somewhere --
 * it is taken from the car's own geometry, the side the front-right wheel is
 * mounted on. If those mounts are ever swapped, the first CHECK reports it
 * rather than letting the test quietly redefine correct along with the bug. */
static void test_vehicle_steers_toward_the_side_the_input_asks_for(void) {
    Testground ground = make_flat_testground(400.0f);
    VehicleParams params = make_test_vehicle_params();
    const f32 half_wheelbase = 1.275f;
    const f32 half_track = 0.800f;
    const int settle_steps = (int)(2.0f * PHYSICS_HZ);
    const int turn_steps = (int)(2.0f * PHYSICS_HZ);
    f32 dot_right = 0.0f, dot_left = 0.0f;
    int arm;

    /* make_test_suspension_config leaves every mount at the body origin, which
     * is fine for a settle test and useless for a steering one: a car with no
     * track and no wheelbase has no geometry to turn about. */
    params.suspension[WHEEL_FL].mount_point_body = vec3_make(+half_wheelbase, 0.0f, -half_track);
    params.suspension[WHEEL_FR].mount_point_body = vec3_make(+half_wheelbase, 0.0f, +half_track);
    params.suspension[WHEEL_RL].mount_point_body = vec3_make(-half_wheelbase, 0.0f, -half_track);
    params.suspension[WHEEL_RR].mount_point_body = vec3_make(-half_wheelbase, 0.0f, +half_track);

    CHECK(params.suspension[WHEEL_FR].mount_point_body.z >
          params.suspension[WHEEL_FL].mount_point_body.z,
          "test setup: the front-right wheel must sit at a larger z than the front-left");

    /* Two arms, full lock each way. One arm alone passes just as happily on a
     * car that cannot steer at all, because it only has to clear a threshold
     * in one direction. */
    for (arm = 0; arm < 2; arm++) {
        const f32 steer_input = (arm == 0) ? +1.0f : -1.0f;  /* +1 == full RIGHT, input.h */
        Vehicle car;
        InputState input;
        Vec3 right_before, forward_after;
        f32 toward_right;
        int i;

        vehicle_init(&car, &params, testground_height_query, &ground);
        car.chassis.position = vec3_make(0.0f, params.suspension[WHEEL_FL].rest_length + 0.05f, 5.0f);
        /* The car must be POINTED along the way it is moving. vehicle_init
         * leaves the orientation identity, i.e. body +X (forward) along world
         * +X, so handing it a velocity down +Z would launch it sideways at
         * 12 m/s -- a 90 degree slip angle, both front tyres saturated, and a
         * car that slides instead of turning. The first draft of this test did
         * exactly that and measured a dot of 0.003 either way.
         *
         * -90 degrees about +Y is the same yaw place_car_at_start uses to face
         * world +Z, and it has to be +Z here because the flat test ground runs
         * along +Z and is only 25 m wide in X. */
        car.chassis.orientation =
            quat_from_axis_angle(vec3_make(0.0f, 1.0f, 0.0f), -1.5707963f);
        /* Rolling, not stationary: a parked wheel has no slip angle and so
         * generates no lateral force to turn with, whatever the steer angle. */
        car.chassis.linear_velocity = vec3_make(0.0f, 0.0f, 12.0f);
        input_init(&input);

        for (i = 0; i < settle_steps; i++) vehicle_step(&car, &input, PHYSICS_DT);

        right_before = vec3_rotate_by_quat(vec3_make(0.0f, 0.0f, 1.0f), car.chassis.orientation);
        input.steer = steer_input;
        for (i = 0; i < turn_steps; i++) vehicle_step(&car, &input, PHYSICS_DT);

        forward_after = vec3_rotate_by_quat(vec3_make(1.0f, 0.0f, 0.0f), car.chassis.orientation);
        toward_right = vec3_dot(forward_after, right_before);
#if DIRT2_INJECT_FAULT == 31
        toward_right = -toward_right; /* deliberately wrong -- proves both CHECKs can fail */
#endif
        if (arm == 0) dot_right = toward_right; else dot_left = toward_right;
    }

    fprintf(stderr, "  (steer direction: full-right dot=%+.4f, full-left dot=%+.4f)\n",
            dot_right, dot_left);
    /* 0.05 rather than 0.0: a threshold of exactly zero would pass on a car
     * that barely twitches, and the failure being guarded against here was a
     * full-magnitude inversion (measured at -0.925 before the fix). */
    CHECK(dot_right > 0.05f,
          "full RIGHT steering input must rotate the car toward its own right-hand side");
    CHECK(dot_left < -0.05f,
          "full LEFT steering input must rotate the car toward its own left-hand side");
}

/*===================================================================================*/

int run_physics_tests(void) {
    test_tyre_lut_survives_nonfinite_rho();
    test_vehicle_params_are_fully_initialised();
    test_testground_surface_lookup_is_total_and_smooth();

    test_timestep_rate_is_compiled_in();
    test_timestep_init_zeroes_state();

    test_rigidbody_freefall_matches_analytic();
    test_rigidbody_offcentre_impulse_angular_velocity();
    test_rigidbody_quaternion_stays_unit_over_10000_steps();
    test_rigidbody_asymmetric_inertia_spin_bounded();

    test_suspension_settled_ride_height_matches_mg_over_k();
    test_suspension_damped_frequency_matches_sqrt_k_over_m();
    test_suspension_normal_load_never_negative();
    test_suspension_airborne_reports_zero_load();
    test_suspension_bottom_out_is_progressive();
    test_suspension_stable_across_all_physics_rates();

    test_tyre_peak_force_matches_mu_times_fz();
    test_tyre_combined_slip_stays_inside_friction_ellipse();
    test_tyre_force_scales_with_fz_pow_0_8();
    test_tyre_zero_slip_gives_zero_force_no_nan();

    test_drivetrain_rpm_tracks_wheel_speed_through_ratio();
    test_drivetrain_brake_torque_scales_with_brake_input();
    test_drivetrain_brake_decelerates_wheel_to_zero_without_reversing();

    test_vehicle_settles_on_flat_ground_and_stays();
    test_vehicle_sum_of_normal_loads_equals_weight_at_rest();
    test_vehicle_kinetic_energy_does_not_grow_with_zero_input();
    test_vehicle_stable_crossing_washboard_zone();
    test_vehicle_steers_toward_the_side_the_input_asks_for();

    fprintf(stdout, "physics tests: %d checks, %d failures (PHYSICS_HZ=%d, DIRT2_INJECT_FAULT=%d)\n",
            g_checks, g_failures, PHYSICS_HZ, DIRT2_INJECT_FAULT);
    return g_failures;
}
