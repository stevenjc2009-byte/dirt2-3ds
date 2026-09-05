/*---------------------------------------------------------------------------------
 * main.c -- entry point and the top-level frame loop.
 *
 * This is the ONE place that ties every module together, in the order the
 * rest of the skeleton documents:
 *   1. input_update()      (input/input.h)          -- read this step's input
 *   2. timestep_advance()  (core/timestep.h)         -- how many physics
 *      steps are owed this render frame
 *   3. vehicle_step() * N  (vehicle/vehicle.h)        -- once per owed step,
 *      each with dt == PHYSICS_DT, following vehicle.h's 8-step canonical
 *      order internally
 *   4. camera_update() / renderer_frame_begin/end / debugdraw_*
 *      (render/ headers)                              -- draw the
 *      INTERPOLATED pose (timestep_get_alpha) once per render frame,
 *      regardless of how many physics steps just ran
 *
 * Every module this file calls is currently a stub (see each .c file's own
 * header comment) -- this loop is real wiring, not a stub itself, so the
 * seven parallel implementation agents have one obviously-correct place to
 * see how their module is meant to be driven.
 *---------------------------------------------------------------------------------*/
#include "core/types.h"
#include "core/vecmath.h"
#include "core/timestep.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicle_params.h"
#include "vehicle/tyre_lut.h"   /* g_tyre_surface_params -- see the surface
                                  * block in make_placeholder_testground_config */
#include "input/input.h"
#include "render/renderer.h"
#include "render/camera.h"
#include "render/debugdraw.h"
#include "world/testground.h"

#ifdef __3DS__
#include <3ds.h>
#endif

/* Phase 1 test-car tuning lives here until vehicle_params.h grows a real
 * loader -- see that header's comment on why it ships no defaults itself.
 * Deliberately conservative placeholder numbers, not tuned/verified yet;
 * the point of this skeleton is that the shape compiles and links, not that
 * these numbers drive well. */
static void make_placeholder_vehicle_params(VehicleParams *params) {
    int i;
    params->chassis_mass = 1200.0f;
    params->chassis_inertia_diag = vec3_make(600.0f, 900.0f, 1500.0f);

    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        params->suspension[i].rest_length = 0.30f;
        params->suspension[i].max_travel = 0.15f;
        params->suspension[i].spring_rate = 35000.0f;
        params->suspension[i].damper_compression = 3000.0f;
        params->suspension[i].damper_rebound = 4500.0f;
        params->suspension[i].bottom_out_spring_rate = 200000.0f;
        params->suspension[i].mount_point_body = vec3_zero();
    }

    params->antiroll_rate[AXLE_FRONT] = 15000.0f;
    params->antiroll_rate[AXLE_REAR] = 10000.0f;

    params->autolevel_strength = 400.0f;
    params->autolevel_damping = 60.0f;

    params->default_tyre_surface.peak_slip_ratio = 0.15f;
    params->default_tyre_surface.peak_slip_angle = 0.12f;
    params->default_tyre_surface.peak_mu = 1.1f;
    params->default_tyre_surface.sliding_mu = 0.8f;

    params->wheel_mass_moment_of_inertia = 1.2f;

    params->drivetrain.layout = DRIVE_AWD;
    params->drivetrain.gear_ratio = 3.5f;
    params->drivetrain.final_drive_ratio = 4.1f;
    params->drivetrain.max_engine_torque = 300.0f;
    params->drivetrain.peak_torque_rpm = 4500.0f;
    params->drivetrain.max_rpm = 7500.0f;
    params->drivetrain.idle_rpm = 900.0f;
    params->drivetrain.max_brake_torque = 1800.0f;
    params->drivetrain.wheel_radius = 0.30f;

    /* These three MUST be set. vehicle.c used to hold them as local #defines
     * and now reads them from here; leaving any of them as uninitialised
     * stack is not a wrong-feeling car, it is a crash. A garbage
     * slip_speed_floor makes the slip-ratio denominator NaN, which makes rho
     * NaN, which indexes the tyre lookup table out of bounds -- caught by
     * AddressSanitizer as a SEGV in tyre_lut_lookup during integration. */
    params->max_steer_angle  = 0.5236f;   /* ~30 deg of front steer at full lock */
    params->slip_speed_floor = 1.0f;      /* m/s; floors the slip denominator so
                                            * it is not singular at rest        */
    params->handbrake_torque = 2600.0f;   /* N*m; deliberately above
                                            * max_brake_torque so a pull always
                                            * locks the rear                    */
}

static void make_placeholder_testground_config(TestgroundConfig *config) {
    config->world_half_width = 25.0f;
    config->base_height = 0.0f;
    config->flat_length = 40.0f;
    config->hills_length = 60.0f;
    config->hills_amplitude = 0.6f;
    config->hills_wavelength = 20.0f;
    config->washboard_length = 40.0f;
    config->washboard_amplitude = 0.05f;
    config->washboard_wavelength = 1.2f;

    /* Surface grip per zone. These MUST be set: TestgroundConfig is a plain
     * stack struct at the call site, so any field left out here is
     * uninitialised stack, and testground_surface_at would hand the tyre
     * model garbage mu values -- a bug the assertion suite cannot catch,
     * because nothing in it drives the surface lookup.
     *
     * Values come from the project's own shipped tyre table
     * (g_tyre_surface_params in vehicle/tyre_lut.c) rather than being
     * invented here, so the test ground and the LUT cannot drift apart:
     * tarmac_dry is {mu 1.00, sr_peak 0.20, alpha_peak 8 deg} and gravel is
     * {mu 0.60, sr_peak 0.25, alpha_peak 15 deg}. Note alpha_peak is DEGREES
     * in the table and RADIANS in TyreSurfaceParams (tyre.h) -- converted
     * below, not copied across. */
    {
        const TyreLutSurfaceParams *tarmac =
            &g_tyre_surface_params[TYRE_SURFACE_TARMAC_DRY];
        const TyreLutSurfaceParams *gravel =
            &g_tyre_surface_params[TYRE_SURFACE_GRAVEL];
        const f32 deg_to_rad = 3.14159265f / 180.0f;

        config->surface_tarmac.peak_mu         = tarmac->mu;
        config->surface_tarmac.peak_slip_ratio = tarmac->sr_peak;
        config->surface_tarmac.peak_slip_angle = tarmac->alpha_peak_deg * deg_to_rad;
        /* Tarmac falls away harder past the peak than gravel does -- once a
         * tarmac tyre lets go it loses a quarter of its grip, which is what
         * makes a tarmac slide something you have to catch. */
        config->surface_tarmac.sliding_mu      = tarmac->mu * 0.75f;

        config->surface_gravel.peak_mu         = gravel->mu;
        config->surface_gravel.peak_slip_ratio = gravel->sr_peak;
        config->surface_gravel.peak_slip_angle = gravel->alpha_peak_deg * deg_to_rad;
        /* Gravel holds much more of its grip while sliding (0.85 vs 0.75).
         * That small gap is the whole rally feel: the car goes sideways
         * early and stays controllable there, instead of snapping away. */
        config->surface_gravel.sliding_mu      = gravel->mu * 0.85f;

        /* 6 m of blend across the flat/hills seam. A hard edge would change
         * grip by 0.4 mu inside a single physics step, which reads as the
         * car being yanked rather than as a surface change. */
        config->surface_transition_length = 6.0f;
    }
}

/* Adapter: vehicle.h's VehicleSurfaceQuery -> world/testground.h's lookup.
 * This thin function is the ONLY thing coupling the vehicle to the world;
 * vehicle/ deliberately does not include world/ (see vehicle.h). Without it
 * being installed on the Vehicle below, testground_surface_at is never called
 * by anything and tarmac and gravel feel identical. */
static const TyreSurfaceParams *main_surface_query(void *userdata,
                                                    f32 world_x, f32 world_z) {
    return testground_surface_at((const Testground *)userdata, world_x, world_z);
}

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    Testground testground;
    TestgroundConfig ground_config;
    VehicleParams params;
    Vehicle car;
    InputState input;
    InputConfig input_config;
    CameraConfig camera_config;
    Camera camera;
    Timestep timestep;
    Vec3 wheel_offsets[VEHICLE_WHEEL_COUNT];
    int i;
    bool running = true;

    make_placeholder_testground_config(&ground_config);
    testground_generate(&testground, &ground_config);

    make_placeholder_vehicle_params(&params);
    vehicle_init(&car, &params, testground_height_query, &testground);
    /* vehicle_init leaves this NULL on purpose (vehicle.h). Setting it is
     * what makes the ground's tarmac/gravel zones actually reach the tyre
     * model -- leave it out and the whole surface system is dead code that
     * still compiles and still passes its own tests. */
    car.surface_query = main_surface_query;
    car.chassis.position = vec3_make(0.0f, 1.0f, 0.0f);

    /* Body-space wheel positions for the renderer, taken from the SAME
     * suspension mount points the physics raycasts from -- not re-typed as
     * literals here. If the two ever disagree, the car is drawn with its
     * wheels somewhere the simulation says they are not, and every
     * suspension bug becomes impossible to read off the screen. */
    for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
        wheel_offsets[i] = params.suspension[i].mount_point_body;
    }

    input_config.circlepad_deadzone = 0.15f;
    input_config.circlepad_radius = 156.0f;
    input_config.throttle_ramp_rate = 2.0f;
    input_config.throttle_release_rate = 3.0f;
    input_config.brake_ramp_rate = 4.0f;
    input_config.brake_release_rate = 3.0f;
    /* These two were the landmine. input.c raises the stick fraction to
     * steer_response_exponent, and fraction^0 == 1.0 -- so leaving this field
     * unset gave INSTANT FULL STEERING LOCK from the smallest deflection.
     * input.c now clamps a garbage value to a sane default, but setting it
     * here explicitly is what makes the intent visible: 1.6 gives fine
     * control near centre and full lock at the rim, which is what a Circle
     * Pad needs. steer_return_rate rate-limits the return to centre on
     * release (steering IN, and countersteer, stay instant). */
    input_config.steer_response_exponent = 1.6f;
    input_config.steer_return_rate = 8.0f;
    input_init(&input);

    camera_config.follow_distance = 6.0f;
    camera_config.follow_height = 2.5f;
    camera_config.look_ahead_height = 1.0f;
    camera_config.position_smoothing = 0.15f;
    camera_config.fov_degrees = 55.0f;
    camera_config.near_clip = 0.1f;
    camera_config.far_clip = 500.0f;
    camera_init(&camera, &camera_config);

    timestep_init(&timestep);

#ifdef __3DS__
    gfxInitDefault();
#endif
    renderer_init();
    debugdraw_init();

#ifdef __3DS__
    /* Wall-clock frame delta source: svcGetSystemTick() is the ARM11 cycle
     * counter, SYSCLOCK_ARM11 (3ds.h) its frequency in Hz -- the standard
     * devkitPro idiom for a frame delta with no dependency on a separate
     * timer service. Seeded once here so the very first iteration's delta
     * is "time since boot-of-this-loop", not since console power-on. */
    u64 last_tick = svcGetSystemTick();
#endif

    while (running) {
        f32 frame_dt;
        uint32_t steps, i;

#ifdef __3DS__
        hidScanInput();
        running = !(hidKeysHeld() & KEY_START);

        {
            u64 now_tick = svcGetSystemTick();
            frame_dt = (f32)(now_tick - last_tick) / (f32)SYSCLOCK_ARM11;
            last_tick = now_tick;
        }
#else
        running = false; /* host/non-3DS builds: run one pass and exit --
            there is no real frame loop off-console, and this file is not
            compiled into the host test build anyway (see Makefile.host). */
        frame_dt = PHYSICS_DT; /* no wall clock off-console; one deterministic
            step so this path still compiles even though it never runs. */
#endif

        /* Spiral-of-death guard: clamp the incoming frame delta itself
         * before it ever reaches the accumulator, so a long stall (a
         * debugger breakpoint, a slow one-off romfs read) cannot bank
         * enough time to demand an unbounded number of catch-up steps.
         * timestep_advance (core/timestep.c) already clamps internally to
         * MAX_STEPS_PER_FRAME * PHYSICS_DT (8 steps at 120 Hz, ~0.067s) --
         * a tighter bound than this -- so this 0.25s clamp is a second,
         * independent line of defense at the call site, not a substitute
         * for timestep.c's own cap. Negative frame_dt (should not happen
         * off a monotonic tick counter, but floored defensively) is clamped
         * to zero rather than draining the accumulator backwards. */
        if (frame_dt > 0.25f) {
            frame_dt = 0.25f;
        } else if (frame_dt < 0.0f) {
            frame_dt = 0.0f;
        }

        steps = timestep_advance(&timestep, frame_dt);
        for (i = 0; i < steps; i++) {
            /* input.h's contract: input_update runs once per physics step,
             * immediately before that step's vehicle_step, with
             * dt == PHYSICS_DT -- see input.h's input_update comment on why
             * ramp rates need physics-step cadence, not render-frame
             * cadence (a multi-step catch-up frame must ramp throttle/brake
             * once per step, not once for the whole frame). */
            input_update(&input, &input_config, PHYSICS_DT);
            vehicle_step(&car, &input, PHYSICS_DT);
        }

        camera_update(&camera, &camera_config, car.chassis.position,
                      car.chassis.orientation, frame_dt);

        renderer_frame_begin(&camera);
        debugdraw_frame_begin();

        /* Solid geometry first: the ground patch around the camera, then the
         * car. Both cull back faces and restore CULL_NONE on the way out, so
         * debugdraw's ribbon quads below still draw from either side. */
        renderer_draw_ground(&testground, camera.position);
        renderer_draw_vehicle(car.chassis.position, car.chassis.orientation,
                               wheel_offsets);

        /* debugdraw on top -- contact points and normals. Not decoration:
         * these are how a wrong suspension or a wheel floating off the
         * surface is spotted on hardware, where there is no debugger. */
        for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
            const SuspensionState *wheel_state = &car.wheels[i].suspension;
            if (wheel_state->grounded) {
                debugdraw_point(wheel_state->contact_point, 0.08f, 0x00FF00FF);
                debugdraw_line(wheel_state->contact_point,
                               vec3_add(wheel_state->contact_point,
                                        vec3_scale(wheel_state->contact_normal, 0.5f)),
                               0xFFFF00FF);
            }
        }

        debugdraw_text(4.0f, 4.0f, 0xFFFFFFFF, "dt %.1fms steps %u alpha %.2f",
                       (double)(frame_dt * 1000.0f), steps,
                       (double)timestep_get_alpha(&timestep));
        debugdraw_text(4.0f, 16.0f, 0xFFFFFFFF, "speed %.1f m/s",
                       (double)vec3_length(car.chassis.linear_velocity));

        debugdraw_frame_end();
        renderer_frame_end();
    }

    debugdraw_shutdown();
    renderer_shutdown();
#ifdef __3DS__
    gfxExit();
#endif

    return 0;
}
