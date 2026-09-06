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
#include "render/hud.h"
#include "race/track.h"
#include "race/lap.h"
#include "race/barrier.h"
#include "ui/pausemenu.h"
#include "world/testground.h"

#include <math.h>   /* atan2f, for deriving the start yaw from the track */

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
    }

    /* Wheel geometry. This USED to be vec3_zero() for all four, which put
     * every suspension raycast at the chassis origin: no wheelbase, no track,
     * so the car had no pitch or roll response at all (it pogoed on a single
     * point) and the renderer drew all four wheel boxes stacked inside the
     * body. That is not a tuning choice, it is a car with no wheels in the
     * places its wheels are.
     *
     * Body axes are x forward, y up, z right (vehicle.c:115). The numbers are
     * the 2.55 m wheelbase / 1.6 m track already recommended in
     * vehicle_params.h's mount_point_body note: +-1.275 fore/aft, +-0.8
     * left/right.
     *
     * y = -0.25 is this file's call, since vehicle_params.h leaves the height
     * to "whoever places the chassis mesh's origin" and that is here. Static
     * spring compression is (mass/4)*g/k = 300*9.81/35000 = 0.084 m, so the
     * settled chassis origin ends up at 0.25 + (0.30 - 0.084) = 0.466 m above
     * the ground -- a plausible centre-of-mass height that also keeps the
     * renderer's chassis box clear of the surface. With y = 0 the origin would
     * settle at 0.216 m and the body box would be drawn half-buried. */
    params->suspension[WHEEL_FL].mount_point_body = vec3_make( 1.275f, -0.25f, -0.8f);
    params->suspension[WHEEL_FR].mount_point_body = vec3_make( 1.275f, -0.25f,  0.8f);
    params->suspension[WHEEL_RL].mount_point_body = vec3_make(-1.275f, -0.25f, -0.8f);
    params->suspension[WHEEL_RR].mount_point_body = vec3_make(-1.275f, -0.25f,  0.8f);

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
    /* GROWN for v1.0.2 to hold the enlarged oval, and it is a matched pair
     * with it: race/track.c's track_build_example_oval now reaches X = +-29,
     * and race/barrier.c's wall sits a further 4 m out beyond the gravel
     * run-off, so the drivable world reaches X = +-33 and Z = 5..167. A 25 m
     * half-width world would have both the ribbon and the wall running off the
     * sides. See track_build_example_oval's comment for the full margin
     * arithmetic. This does not fail loudly if the two drift apart -- the
     * track just leaves the world and testground_height_query starts returning
     * false under a car that is still visually on the road. */
    config->world_half_width = 38.0f;
    config->base_height = 0.0f;
    config->flat_length = 45.0f;
    config->hills_length = 70.0f;
    config->hills_amplitude = 0.6f;
    config->hills_wavelength = 20.0f;
    config->washboard_length = 60.0f;
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

/* Everything the surface lookup below needs, in one struct, because
 * vehicle.h's VehicleSurfaceQuery passes exactly one void* and the answer now
 * depends on BOTH the world and the circuit. */
typedef struct SurfaceContext {
    /* Non-const because suspension.h's SuspensionGroundQuery takes a plain
     * void*, and casting the const away at every call is a worse smell than
     * simply not claiming a constness the callback signature cannot carry.
     * Nothing here writes through it. */
    Testground *ground;
    const Track *track;

    /* track_query's search hint. Deliberately its OWN cache and not shared
     * with lap_update's or the HUD's: each caller queries at a different
     * cadence and from a different position, and two callers sharing one hint
     * hand each other the wrong starting guess. It is only a hint --
     * track_query falls back to a full scan -- so sharing would be slower and
     * confusing rather than wrong, which is exactly the kind of bug that never
     * gets found. */
    int cached_segment;

    /* Copies, not pointers into the TestgroundConfig, which is a caller stack
     * local that does not outlive main()'s setup block. */
    TyreSurfaceParams on_track;
    TyreSurfaceParams off_track;
} SurfaceContext;

/* Adapter: suspension.h's SuspensionGroundQuery -> the testground inside the
 * context. This indirection exists for one reason: vehicle.h gives the ground
 * query and the surface query a SINGLE shared userdata pointer ("both queries
 * answer about the same world"), and as of v1.0.2 the surface answer needs the
 * Track as well as the heightfield. So the shared pointer became the
 * SurfaceContext, and the height query unwraps it.
 *
 * The alternative was to add a second userdata field to Vehicle. Rejected:
 * vehicle.h's one-world-one-pointer contract is correct, and it is still true
 * here -- SurfaceContext simply IS this project's description of that one
 * world. Widening a shared header to avoid writing a four-line adapter in the
 * one file that owns the coupling is the wrong trade. */
static bool main_height_query(void *userdata, f32 world_x, f32 world_z,
                               f32 *out_height, Vec3 *out_normal) {
    SurfaceContext *ctx = (SurfaceContext *)userdata;
    return testground_height_query(ctx->ground, world_x, world_z,
                                    out_height, out_normal);
}

/* Adapter: vehicle.h's VehicleSurfaceQuery -> what the tyre is actually
 * standing on. This thin function is the ONLY thing coupling the vehicle to
 * the world; vehicle/ deliberately does not include world/ (see vehicle.h).
 * Without it being installed on the Vehicle below, the whole surface system is
 * dead code that still compiles and still passes its own tests.
 *
 * v1.0.2: THE RIBBON IS NOW ITS OWN SURFACE. Before this, grip came only from
 * testground.c's three height zones, which meant the racing line and the dirt
 * beside it had identical grip and cutting a corner was free -- there was no
 * such thing as "off the track", only "off the world". Now the circuit decides:
 * inside the ribbon is tarmac, everything else is gravel.
 *
 * That deliberately OVERRIDES testground's own per-zone surfaces rather than
 * blending with them. The zones exist to make suspension behaviour testable
 * (flat / hills / washboard), and they were never a statement about where the
 * road is. Keeping them would have left the flat zone with tarmac grip on both
 * sides of the white line, so a third of the lap would still have had no
 * penalty for cutting -- a rule that applies in two thirds of the cases is
 * worse than none, because it teaches the driver the wrong thing.
 *
 * Cost: four calls per physics step, 480/s. track_query is a windowed search
 * around the cached hint, and all four wheels are within two metres of each
 * other, so the hint is warm on every call after the first. */
static const TyreSurfaceParams *main_surface_query(void *userdata,
                                                    f32 world_x, f32 world_z) {
    SurfaceContext *ctx = (SurfaceContext *)userdata;
    TrackQueryResult q;
    f32 offset;

    track_query(ctx->track, vec3_make(world_x, 0.0f, world_z),
                &ctx->cached_segment, &q);

    offset = q.lateral_offset < 0.0f ? -q.lateral_offset : q.lateral_offset;
    return (offset <= q.half_width) ? &ctx->on_track : &ctx->off_track;
}

/* Where the car starts, and which way it points. A separate function rather
 * than two lines inline in main() so a host probe can call the SHIPPED start
 * pose instead of re-typing it: a probe that hard-codes these numbers keeps
 * passing while this file drifts, which is exactly the failure it exists to
 * catch. Both things it sets are load-bearing:
 *
 * POSITION. testground.h places its three zones along +Z starting at Z=0
 * (flat 0..40, hills 40..100, washboard 100..140) and bounds the world to
 * X in [-25, +25]. testground_query returns false outside that, which the
 * vehicle reads as "no ground" -- so a car that leaves the box falls forever.
 * Z = 5 starts it 5 m into the flat zone with the whole 140 m run ahead.
 * Y = 1.0 is above the settled ride height (0.466 m, see the mount-point
 * comment above), so the car drops ~0.5 m and settles on its springs at boot:
 * a free, obvious check that gravity and the suspension are both alive.
 *
 * ORIENTATION. The car's body forward is +X (vehicle.c:115) but the test
 * ground runs along +Z, so an unrotated car drives into the world's 25 m side
 * wall in a couple of seconds and never sees the hills at all. A -90 degree
 * yaw about +Y maps body +X onto world +Z -- the sign was confirmed by
 * running quat_from_axis_angle/vec3_rotate_by_quat, not reasoned about, since
 * it depends on this project's own handedness:
 *     yaw=+90 -> forward=(0, 0, -1)
 *     yaw=-90 -> forward=(0, 0, +1)   <- this one                          */
/* Extracted from main() for the same reason place_car_at_start was: a host
 * probe that hand-copies these numbers passes happily while main.c drifts.
 * The probe for the inverted-steering fix drives through input_update_raw,
 * so it needs the SHIPPED dead zone and response curve, not a lookalike. */
static void make_placeholder_input_config(InputConfig *config) {
    config->circlepad_deadzone = 0.15f;
    config->circlepad_radius = 156.0f;
    config->throttle_ramp_rate = 2.0f;
    config->throttle_release_rate = 3.0f;
    config->brake_ramp_rate = 4.0f;
    config->brake_release_rate = 3.0f;
    /* These two were the landmine. input.c raises the stick fraction to
     * steer_response_exponent, and fraction^0 == 1.0 -- so leaving this field
     * unset gave INSTANT FULL STEERING LOCK from the smallest deflection.
     * input.c now clamps a garbage value to a sane default, but setting it
     * here explicitly is what makes the intent visible: 1.6 gives fine
     * control near centre and full lock at the rim, which is what a Circle
     * Pad needs. steer_return_rate rate-limits the return to centre on
     * release (steering IN, and countersteer, stay instant). */
    config->steer_response_exponent = 1.6f;
    config->steer_return_rate = 8.0f;
}

static void place_car_at_start(Vehicle *car, const Track *track) {
    /* ON THE GRID, not next to it, and DERIVED FROM THE TRACK rather than
     * typed out. v0.2 gave the world a closed circuit and the car has to start
     * on waypoint 0 of it or no lap can ever be counted -- lap.c only counts a
     * forward wrap-crossing of the start line, and a car parked outside the
     * loop never crosses anything.
     *
     * This USED to be the literal `(-12, 1, 22)`, which was waypoint 0 of the
     * old oval. That is precisely the bug this version exists to stop
     * repeating: v1.0.2 enlarged the circuit, waypoint 0 moved to
     * (-24, 38), and a hardcoded start pose would have put the car 12 m off
     * the road facing nothing in particular, with no error and no warning --
     * just a lap counter that never moved. The pose now cannot drift from the
     * track because it is read out of the track.
     *
     * THE YAW IS COMPUTED, NOT ASSUMED. Body forward is +X (vehicle.c:115),
     * and a rotation of theta about +Y maps it to (cos theta, 0, -sin theta):
     *     theta =   0 -> forward = (+1, 0,  0)
     *     theta = +90 -> forward = ( 0, 0, -1)
     *     theta = -90 -> forward = ( 0, 0, +1)
     * So to face along a tangent (tx, tz) we need cos theta = tx and
     * sin theta = -tz, i.e. theta = atan2(-tz, tx). For this oval waypoint 0
     * -> 1 runs due +Z, giving atan2(-1, 0) = -90 degrees -- the same value
     * that was hardcoded before, which is the arithmetic agreeing with the
     * measurement rather than replacing it. Facing the other way would make
     * every lap a reverse crossing, which lap.c correctly refuses to count.
     *
     * Y = 1.0 is deliberately above the settled ride height (0.466 m) so the
     * car drops onto its springs at boot rather than starting interpenetrating
     * the ground: a free, obvious check that gravity and the suspension are
     * both alive. */
    Vec3 start = track->waypoints[0].center;
    Vec3 next = track->waypoints[1 % track->count].center;
    f32 tx = next.x - start.x;
    f32 tz = next.z - start.z;

    car->chassis.position = vec3_make(start.x, 1.0f, start.z);
    car->chassis.orientation =
        quat_from_axis_angle(vec3_make(0.0f, 1.0f, 0.0f), atan2f(-tz, tx));
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
    bool running = true;

    /* SELECT opens this. While it is open the simulation is frozen and the
     * bottom screen belongs to the menu instead of the HUD -- see the wiring
     * in the frame loop below and the call-order note in ui/pausemenu.h. */
    PauseMenu pause_menu;

    /* The circuit and the lap timer. Both live here for the whole run: the
     * Track is immutable once built, and LapState borrows a pointer to it
     * (lap.h says borrowed, not owned) so the Track must outlive it -- same
     * scope, declared in that order, is the simplest way to guarantee that.
     *
     * hud_query_segment and barrier_segment are track_query search caches,
     * each private to one caller: the HUD queries once per FRAME, the barrier
     * once per physics STEP, and lap_update and SurfaceContext keep their own
     * on top of that. Four callers, four caches, for the reason given on
     * SurfaceContext.cached_segment -- a shared cache would be thrashed
     * between callers sampling at different cadences and different positions,
     * turning track_query's O(1) hint into a full O(n) rescan every time. */
    Track track;
    LapState lap_state;
    SurfaceContext surface_ctx;
    int hud_query_segment = TRACK_UNKNOWN_SEGMENT;
    int barrier_segment = TRACK_UNKNOWN_SEGMENT;

    make_placeholder_testground_config(&ground_config);
    testground_generate(&testground, &ground_config);

    /* THE TRACK IS BUILT FIRST, before the car exists. Two things now read
     * their setup out of it -- where the car starts, and which surface each
     * tyre is on -- so building it later would mean deriving both from an
     * uninitialised Track. It used to be built just above the frame loop,
     * which was fine when nothing depended on it. */
    track_build_example_oval(&track);
    lap_init(&lap_state, &track);

    /* Filled BEFORE vehicle_init, because it is the userdata both of the
     * vehicle's world callbacks are about to be handed. The two surfaces come
     * from the ground config rather than being invented here, so the numbers
     * the tyre model sees stay tied to the ones the project's shipped tyre
     * table defines. */
    surface_ctx.ground = &testground;
    surface_ctx.track = &track;
    surface_ctx.cached_segment = TRACK_UNKNOWN_SEGMENT;
    surface_ctx.on_track = ground_config.surface_tarmac;
    surface_ctx.off_track = ground_config.surface_gravel;

    make_placeholder_vehicle_params(&params);
    vehicle_init(&car, &params, main_height_query, &surface_ctx);

    /* vehicle_init leaves surface_query NULL on purpose (vehicle.h). Setting
     * it is what makes the tarmac/gravel distinction actually reach the tyre
     * model -- leave it out and the whole surface system is dead code that
     * still compiles and still passes its own tests. It reuses
     * ground_userdata, which is why that had to be the SurfaceContext. */
    car.surface_query = main_surface_query;

    place_car_at_start(&car, &track);

    make_placeholder_input_config(&input_config);
    input_init(&input);

    camera_config.follow_distance = 6.0f;
    camera_config.follow_height = 2.5f;
    camera_config.look_ahead_height = 1.0f;
    /* This is a PER-SECOND blend factor (camera.h:41), not a per-frame one:
     * camera.c turns it into t = 1 - (1 - s)^dt, so the exponential time
     * constant is -1/ln(1 - s). An exponential follow lags a car moving at
     * constant speed by exactly speed * tau, forever -- it never catches up.
     *
     * The 0.15 that used to be here reads like a per-frame lerp factor and is
     * a catastrophe as a per-second one: tau = -1/ln(0.85) = 6.15 s, i.e. 98 m
     * behind at 16 m/s and 215 m behind at 35 m/s. Measured in Azahar, both:
     * the car left the top screen entirely and was a red dot against the sky.
     *
     * 0.98 gives tau = -1/ln(0.02) = 0.256 s, so the trailing distance is
     * 6 m + speed * 0.256 -- about 10 m at 16 m/s and 15 m at 35 m/s. The
     * camera easing back as the car gains speed is the behaviour a chase cam
     * wants anyway; it just has to be metres, not hundreds of metres. */
    camera_config.position_smoothing = 0.98f;
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
    /* After renderer_init on purpose -- hud.c creates a render target and a
     * citro2d text buffer, and renderer_init is what calls C3D_Init/C2D_Init.
     * See hud.h. */
    hud_init();
    /* Also after renderer_init, and for the same reason: pausemenu.c allocates
     * its own citro2d text buffers. It draws to hud.c's bottom-screen target
     * rather than creating a second one, so it must also come after hud_init.
     * See pausemenu.h's call-order note. */
    pausemenu_init(&pause_menu);

#ifdef __3DS__
    /* Wall-clock frame delta source: svcGetSystemTick() is the ARM11 cycle
     * counter, SYSCLOCK_ARM11 (3ds.h) its frequency in Hz -- the standard
     * devkitPro idiom for a frame delta with no dependency on a separate
     * timer service. Seeded once here so the very first iteration's delta
     * is "time since boot-of-this-loop", not since console power-on. */
    u64 last_tick = svcGetSystemTick();
#endif

    /* The pose the car had one physics step ago, kept ACROSS frames so a
     * render frame that owes zero physics steps (physics is 120 Hz, VBlank is
     * ~59.83 Hz, so the step count per frame is 2 most frames and 3
     * occasionally) still has something to interpolate from.
     *
     * This lives in main.c rather than as two new fields on RigidBody
     * deliberately: RigidBody has no notion of render frames, nothing in the
     * physics needs a previous pose, and rigidbody.h is a shared header that
     * three other modules and both test suites include. Keeping the render
     * concern in the render loop costs two locals and touches nobody. */
    Vec3 prev_pos = car.chassis.position;
    Quat prev_orient = car.chassis.orientation;

    /* Smoothed frame rate for the bottom-screen readout. The raw 1/frame_dt is
     * unreadable -- it flickers several fps every frame because the tick delta
     * quantises against VBlank -- so it is exponentially smoothed here rather
     * than in hud.c, which deliberately holds no state between frames (hud.h).
     * 0.05 per frame is roughly a third of a second to settle: slow enough to
     * read, fast enough that a real drop below 60 shows up while it is
     * happening rather than after it has passed. */
    f32 fps_smoothed = 60.0f;

    /* (The Track, LapState and hud_query_segment used to be declared and built
     * here. They moved to the top of main() in v1.0.2 because the car's start
     * pose and the tyre surface lookup are both derived from the track now, and
     * both are set up well before this point.) */

    while (running) {
        f32 frame_dt, alpha;
        Vec3 draw_pos;
        Quat draw_orient;
        uint32_t steps, i;
        HudStats hud_stats;

#ifdef __3DS__
        hidScanInput();
        running = !(hidKeysHeld() & KEY_START);

        /* Exactly once per rendered frame, and BEFORE the physics below --
         * pausemenu.h's contract. It reads SELECT to open, and the D-pad and
         * A/B to navigate, straight off hidKeysDown, so it deliberately does
         * NOT go through input.c: input.c models a driver's controls (ramped
         * throttle, ramped brake, steer) at physics cadence, and a menu needs
         * raw one-shot button edges at frame cadence instead. */
        pausemenu_update(&pause_menu, (uint32_t)hidKeysDown());
        if (pausemenu_take_quit_request(&pause_menu)) {
            running = false;
        }

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

        if (pausemenu_is_open(&pause_menu)) {
            /* Frozen while the menu is up. The accumulator is RESET rather
             * than merely left un-advanced, because the update check is a
             * blocking network call that can sit there for seconds: banking
             * that wall time would hand timestep_advance a huge debt the
             * moment the menu closed, and the car would fast-forward across
             * the track on the first frame after resume.
             *
             * The previous pose is dragged up to the current one so the
             * interpolation below resolves to the car's actual resting pose
             * instead of the pose one step before the freeze -- otherwise
             * pausing and resuming each nudge the drawn car by one step. */
            steps = 0;
            timestep_init(&timestep);
            prev_pos = car.chassis.position;
            prev_orient = car.chassis.orientation;
        } else {
            steps = timestep_advance(&timestep, frame_dt);
        }
        for (i = 0; i < steps; i++) {
            /* Snapshot INSIDE the loop, not before it: on a catch-up frame
             * that runs several steps, the pose to interpolate from is the
             * one before the LAST step, not the one at the top of the frame.
             * Snapshotting outside would make the car visibly rubber-band
             * every time the step count changed. */
            prev_pos = car.chassis.position;
            prev_orient = car.chassis.orientation;

            /* input.h's contract: input_update runs once per physics step,
             * immediately before that step's vehicle_step, with
             * dt == PHYSICS_DT -- see input.h's input_update comment on why
             * ramp rates need physics-step cadence, not render-frame
             * cadence (a multi-step catch-up frame must ramp throttle/brake
             * once per step, not once for the whole frame). */
            input_update(&input, &input_config, PHYSICS_DT);
            vehicle_step(&car, &input, PHYSICS_DT);

            /* The invisible wall, applied to the RESULT of the step that just
             * ran -- barrier.h's contract is explicitly "after vehicle_step",
             * because it corrects an already-integrated position rather than
             * contributing a force to the integration.
             *
             * It must run BEFORE lap_update, not after: lap_update decides
             * whether the car crossed the start line from the position it is
             * handed, and handing it a position the barrier is about to move
             * would let a crossing be credited at a place the car never
             * actually occupied.
             *
             * Its own segment cache, separate from lap_update's and the HUD's
             * -- three callers querying the track at three different cadences
             * must not share one cache (see track.h's caching contract). */
            barrier_apply(&track, &car.chassis, &barrier_segment);

            /* Once per physics STEP, not once per render frame -- lap.h's
             * contract. Lap times are accumulated dt, so calling this once
             * for a frame that owed three steps would run the clock at a
             * third speed, and the crossing test would sample the car's
             * position at render cadence and could step clean over the
             * start line at 128 km/h without noticing.
             *
             * Uses the RAW post-step position, not the interpolated draw
             * pose: the draw pose is one step in the past by design, and a
             * lap time must measure the simulation, not the picture of it. */
            lap_update(&lap_state, car.chassis.position, PHYSICS_DT);
        }

        /* The interpolated draw pose camera.h and renderer.h both ask for.
         * timestep_get_alpha is the leftover accumulator as a fraction of one
         * step, so the frame is rendered `alpha` of the way from the
         * second-to-last completed step to the last one -- one step in the
         * past, but smooth. Drawing the raw physics pose instead judders
         * whenever the step count per frame changes, which at 120 Hz physics
         * against a 59.83 Hz VBlank it periodically does.
         *
         * The SAME pose goes to the camera and to the car. Passing the raw
         * pose to one and the interpolated pose to the other would make the
         * car shimmer against its own chase camera. */
        alpha = timestep_get_alpha(&timestep);
        draw_pos = vec3_lerp(prev_pos, car.chassis.position, alpha);
        draw_orient = quat_slerp(prev_orient, car.chassis.orientation, alpha);

        camera_update(&camera, &camera_config, draw_pos, draw_orient, frame_dt);

        renderer_frame_begin(&camera);
        debugdraw_frame_begin();

        /* Solid geometry first: the ground patch around the camera, then the
         * car. Both cull back faces and restore CULL_NONE on the way out, so
         * debugdraw's ribbon quads below still draw from either side. */
        /* Body-space wheel centres, rebuilt every frame from the SAME
         * suspension state the physics just wrote -- not re-typed literals,
         * and not the static mount points either.
         *
         * The mount point is where the raycast STARTS; the wheel hangs below
         * it by the spring's current length (rest_length shortens by
         * `compression`, see suspension.h), and the wheel's centre is one
         * radius back up from where it touches. Drawing at the raw mount
         * point instead buries every wheel by the static spring compression
         * (8 cm on this car) and, worse, makes the suspension invisible: the
         * wheels would ride rigidly with the body over the washboard section
         * instead of moving in their arches, which is the one thing a
         * suspension bug shows up in. */
        for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
            const SuspensionConfig *sc = &params.suspension[i];
            f32 hang = sc->rest_length - car.wheels[i].suspension.compression
                       - params.drivetrain.wheel_radius;
            wheel_offsets[i] = vec3_sub(sc->mount_point_body,
                                        vec3_make(0.0f, hang, 0.0f));
        }

        renderer_draw_ground(&testground, camera.position);
        /* Track ribbon AFTER the terrain: it is lifted a few centimetres above
         * the queried ground height rather than being part of it, so it has to
         * be drawn over the surface it sits on. It takes the Testground as
         * well as the Track because a waypoint's Y is never authoritative --
         * see race/track.h -- so every ribbon vertex re-queries the real
         * ground height at its own XZ. */
        renderer_draw_track(&testground, &track);
        renderer_draw_vehicle(draw_pos, draw_orient, wheel_offsets);

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

        /* Lap and clock on the TOP screen as well as the bottom one, and not
         * as duplication for its own sake: this is the one readout a driver
         * needs without looking away from the road, and looking down at the
         * bottom screen mid-corner is exactly when it cannot be read. Kept to
         * one line -- the detail (last, best, progress, off-course) stays
         * downstairs where there is room for it. */
        debugdraw_text(4.0f, 28.0f, 0xFFD060FF, "lap %d   %d:%05.2f",
                       lap_state.lap_count + 1,
                       (int)(lap_state.current_lap_time / 60.0f),
                       (double)(lap_state.current_lap_time -
                                60.0f * (f32)(int)(lap_state.current_lap_time / 60.0f)));

        debugdraw_frame_end();

        /* AFTER debugdraw_frame_end, per pausemenu.h: the menu's top-screen
         * banner is citro2d text, and debugdraw_frame_end is what calls
         * C2D_Prepare. It draws nothing at all while the menu is closed. */
        pausemenu_draw_top(&pause_menu);

        /* Bottom screen LAST. hud_draw switches the active render target to
         * the bottom screen and leaves it there (hud.h), so anything drawn to
         * the top screen after this point would silently land down here. It
         * also relies on debugdraw_frame_end having already called
         * C2D_Prepare -- see the comment in hud_draw.
         *
         * The numbers come from the same variables the frame just used, not
         * from a second read of the car: a HUD that samples the simulation
         * separately from the renderer eventually disagrees with what is on
         * the top screen, and then the readout is worse than none. */
        if (frame_dt > 0.0f) {
            f32 fps_instant = 1.0f / frame_dt;
            fps_smoothed += (fps_instant - fps_smoothed) * 0.05f;
        }

        hud_stats.speed_ms = vec3_length(car.chassis.linear_velocity);
        hud_stats.engine_rpm = car.drivetrain_state.engine_rpm;
        hud_stats.max_rpm = params.drivetrain.max_rpm;
        hud_stats.idle_rpm = params.drivetrain.idle_rpm;
        /* The RAMPED values off InputState, which is what vehicle_step was
         * actually handed -- not the raw button state. A throttle bar that
         * snaps to full the instant R is held would be lying about what the
         * physics received (input.h ramps it over throttle_ramp_rate). */
        hud_stats.throttle = input.throttle;
        hud_stats.brake = input.brake;
        hud_stats.steer = input.steer;
        hud_stats.handbrake = input.handbrake;
        hud_stats.position = draw_pos;
        hud_stats.frame_ms = frame_dt * 1000.0f;
        hud_stats.fps = fps_smoothed;
        hud_stats.physics_steps = steps;

        /* Race state. The lap counter and the two stored times come straight
         * off LapState; progress and lateral offset need their own query
         * because lap_update keeps neither -- it only needs the crossing. */
        {
            TrackQueryResult q;
            f32 off;
            track_query(&track, car.chassis.position, &hud_query_segment, &q);
            off = q.lateral_offset < 0.0f ? -q.lateral_offset : q.lateral_offset;

            hud_stats.lap_count = lap_state.lap_count;
            hud_stats.current_lap_time = lap_state.current_lap_time;
            hud_stats.last_lap_time = lap_state.last_lap_time;
            hud_stats.best_lap_time = lap_state.best_lap_time;
            hud_stats.lap_progress = q.progress;
            hud_stats.lateral_offset = q.lateral_offset;
            hud_stats.track_half_width = q.half_width;
            /* The off-course VERDICT is made here, not in hud.c, so the
             * bottom screen states no policy about what counts as on-track --
             * see hud.h. Today it is the plain "within the ribbon" test
             * track.h suggests; a real rally game would want a grace period
             * and a two-wheels-off rule, and that would change here. */
            hud_stats.on_track = (off <= q.half_width);
        }

        for (i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
            const SuspensionState *ws = &car.wheels[i].suspension;
            hud_stats.wheels[i].compression = ws->compression;
            hud_stats.wheels[i].max_travel = params.suspension[i].max_travel;
            hud_stats.wheels[i].normal_load = ws->normal_load;
            hud_stats.wheels[i].grounded = ws->grounded;
        }
        /* EXACTLY ONE of these two runs, never both: they draw to the same
         * bottom-screen target (pausemenu.c asks hud.c for it via
         * hud_get_target rather than creating a second one), and each begins
         * by clearing it, so calling both would leave whichever ran second as
         * the only thing visible and waste a full clear on the other. */
        if (pausemenu_is_open(&pause_menu)) {
            pausemenu_draw_bottom(&pause_menu);
        } else {
            hud_draw(&hud_stats);
        }

        renderer_frame_end();
    }

    /* Reverse of init order: hud before renderer, because hud_shutdown deletes
     * a citro2d text buffer and a citro3d render target, and renderer_shutdown
     * is what calls C2D_Fini/C3D_Fini.
     *
     * pausemenu before hud for the same reason one level down: it borrows
     * hud.c's render target, so it must let go of its own citro2d resources
     * while that target is still alive. */
    pausemenu_shutdown(&pause_menu);
    hud_shutdown();
    debugdraw_shutdown();
    renderer_shutdown();
#ifdef __3DS__
    gfxExit();
#endif

    return 0;
}
