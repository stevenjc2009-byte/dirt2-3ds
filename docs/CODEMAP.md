# dirt2 -- Code Map

Phase 1 skeleton for a from-scratch DiRT 2-style rally game on Nintendo 3DS
(devkitPro / libctru / citro3d + citro2d). Old 3DS is the baseline target
(268 MHz ARM11, VFPv2 only -- no NEON, ~64-80 MB app RAM, 6 MB VRAM in two
3 MB banks); New 3DS enhancements are later work.

This is task 1A of Phase 1: the project skeleton and the interface headers
every other Phase 1 module is built against. Every `.c` file listed below is
currently a **compiling stub** (empty/placeholder bodies, `TODO(module):`
markers) -- the headers are the real deliverable of this task and are the
contract the seven parallel implementation agents build to.

## Directory layout

```
dirt2/
  Makefile              3DS build (devkitPro), .3dsx + .cia targets, romfs
  Makefile.host          host build of the physics core (WSL/gcc), zero 3DS deps
  source/
    main.c               entry point + frame loop wiring (real glue, not a stub)
    core/                 math, rigid body, fixed timestep -- no vehicle knowledge
    vehicle/              chassis + wheels + the canonical per-step order
    input/                Circle Pad / button reading, ramped to vehicle-ready input
    render/                citro3d/citro2d setup, camera, debug wireframe/text draw
    world/                 procedural Phase 1 test heightfield
  tests/                  host-side smoke tests for the physics core
  romfs/                  empty for Phase 1 -- no assets shipped yet
  docs/CODEMAP.md          this file
```

## Ownership, file by file

### core/ -- math and simulation primitives, no vehicle/world knowledge

| File | Owns |
|---|---|
| `core/types.h` | `f32` (plain float -- see file header on why not fixed-point), `Vec3`/`Quat`/`Mat3` layout, `WheelIndex`/`AxleIndex`/`VehicleSide` enums, `VEHICLE_WHEEL_COUNT`/`VEHICLE_AXLE_COUNT`. |
| `core/vecmath.h` / `.c` | All Vec3/Quat/Mat3 *operations* (add, dot, cross, quaternion integrate/normalize/slerp, mat3 from quat). Scalar-only -- no NEON, VFPv2 has none. |
| `core/rigidbody.h` / `.c` | `RigidBody`: position, linear velocity, quaternion orientation (renormalised every step), angular velocity, mass, body-space diagonal inertia + its world-space inverse. `rigidbody_step` does semi-implicit Euler. |
| `core/timestep.h` / `.c` | Fixed-step accumulator + render interpolation alpha. `PHYSICS_HZ` (default 120, override with `-DPHYSICS_HZ=60` or `=100`) and derived `PHYSICS_DT` are compile-time constants -- see file header for why. |

### vehicle/ -- the car, and the order that makes it correct

| File | Owns |
|---|---|
| `vehicle/vehicle.h` / `.c` | `Vehicle` (one `RigidBody` chassis + 4 `Wheel`), and **`vehicle_step`'s eight-step canonical per-frame order** -- see the long comment block at the top of `vehicle.h` for the full order and the three ordering bugs it exists to prevent (stale normal load, one-frame-lagged slip feedback, wheel spin integrated from the post-integration body velocity instead of the one that produced this step's tyre force). Every other vehicle/* module is written to be driven by this order; nothing outside `vehicle.c` calls `suspension_solve`/`tyre_solve`/`drivetrain_update` directly. |
| `vehicle/suspension.h` / `.c` | Per-wheel raycast spring/damper (`suspension_solve`), anti-roll bar (`suspension_antiroll`), bottom-out handling (built into `suspension_solve`, extra stiffness past `max_travel` rather than a hard clamp), airborne auto-level PD torque (`suspension_airborne_autolevel`, gameplay assist for un-tumbling after a jump -- yaw untouched). Ground contact is queried through a `SuspensionGroundQuery` function pointer so this module has zero `#include` dependency on `world/testground.h`. |
| `vehicle/tyre.h` / `.c` | **Single-curve combined-slip tyre model.** `tyre_solve` takes slip ratio + slip angle + normal load together and returns Fx/Fy from ONE `rho = sqrt((SR/SR_peak)^2 + (alpha/alpha_peak)^2)` lookup, split back into components. Deliberately no independent Fx-only/Fy-only entry points -- see the file header's long comment on why that would double-count grip at the friction-ellipse boundary. This collapses to a single 1-D lookup table, which is what a device with no fast float divide wants. |
| `vehicle/drivetrain.h` / `.c` | Engine RPM model, fixed gear + final drive ratio, `DriveLayout` (FWD/RWD/AWD, even split, no differential yet), per-wheel drive/brake torque request (`drivetrain_update`). No gearbox/shifting/clutch/differential in Phase 1 scope -- explicitly deferred, not half-built. |
| `vehicle/vehicle_params.h` | Aggregates one vehicle's tuning: chassis mass/inertia, per-wheel `SuspensionConfig`, per-axle anti-roll rate, auto-level gains, default tyre surface, wheel inertia, `DrivetrainConfig`. Declares the struct shape only -- no default numbers shipped here; real tuning is Phase 1 content work (`source/main.c` currently carries clearly-marked placeholder numbers just so the skeleton links and runs). |

### input/ -- the ONE module allowed to touch hardware input

| File | Owns |
|---|---|
| `input/input.h` / `.c` | Reads Circle Pad (`hidCircleRead`, raw range roughly ±156, dead-zoned and normalised to [-1,1]) and buttons (`hidKeysHeld`) and turns them into `InputState` (steer, **ramped** throttle/brake in [0,1], digital handbrake). L/R are digital 3DS buttons with no analog travel -- `InputConfig` carries throttle/brake ramp and release rates specifically so no other module ever sees a raw instant-0-or-1 button state. `input.c` is the only translation unit allowed to call libctru's `hidCircleRead`/`hidKeysHeld`, guarded by `#ifdef __3DS__` so it still compiles (as a no-op body) on the host build. |

### render/ -- citro3d/citro2d, camera, debug drawing

| File | Owns |
|---|---|
| `render/renderer.h` / `.c` | One-time citro3d/citro2d/render-target setup; `renderer_frame_begin/end` brackets each GPU frame. Top screen only, no stereo yet (Old 3DS baseline). |
| `render/camera.h` / `.c` | Smoothed chase camera, reads the vehicle's already-**interpolated** draw pose (see `core/timestep.h`'s alpha) -- has no knowledge of physics steps or `PHYSICS_DT`. |
| `render/debugdraw.h` / `.c` | Immediate-mode world-space lines/wire boxes/points (citro3d) and screen-space text (citro2d) -- the load-bearing Phase 1 tool for actually **seeing** suspension compression/rebound and tyre slip on real hardware, per this project's verify-before-claiming-done discipline. Explicitly separate from `renderer.h`'s real car/world rendering. |

### world/ -- Phase 1 test terrain

| File | Owns |
|---|---|
| `world/testground.h` / `.c` | Procedural three-zone heightfield along +Z: flat (settle test), rolling hills (tracks a slowly-changing surface), washboard (high-frequency small-amplitude corrugation, tests damper stability at 120 Hz). `testground_height_query` matches `suspension.h`'s `SuspensionGroundQuery` signature exactly so it drops into `vehicle_init` with no glue code. Out-of-bounds returns "no ground" rather than clamping/extrapolating. |

### tests/ -- host-side physics smoke tests

| File | Owns |
|---|---|
| `tests/test_physics.c` | The actual checks, run via `run_physics_tests()`. Currently smoke-level (confirms `PHYSICS_HZ`/`PHYSICS_DT` resolve correctly, confirms the documented `vehicle_step` call sequence runs 10 steps without crashing) with `TODO(...)` markers for the real physical assertions once suspension/tyre/rigidbody are implemented. |
| `tests/test_main.c` | Thin entry point: calls `run_physics_tests()`, turns the failure count into a process exit code (0 = all passed). |

### source/main.c

Real wiring, not a stub: owns the frame loop order --
`input_update` -> `timestep_advance` -> `vehicle_step` (N times) ->
`camera_update` -> `renderer_frame_begin`/`debugdraw_*`/`renderer_frame_end`.
Carries clearly-marked placeholder `VehicleParams`/`TestgroundConfig` numbers
so the skeleton compiles, links, and runs on real hardware; none of those
numbers are tuned or verified.

## The canonical per-frame order (authoritative copy lives in `vehicle.h`)

1. per-wheel suspension raycast
2. spring + damper force from compression and compression velocity
3. normal load = suspension force, clamped at or above zero
4. tyre slip ratio and slip angle from wheel spin vs contact-patch velocity
5. tyre longitudinal and lateral force from slip, normal load, and surface
6. sum all wheel forces and torques into the rigid body
7. integrate the rigid body (semi-implicit Euler)
8. integrate wheel spin from drive/brake torque minus tyre-force reaction

Steps 1-5 run for **all four wheels** before step 6 begins for any of them;
step 7 runs exactly once; step 8 uses the tyre force captured **before**
step 7's integration, never a value re-derived from the post-integration
body velocity. See `vehicle.h`'s header comment for the full rationale and
the three ordering bugs this sequence prevents.

## Build

- **3DS target** (`Makefile`): devkitPro MSYS2 shell only.
  `export DEVKITARM=/opt/devkitpro/devkitARM` (devkitPro's MSYS2 sets this
  itself), then `make` from the `dirt2/` directory. Produces `dirt2.3dsx`
  (and `dirt2.elf`/`dirt2.smdh`). `make cia` additionally builds
  `dirt2.cia`, and needs `dirt2.rsf` (copied from raytracer3ds's spec, which
  came from Universal-Updater's known-good one) plus `cia/banner.png` and
  `cia/banner.wav` (placeholder art, regenerate with
  `python meta/make_assets.py`). All three exist as of the v1.0.0 release.
- **Host physics test build** (`Makefile.host`): WSL only, plain system
  `gcc`, zero 3DS dependencies. `make -f Makefile.host clean test` builds
  and runs the suite once at the default 120 Hz; `make -f Makefile.host
  test-rates` runs it again at 60 Hz and 100 Hz in separate build
  directories, per `core/timestep.h`'s three-rate requirement.
