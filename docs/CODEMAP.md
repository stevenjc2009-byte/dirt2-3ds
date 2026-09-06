# dirt2 -- Code Map

A from-scratch DiRT 2-style rally driving game for Nintendo 3DS (devkitPro /
libctru / citro3d + citro2d). Old 3DS is the baseline target (268 MHz ARM11,
VFPv2 only -- no NEON, ~64-80 MB app RAM, 6 MB VRAM in two 3 MB banks); New
3DS enhancements are later work.

This is a playable driving game, not a skeleton or a set of stubs. The
modules below are real, working code with host-side test coverage and/or
on-hardware verification behind them: a rigid-body vehicle physics core
(chassis, suspension, combined-slip tyres, drivetrain), a citro3d/citro2d
rendering pipeline (chase camera, world/debug drawing, and a bottom-screen
telemetry HUD), a drivable closed-loop circuit with invisible-wall
containment, and lap counting/timing. `source/net/`, `source/update/` and
`source/ui/` (HTTPS GitHub-release version checking and a QR-code renderer
for grabbing a build with the console's own camera) are also implemented and
are now in `Makefile`'s `SOURCES` list; they are reached from `main.c` through
the pause menu's update screen. They remain absent from `Makefile.host`'s
`SRCS`, which is correct -- they need libctru/curl and cannot run on the host.
See their sections below and the note in Build.
Where a specific file genuinely is unfinished, placeholder, or unverified,
its own row says so explicitly. The blanket claim that used to be here
("every `.c` file listed below is currently a compiling stub") stopped being
true after v0.1 and has been removed.

## Directory layout

```
dirt2/
  Makefile              3DS build (devkitPro), .3dsx + .cia targets, romfs
  Makefile.host          host build of the physics core (WSL/gcc), zero 3DS deps
  source/
    main.c               entry point + frame loop wiring (real glue, not a stub)
    version.h             the ONE place DIRT2_VERSION is defined
    core/                 math, rigid body, fixed timestep -- no vehicle knowledge
    vehicle/              chassis + wheels + the canonical per-step order
    input/                Circle Pad / button reading, ramped to vehicle-ready input
    race/                 the circuit: waypoint loop, invisible-wall containment,
                           lap counting, lap timing
    render/                citro3d/citro2d setup, camera, debug wireframe/text draw,
                           bottom-screen telemetry HUD
    world/                 procedural Phase 1 test heightfield
    net/                   HTTPS GET over curl+mbedTLS -- NOT libctru's ssl:C,
                           which is measured to top out at TLS 1.1; see Build
    update/                GitHub release version check, built on net/
    ui/                    pause menu, and QR-code encoding: qr.c wraps the
                           VENDORED qrcodegen.c/.h (Nayuki QR-Code-generator)
  tests/                  host-side test suites: physics core, race (track+lap),
                           barrier containment -- three separate binaries
  romfs/                  cacert.pem, the CA bundle net/http.c needs -- the only
                           asset in this repo so far
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
| `render/hud.h` / `.c` | The **bottom** screen (320x240) and its own render target -- `renderer.c` owns the top screen and nothing else, so the panel can be dropped or replaced without touching the 3D path. Speed, engine rpm with a redline-marked bar, ramped throttle/brake/steer bars, per-wheel compression/load, lap count and lap times. `hud_draw` takes one `HudStats` struct by pointer and therefore has **zero dependency on `Vehicle`** -- it renders numbers it is handed and cannot be broken by a physics refactor. Two ordering rules, both load-bearing: it must be called **inside** `renderer_frame_begin`/`end` and **after** everything top-screen (it switches the draw target and leaves it switched), and it deliberately does **not** call `C2D_Prepare` because `debugdraw_frame_end` already did this frame. |
| `render/debugdraw.h` / `.c` | Immediate-mode world-space lines/wire boxes/points (citro3d) and screen-space text (citro2d) -- the load-bearing Phase 1 tool for actually **seeing** suspension compression/rebound and tyre slip on real hardware, per this project's verify-before-claiming-done discipline. Explicitly separate from `renderer.h`'s real car/world rendering. |

### race/ -- the circuit, containment, lap counting and lap timing

| File | Owns |
|---|---|
| `race/track.h` / `.c` | A closed-loop circuit as an ordered ring of waypoints (`center` + `half_width`), **2D in the XZ plane** -- a waypoint's Y is never read, because `world/testground.h`'s height query is the only source of truth for how high the ground is. `track_query` returns the nearest segment, 0..1 progress round the loop, and a **signed** lateral offset from the centreline, using a windowed search around a caller-held `cached_segment` hint with a full-scan fallback. The module holds no state of its own: that `int` is the only thing persisted, which is why callers (lap counting, the barrier, the HUD) each keep their own. `track_build_example_oval` builds the concrete stadium oval, ENLARGED for v1.0.2 (verified against the current file): 10 m wide (5 m half-width, was 8 m), 24 m-radius hairpins (was 12 m -- doubled so the real driven top speed on the 96 m-unchanged straights doesn't overshoot the corner's grip), 16 arc segments per hairpin (was 12, so a straight-chord-built wall doesn't cut the now-wider corner). Sized against the testground extents `main.c`'s `make_placeholder_testground_config` grew to match: `world_half_width` 38.0, `flat_length` 45 + `hills_length` 70 + `washboard_length` 60 = 175 in Z (verified directly in `source/main.c`, not just the comment in `track.c`). |
| `race/barrier.h` / `.c` (**new this version**) | The invisible walls that make the circuit physically inescapable, built entirely on `track_query` -- no new collision geometry, no dependency on `vehicle.h`. `barrier_apply` runs once per physics step, after `vehicle_step`'s integration, and is a **true no-op** (no read or write at all) whenever the car is within `half_width + BARRIER_RUNOFF_M (4m) - BARRIER_INSET_M (0.6m)` of the centreline -- i.e. on the painted ribbon or the gravel run-off band beyond it; only tarmac vs. gravel changes grip out there (via `main.c`'s tyre-surface lookup), not this module. Past that limit it clamps `position` back to the limit along the segment's lateral axis (XZ only, `position.y` untouched) and reflects only the outward component of `linear_velocity` at `BARRIER_RESTITUTION_FACTOR` (0.25) -- the along-track component and vertical velocity are never touched. Deliberately NOT placed at the ribbon edge itself: an earlier version that did made the gravel-grip-penalty code unreachable, which the file's own header calls out as the same defect class as a feature landing unreachable. Guards against NULL/degenerate/non-finite input by doing nothing rather than propagating a bad value. |
| `race/lap.h` / `.c` | Lap counting and timing on top of `track_query`'s progress. A lap counts only on a **forward wrap-crossing** and only once the car has previously reached progress >= 0.5 (`far_side_reached`) -- that gate is what defeats sitting on the line, an out-and-back, and a reverse-then-forward double crossing, three cases a naive "progress jumped by more than 0.5" detector counts wrongly. A crossing that happens mid-step splits that step's `dt` proportionally between the finishing lap and the new one. Called **once per physics step**, never once per render frame: the times are accumulated `dt`, and at 128 km/h a render-cadence sample can step clean over the start line. |

### world/ -- Phase 1 test terrain

| File | Owns |
|---|---|
| `world/testground.h` / `.c` | Procedural three-zone heightfield along +Z: flat (settle test), rolling hills (tracks a slowly-changing surface), washboard (high-frequency small-amplitude corrugation, tests damper stability at 120 Hz). `testground_height_query` matches `suspension.h`'s `SuspensionGroundQuery` signature exactly so it drops into `vehicle_init` with no glue code. Out-of-bounds returns "no ground" rather than clamping/extrapolating. |

### net/ -- HTTPS, for the one thing that needs it (**new in v1.0.2**)

`net/`, `update/` and `ui/` are all in `Makefile`'s `SOURCES` and are reached
from `main.c` via the pause menu's update screen. They are deliberately NOT in
`Makefile.host`'s `SRCS`: they need libctru and libcurl, which the host build
does not have and is not going to grow.

**A caveat specific to this group, worth knowing before trusting a green
build.** `3dsx.specs` links with `--gc-sections`, so an unreferenced function
here is silently dropped and libcurl is then never pulled from its archive at
all. That means a successful `make` does NOT by itself prove the curl/mbedTLS
link chain works -- it only proves the linker was not asked to do the hard
part. The check that actually proves it is `arm-none-eabi-nm dirt2.elf` finding
real `curl_` and `mbedtls_` symbols. This was learned the direct way on this
project: the first build after adding the `LIBS` chain returned exit 0 while
containing zero curl symbols.

**Never verified on real 3DS hardware:** soc:U bring-up and whether a
romfs-loaded `CAINFO_BLOB` completes a TLS handshake on this console. The pure
logic (version compare, tag parsing) and the request shape are host-tested
against a real TLS 1.2 server; the 3DS-side transport is not.

| File | Owns |
|---|---|
| `net/http.h` / `.c` | A small blocking HTTPS **HEAD-shaped** GET (`CURLOPT_NOBODY` -- costs DNS+TLS+headers only, no body) over libcurl + 3ds-mbedtls, deliberately NOT libctru's own `httpc`/`ssl:C` service -- `ssl:C` has been MEASURED (sibling project skywave) to top out at TLS 1.1, and GitHub requires TLS 1.2+. `http_net_init()`/`http_net_exit()` own three global 3DS-wide resources (`soc:U`, libcurl's global state, a romfs mount for `romfs/cacert.pem`) as a paired, idempotent init/exit -- a second `http_net_init()` while already up is a documented no-op, not an error. The CA bundle is loaded into memory once and handed to curl as an in-memory `CURLOPT_CAINFO_BLOB`, specifically to avoid a lazy-open failure mode (`CURLE_SSL_CACERT_BADFILE`) the sibling project mc's updater hit on real hardware -- see the file's own header for that history. `http_get()` returns the underlying `CURLcode` as a positive int on transport failure, or `HTTP_ERR_NOT_INITIALIZED` (-1) if called out of sequence, and always fills `HttpResponse` (zeroed first) including `effective_url`, the actual point of this wrapper: GitHub's `/releases/latest` redirect is read from there. The mbedTLS certificate-parse memory cost (~325 KB peak, per mc's own measurement) has NOT been independently re-measured or verified on real DiRT2 hardware -- the file says so itself, and this codemap is not going to claim otherwise. |

### update/ -- "is there a newer release" check, built on net/ (**new in v1.0.2**)

| File | Owns |
|---|---|
| `update/update.h` / `.c` | `update_check()`: one blocking call that HEAD-requests `github.com/<owner>/<repo>/releases/latest` (the **web** redirect, not `api.github.com` -- deliberately, so it costs none of GitHub's 60-requests/hour unauthenticated API quota) and reads the version tag out of the final redirected-to URL rather than parsing any JSON. Scope is explicitly check-and-report only -- no download, no install. Fills an `UpdateResult` (status enum, `current_version` from `version.h`, `latest_version`, `release_url`, and a human-readable `detail` string that is REQUIRED to be specific on every failure path, e.g. "curl 28: Timeout was reached", not a bare "check failed" -- reasoned in the header as necessary for a console with no attached debugger). Also exposes two pure, host-testable helpers used internally: `update_version_compare()` (numeric per-segment compare, so "1.0.10" correctly sorts after "1.0.9" where a plain `strcmp` would not) and `update_parse_tag_from_url()` (pulls `vX.Y.Z` out of a `.../releases/tag/...` URL, stripping a leading `v` if present). |

### ui/ -- QR-code rendering (**new in v1.0.2**)

| File | Owns |
|---|---|
| `ui/qr.h` / `.c` | Project-specific wrapper around the vendored `qrcodegen` library (see below), turning its bit-packed API into one call (`qr_encode_url`) and one plain byte-per-module `QrCode` grid any renderer can walk with a nested loop. Capped at QR version 8 (49x49 modules) at error-correction level MEDIUM -- a capacity of 152 bytes in byte mode, MEASURED directly against the real library during development (not recalled from a table), leaving headroom over this project's real release URLs (~66 characters). No dynamic allocation: fixed stack-local scratch buffers sized by a compile-time `qrcodegen_BUFFER_LEN_FOR_VERSION` constant. Returns `false` with `out->size` left at 0 for a NULL/empty/too-long input; never overflows `out`. |
| `ui/qrcodegen.h` / `.c` | **VENDORED THIRD-PARTY CODE -- NOT this project's code, do not "clean up" or restyle it.** Project Nayuki's QR-Code-generator library (C port), MIT-style licence, copied in verbatim (byte-for-byte identical to upstream, per this project's own "copy, don't depend on a path outside the project" rule) at upstream commit `8329a7108fc22be3e1eec0a9f9318978579e3621` (2024-09-01) -- both files carry this exact provenance note at their own top. Re-vendor from upstream to update it; do not hand-edit. |

### tests/ -- host-side test suites: physics, race, barrier (three separate binaries)

| File | Owns |
|---|---|
| `tests/test_physics.c` | The actual checks, run via `run_physics_tests()`. Currently smoke-level (confirms `PHYSICS_HZ`/`PHYSICS_DT` resolve correctly, confirms the documented `vehicle_step` call sequence runs 10 steps without crashing) with `TODO(...)` markers for the real physical assertions once suspension/tyre/rigidbody are implemented. |
| `tests/test_race.c` | The race module's checks: progress monotonicity and wrapping, a clean lap, and the three false-positive cases the `far_side_reached` gate exists for (reverse crossing, out-and-back, back-and-forth), plus best-time-only-on-improvement, rate-independent timing, the proportional `dt` split on a same-step completion, `lap_notify_reset`, and starting mid-lap. Rate-independent via `seconds_to_steps`, same convention as `test_physics.c`, and carries the same `DIRT2_INJECT_FAULT` scheme (faults 1/2/3/4/6 for `lap.c`, 5 for `track.c`) so each check can be proven able to go red. Has its own `main()` and so builds as a **separate host binary** (`build-host-race/dirt2_race_tests`) from `test_main.c`. |
| `tests/test_barrier.c` (**new this version**) | The barrier suite: checks that well-inside-the-limit positions are a true bit-for-bit no-op (see `barrier.h`'s own rationale for why that matters), that an over-the-limit car is clamped to exactly the limit on the side it was already on and never flipped to the opposite side, that only the outward velocity component is reflected (at `BARRIER_RESTITUTION_FACTOR`) while the tangential component and `position.y`/vertical velocity are left alone, and a repeated-contact case expressed in simulated seconds via `PHYSICS_HZ` (same rate-independence convention as `test_race.c`). Uses `DIRT2_INJECT_FAULT` 10-13 (`barrier.c`'s own numbering, clear of `track.c`'s 5 and `lap.c`'s 1/2/3/4/6) and, per its own header, gets its pass/fail ground truth from an independent fresh `track_query` call rather than mirroring `barrier.c`'s own lateral-direction math, specifically so a shared sign bug can't cancel out and pass. Own `main()`, own host binary (`build-host-barrier/dirt2_barrier_tests`). |
| `tests/test_main.c` | Thin entry point: calls `run_physics_tests()`, turns the failure count into a process exit code (0 = all passed). |

### source/main.c

Real wiring, not a stub. Verified directly against the current file's frame
loop, which is wider than it used to be: `timestep_advance` decides how many
physics steps are owed this render frame, then for EACH of those steps, in
order, `input_update` -> `vehicle_step` -> `barrier_apply` -> `lap_update`
(`barrier_apply` must run after `vehicle_step`'s integration and before
`lap_update`, per `barrier.h`'s own contract -- `lap_update` decides
completion off the corrected position, not the pre-barrier one). Once all
owed steps are done: `camera_update`, then `renderer_frame_begin` /
`debugdraw_*` (top screen, including a `track_query` call through its own
segment cache for the on-screen lap readout) / `debugdraw_frame_end`, then
`hud_draw` (bottom screen, through yet another independent `track_query`
segment cache -- see `race/track.h`'s row above for why each caller keeps
its own), then `renderer_frame_end`. Carries clearly-marked placeholder
`VehicleParams`/`TestgroundConfig` numbers so the game compiles, links, and
runs on real hardware; those specific numbers are not tuned or verified,
which is a separate claim from the surrounding systems (physics, race,
rendering) being unimplemented -- they are not.

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
  **`SOURCES` (read directly out of the current `Makefile`) is
  `source source/core source/vehicle source/input source/render source/world
  source/race source/net source/update source/ui`.** `ROMFS := romfs` now
  ships a real asset, `romfs/cacert.pem`, which `net/http.c` reads once at
  init -- deleting it breaks every HTTPS request the update check makes.
  **`LIBS` link order is load-bearing**: `-lcitro2d -lcitro3d -lcurl
  -lmbedtls -lmbedx509 -lmbedcrypto -lz -lctru -lm`. `-lctru` sits
  second-to-last, not third as it used to, because curl needs libctru's BSD
  sockets via soc:U and the devkitARM linker resolves archives strictly left
  to right without re-scanning; with `-lctru` in its old position curl's
  socket references resolve against an archive already walked past. See the
  `net/` section above on why a clean `make` is not sufficient evidence that
  this chain actually linked.
- **Host physics/race/barrier test build** (`Makefile.host`): WSL only,
  plain system `gcc`, zero 3DS dependencies -- and, per its own header
  comment, deliberately NOT WSL's `gcc` invoked from Git Bash's MinGW
  environment or vice versa (different libc/ABI, has bitten this machine
  before). Three independent suites, each its own binary/object directory
  because each `tests/test_*.c` defines its own `main()`:
  - `make -f Makefile.host test` -- the physics suite (`tests/test_physics.c`)
    at the default 120 Hz; `test-rates` re-runs it at 60/100/120 Hz in
    separate build dirs, per `core/timestep.h`'s three-rate requirement.
  - `make -f Makefile.host race-test` -- the race suite (`tests/test_race.c`
    against `track.c`+`lap.c`), builds into `build-host-race/`.
  - `make -f Makefile.host barrier-test` -- the barrier suite
    (`tests/test_barrier.c` against `track.c`+`barrier.c`), builds into
    `build-host-barrier/`.
  - `make -f Makefile.host test-all` -- all three, physics first, stopping
    at the first failure.
  - `DIRT2_INJECT_FAULT=<N>` (race-test and barrier-test only) deliberately
    breaks one real piece of `track.c`/`lap.c`/`barrier.c` logic to prove a
    specific check can fail on real code -- see each suite's own header for
    its fault numbers.
  `net/`, `update/` and `ui/` are deliberately absent from `Makefile.host`'s
  `SRCS` -- they need libctru and libcurl. So there is no host-side suite
  covering HTTPS, the update check, or QR encoding. What coverage exists was
  done with throwaway probes outside the repo: version-compare and tag-parse
  logic against a real TLS 1.2 server, and QR output decoded back with
  OpenCV. The 3DS transport path itself has no automated coverage at all.
