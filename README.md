# dirt2

A from-scratch DiRT 2-style rally driving game for the Nintendo 3DS. Written in
C against devkitPro / libctru / citro3d, with no engine and no ported code —
the rigid body, the suspension, the tyre model and the terrain are all written
for this project.

Old 3DS is the baseline target: 268 MHz ARM11, VFPv2 only (no NEON), ~64–80 MB
of app RAM and 6 MB of VRAM in two 3 MB banks.

![placeholder](meta/icon.png)

## What this release actually is

**A car you can drive, on a closed circuit, against a lap clock.** There are no
opponents and no car model beyond a debug shape — those are the next
milestones. What works today:

- A **closed oval circuit** with painted edges, a start/finish line, lap
  counting and lap timing (current, last and best)
- **Invisible walls** just outside the track, with a 4 m gravel run-off between
  the racing line and the wall — off the ribbon you lose grip, past the wall
  you simply stop
- A **pause menu** on SELECT, with an in-app **update check** (see below)

- A four-wheel vehicle with a real **rigid body** (semi-implicit Euler,
  quaternion orientation renormalised every step)
- **Raycast suspension** per wheel — spring, damper, anti-roll bar, bottom-out
  stiffening past max travel, and an airborne auto-level assist
- A **combined-slip tyre model**: slip ratio and slip angle go into one
  `rho = sqrt((SR/SR_peak)² + (α/α_peak)²)` curve lookup and come back out as
  Fx/Fy, so grip at the friction-ellipse boundary is never double-counted
- A **drivetrain** — engine RPM, fixed gear and final drive, FWD/RWD/AWD split
- A **fixed 120 Hz physics step** with render interpolation, so the frame rate
  and the simulation rate are independent
- A **smoothed chase camera** and an on-screen debug readout (speed, FPS,
  frame time)
- A procedural **three-zone test ground**: flat, rolling hills, and a washboard
  section that exists specifically to try and destabilise the dampers

## Controls

| Button | Action |
| --- | --- |
| R | Throttle |
| L | Brake |
| A | Handbrake |
| Circle Pad | Steer |
| SELECT | Pause menu |
| START | Exit |

L and R are digital buttons with no analog travel, so throttle and brake are
**ramped in software** rather than snapping between 0 and 1 — no module outside
`input.c` ever sees the raw button state.

### You can no longer fall off the world

v1.0.0 had no edges: outside roughly X ±25 m the ground query returned "no
ground" and the car fell forever with no way back. This release closes that.
Three bands, measured from the centre line of the track:

- **0 – 5 m** — the tarmac ribbon, full grip
- **5 – 9 m** — gravel run-off, reduced grip, and the lap readout turns red
- **8.4 m** — an invisible wall the car cannot be pushed past

The wall sits *outside* the painted edge on purpose, so the off-track grip
penalty is somewhere you can actually reach and drive on rather than dead code.

## Building

Needs devkitPro with devkitARM and libctru.

```bash
make
```

That produces `dirt2.3dsx`, `dirt2.elf` and `dirt2.smdh`. `make cia`
additionally builds `dirt2.cia` from `dirt2.rsf` and the placeholder art in
`cia/` — it needs `makerom` and `bannertool` in `$DEVKITPRO/tools/bin`.

The 3DS target builds **only** in the devkitPro MSYS2 shell.

The physics core also builds on the host, with plain system `gcc` and zero 3DS
dependencies, so it can be tested without an emulator:

```bash
make -f Makefile.host clean test
```

`make -f Makefile.host test-all` runs three suites: the physics core, the race
suite (track, lap timing) and the barrier suite (the invisible walls).
`make -f Makefile.host test-rates` runs the physics suite again at 60 Hz and
100 Hz in separate build directories — the physics must not change with the
step rate, and this is what proves it.

## Installing the CIA

Grab the `.cia` from [Releases](../../releases), copy it to your SD card, and
install it with FBI. Requires custom firmware.

## Install on your 3DS

![QR](meta/qr-latest.png)

Scan this in FBI's "Scan QR Code" to download and install the latest release
directly on the console.

Each release also gets its own branch and its own pinned QR code under `meta/`,
so a specific version can be installed rather than whatever is newest:

| Version | Branch | QR |
| --- | --- | --- |
| v1.0.2 | `v1.0.2` | `meta/qr-v1.0.2.png` |
| v1.0.0 | `v1.0.0` | `meta/qr-v1.0.0.png` |

## Checking for updates on the console

Press **SELECT** while driving to pause. **Options → Update** asks GitHub
whether a newer release exists and shows this build's version and the latest
published version side by side, with a QR code for that release on the bottom
screen.

It **checks and reports only** — it does not download or install anything.
Install with FBI and the QR code, as above.

This talks to GitHub over libcurl + mbedTLS rather than the console's own
`ssl:C` service, which was measured on real hardware to top out at TLS 1.1
while GitHub has required TLS 1.2 or better since 2018.

## Art

The icon and banner in `meta/` are **placeholder art**, generated by
`meta/make_assets.py` to make the CIA build. Replace them and rebuild if you
want something better. The car itself is currently debug wireframe — there is
no textured model yet.

## Verification status

Measured, not asserted:

- **Host test suites, from a clean build** (`make -f Makefile.host
  test-all`): physics 68 checks 0 failures (`PHYSICS_HZ=120`); `test_race`
  1475 checks 0 failures; `test_barrier` 68 checks 0 failures; overall exit
  code 0.
- **Fault injection:** both the race suite and the barrier suite have a red
  arm; each returns exit 2 with named failures, so the suites can actually
  go red.
- **Containment sweep** over the whole reachable area (racing ribbon plus
  gravel run-off out to the wall): 0 of 4284 sampled points had no ground
  under them; reachable area spans x -32.4..32.4 m, z 5.6..166.4 m. The red
  arm of the same sweep (world shrunk to half-width 30 m) reported 152 of
  4284 missing, so the sweep can fail.
- **Link chain**, from the symbol table on `dirt2.elf` (`arm-none-eabi-nm`):
  563 `curl_` symbols, 561 `mbedtls_` symbols, and `socInit`, `update_check`,
  `qr_encode_url` and `pausemenu_update` all present as defined text
  symbols. No unresolved symbols other than the usual weak ones.
- **The CIA artifact** contains the CA bundle: 121
  `-----BEGIN CERTIFICATE-----` and 121 `-----END CERTIFICATE-----`
  occurrences in `dirt2.cia`, and the RomFS filename `cacert.pem`.
  `dirt2.cia` is 951,232 bytes; `dirt2.3dsx` is 1,240,832 bytes.
- **In the Azahar 2126.0 emulator, booting the CIA** rather than the 3dsx:
  the pause menu opens on SELECT and shows Resume / Options / Quit; Options
  → Update completes and reports current 1.0.2, latest 1.0.0, "up to
  date", and draws a QR code for the release on the bottom screen; B backs
  out; SELECT resumes and the car drives again at 60 FPS. The simulation is
  frozen while the menu is open (steps 0, alpha 0.00).
- **The invisible wall, driven in the emulator** rather than only
  unit-tested: with throttle held at full (6209 rpm) and the steering on
  full lock for 25 seconds, the car came to rest pinned at 8 m off the
  racing line, position x -16 y +1 z +98, all four wheels grounded (loads
  2460/2332/3490/3306 N), lap readout red for off-track. It could not be
  pushed further.
- **The circuit's corner radius** went from 12 m to 24 m this release,
  raising the physics-limited corner speed from about 37 km/h to about
  52 km/h. The straights are unchanged at 96 m.

Not verified:

- **Nothing in this project has ever run on real 3DS hardware.** Every
  measurement above is from the host build or from the Azahar emulator.
- The CIA has been built and booted in an emulator but never installed on a
  console with FBI, so the QR install route is untested end to end.
- The TLS path works in the emulator, where the raw sockets are serviced by
  the host machine's network stack. It has not been exercised on a
  console's own Wi-Fi.
- The CA bundle is a 121-certificate, ~185 KB file copied unmodified from a
  sibling project. That project has an open, unresolved concern that
  parsing all 121 certificates peaks at roughly 325 KB of heap during the
  TLS handshake. This project has not independently reproduced or ruled
  out that measurement.
- The physics numbers are placeholders that produce plausible behaviour;
  they are not tuned against anything real.

## Licence

None specified.
