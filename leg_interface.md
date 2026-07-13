# Leg interface (`leg_control` / `stablecops::leg::AnkleLeg`)

Joint-space PD torque control of one closed-chain ankle over CAN only.
One `AnkleLeg` owns the two drives of one leg (CST mode), runs the
closed-chain-mapper PD pipeline on an internal control thread, and exposes
joint-side targets/gains in and joint + motor feedback out. No EtherCAT —
all feedback is derived from the CAN drives through the mapper.

- Interface: [leg/include/stablecops/leg/AnkleLeg.hpp](leg/include/stablecops/leg/AnkleLeg.hpp)
  (full pipeline, frame-convention and safety documentation in the header)
- Demo: [leg/examples/two_leg_demo.cpp](leg/examples/two_leg_demo.cpp)
- Mapper: `leg/4ne1_closed_chain_mapping` (git submodule)

## Wiring (demo robot)

| Bus  | Leg   | Node 1       | Node 2         |
|------|-------|--------------|----------------|
| can0 | right | top (inner)  | bottom (outer) |
| can1 | left  | top (inner)  | bottom (outer) |

Joint frame (user-facing, hardware-validated): positive pitch is the same
direction on both legs; roll is mirror-symmetric between legs. The mapper's
internal sign difference (pitch and roll both inverted, identically on both
legs) is handled inside `AnkleLeg`.

## Usage

```cpp
#include "stablecops/leg/AnkleLeg.hpp"
using namespace stablecops::leg;

AnkleLegConfig cfg;
cfg.can_interface = "can0";
cfg.side = flexion::Side::Right;
cfg.rt.enabled = true;               // optional SCHED_FIFO for bus + PD threads

AnkleLeg right(cfg);                 // construct ALL legs first...
right.start();                       // ...then start: boots CAN chain, enables
                                     // CST limp, resolves torque scaling
                                     // (0x6076 from both drives, gear ratio
                                     // from the motor profile), starts the PD.

right.setGains({20.0, 0.5, 20.0, 0.5});     // kp/kd pitch, kp/kd roll (start low!)
right.setTargets({0.1, 0.0});               // pitch, roll [rad], joint side

AnkleFeedback fb = right.feedback();        // one coherent control cycle:
// fb.pitch_rad / roll_rad (+ velocities)   joint position, mapped
// fb.joint_torque_{pitch,roll}_nm          measured joint torque, mapped
// fb.top / fb.bottom                       per motor: position_rad,
//                                          torque_motor_nm, torque_output_nm

right.limp();                        // gains to 0 (energised, zero torque)
right.stop();                        // zero torque + de-energise (also on dtor)
```

`running()` goes false on any drive fault / lost feedback (the leg commands
zero torque by itself); `lastError()` says why.

Prerequisites: drives zeroed to the mechanical joint zero (`zero_leg`) — the
absolute position is used directly as the actuator angle.

## Build — x86 (native)

```bash
git clone --recurse-submodules <repo>       # or: git submodule update --init
cmake --preset default -DSTABLECOPS_BUILD_MAPPER=ON
cmake --build --preset default -j
```

Needs `liblely-coapp` (pkg-config) and CMake >= 3.21; Eigen is fetched
automatically if not installed. `-DSTABLECOPS_ANKLE_GENERATION=gen3-1|gen3-2`
selects the ankle hardware (default gen3-2).

## Build — ARM (aarch64 cross)

Uses [cmake/toolchains/aarch64-linux-gnu.cmake](cmake/toolchains/aarch64-linux-gnu.cmake).
On the build host you need:

1. The cross compiler: `apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu`
2. An aarch64 sysroot containing the ARM build of `liblely-coapp` (+ `.pc`
   files) — easiest is to rsync `/usr` and `/lib` from the robot's rootfs.

```bash
export STABLECOPS_ARM_SYSROOT=/path/to/aarch64-sysroot
cmake --preset arm64                        # build-arm64/, Release, mapper ON
cmake --build --preset arm64 -j
```

(Alternatively pass `-DCMAKE_SYSROOT=...` instead of the env var.)
Building **natively on an ARM board** needs no toolchain at all — the x86
instructions apply unchanged.

## Run the demo

```bash
sudo ./canup.sh can0 can1
sudo build/examples/two_leg_demo --rt       # --right-only / --left-only for bring-up
```

Interactive: `[r|l|b] pitch|roll <deg>`, `[r|l|b] kp|kd <v>` (per-axis:
`kpp|kpr|kdp|kdr`), `st` status, `s` all limp, `q` quit. Telemetry streams to
PlotJuggler (UDP-JSON, port 9870, keys `r_*` / `l_*`).

**Safety**: gains start at 0 (limp); there is no artificial torque clamp
(the drives' 0x6072 max torque still applies). Bring `kp` up slowly — if a
joint diverges, the torque sign is wrong: `--torque-sign -1`
(`AnkleLegConfig::torque_sign`). `s`, Ctrl-C and any fault all drop to zero
torque.
