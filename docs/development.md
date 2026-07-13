# Development guide

Building from source, running the tests, the CLI tools, the runnable examples,
and real-time tuning. For the library API see the top-level
[`README.md`](../README.md); for runtime internals see
[`architecture.md`](architecture.md).

## Build

```bash
sudo apt-get install pkg-config liblely-coapp-dev liblely-co-tools python3-dcf-tools

cmake --preset default
cmake --build --preset default
```

Artifacts land in `build/`: `libstablecops.so`, the tools (`stablecops_master`,
`stablecops_commissiond`), the unit tests (`stablecops_tests`), and the
examples under `build/examples/`.

Install (versioned `.so`, headers, `stableCOPS` CMake package, sample data
under `share/stablecops/`):

```bash
cmake --install build --prefix /your/prefix   # sudo for a system prefix
```

CMake options (all default ON when stableCOPS is the top-level project, OFF
when embedded): `STABLECOPS_BUILD_TOOLS`, `STABLECOPS_BUILD_EXAMPLES`,
`STABLECOPS_BUILD_TESTS`, `STABLECOPS_INSTALL`. EtherCAT extras are opt-in
(`STABLECOPS_BUILD_ECAT`, see below).

## Tests

Framework-free unit tests cover the pure parts of the library: DS402
statusword decoding, fault/abort-code decoding, the PDO-summary loader, and
profile resolution.

```bash
build/stablecops_tests        # run directly, or:
ctest --test-dir build -R stablecops_tests --output-on-failure
```

No hardware or CAN interface is needed. Add checks here whenever you touch
`ds402::decodeState`, the diagnostics tables, `config::PdoMap`, or
`config::resolveMotorConfig`.

## Lint & format

clangd/clang-tidy/clang-format read `build/compile_commands.json` plus the repo
configs (`.clangd`, `.clang-tidy`, `.clang-format`).

```bash
tools/lint.sh format          # rewrite files in place
tools/lint.sh format-check    # fail if anything is unformatted
tools/lint.sh tidy            # clang-tidy static analysis
tools/lint.sh                 # format-check + tidy
```

Note: pip-installed clang tools default to a gcc toolchain dir without
libstdc++ headers, so `.clangd` and `tools/lint.sh` pin
`--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11`; adjust for your system.

## Bring up CAN

```bash
sudo ./canup.sh      # sets the bitrate and txqueuelen 1000
```

## Motor profiles & generated artifacts

Runtime configuration is generated from a motor profile YAML — the single
source of truth for the PDO layout, SYNC period, and actuator runtime settings:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/euservo_rp.yml
```

Full workflow and profile reference:
[`canopen_motor_pipeline.md`](canopen_motor_pipeline.md).

## CLI: `stablecops_master`

Bring-up and diagnostics front-end over the runtime. Without `--run` it only
prints the resolved configuration.

```bash
# Print the resolved configuration (profile values included), no CAN access:
build/stablecops_master --can can0

# Boot and dump live CANopen/DS402 objects (read-only):
build/stablecops_master --can can0 --node 1 --inspect --run

# Receive cyclic PDO feedback without energising the drive:
build/stablecops_master --can can0 --node 1 --monitor --run

# Safely enable, or enable + hold the current CSP position:
build/stablecops_master --can can0 --node 1 --enable --run
build/stablecops_master --can can0 --node 1 --hold-position --run

# Select the operation mode at boot (csp|csv|cst cyclic, pp|pv|pt profile):
build/stablecops_master --can can0 --node 1 --mode csv --enable --run
build/stablecops_master --can can0 --node 1 --mode pv --profile-accel 200000 --enable --run

# Guarded CSP step (only after hold/enable is verified):
build/stablecops_master --can can0 --node 1 --csp-relative 1000 --max-position-step 1000 --run

# Raw SDO write at boot, persist to NVM:
build/stablecops_master --can can0 --node 1 --set 0x605C:0:i16=0 --save --run

# Several nodes on one shared SYNC, with a cadence/jitter readout:
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --run
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --rt --rt-cpu 2 --run
```

## CLI: `stablecops_commissiond` (browser UI)

Boots the runtime in monitor mode and serves a local web UI for status, CiA402
enable / quick-stop / stop / fault-reset, setpoints, homing, and typed object
reads/writes:

```bash
build/stablecops_commissiond --can can0 --nodes 1,2 --mode csp
# open http://127.0.0.1:8765/
```

- Motion goes through the same `MotorDrive`/CiA402 path as the library; the
  object panel is for parameters and diagnostics (controlword/targets are
  PDO-driven on this firmware — use the motion buttons).
- Send the mode (0x6060) with **Send Mode** before enabling or before the first
  setpoint, then confirm the displayed mode in the status panel.
- Homing fields start from the actuator's profile defaults; the object panel
  loads the generated normalized EDS (`--eds` to override).
- With several nodes the daemon boots them on one shared bus/SYNC; the target
  node selector picks which drive receives commands.

## Examples

Each example is a small program built on the `MotorDrive` API; the common
bus/node flags live in [`examples/example_cli.hpp`](../examples/example_cli.hpp).
Defaults command **no motion** (a zero setpoint); a nonzero setpoint MOVES the
motor, so keep the joint clear.

| Example | Mode | What it shows |
| --- | --- | --- |
| `pdo_feedback_monitor` | monitor | Decoded cyclic feedback, power stage off |
| `enable_and_hold` | CSP | Enable and hold the current position |
| `csp_position` | CSP | Stream a sine wave around the start position |
| `csv_velocity` | CSV | Stream a constant target velocity |
| `cst_torque` | CST | Stream a constant target torque |
| `pp_move` | PP | One relative profile-position move |
| `pv_velocity` | PV | Velocity via the drive's accel/decel ramp |
| `pt_torque` | PT | Torque via the drive's torque slope |
| `multi_drive` | multi | Two drives on one shared bus, jitter readout |

```bash
build/examples/pdo_feedback_monitor --can can0 --node 1 --seconds 10
build/examples/csv_velocity         --can can0 --node 1 --velocity 0 --seconds 5
build/examples/multi_drive          --can can0 --nodes 1,2 --seconds 10   # --enable to energise
```

Leg-specific tools live under `leg/examples/`:

| Tool | Needs | What it does |
| --- | --- | --- |
| `zero_leg` | CAN | Hardstop-midpoint homing + leg zero offset, saved to NVM |
| `dump_config` | CAN | Read-only side-by-side dump of both drives' parameters (optional CSV) |
| `two_leg_demo` | CAN (+mapper) | Both ankles (can0=right, can1=left) under interactive joint-space PD torque control via `AnkleLeg` |
| `capture_ecat_zero`, `capture_rom`, `validate_trajectory`, `validate_mapper`, `ankle_pd_torque` | EtherCAT (+CAN) | Encoder calibration / validation workflows |

## Closed-chain ankle control (leg_control)

`-DSTABLECOPS_BUILD_MAPPER=ON` builds the vendored closed-chain mapper
(`leg_mapper`, select the hardware with
`-DSTABLECOPS_ANKLE_GENERATION=gen3-1|gen3-2`) plus `leg_control`, a CAN-only
library on top of it — no EtherCAT required. Its `stablecops::leg::AnkleLeg`
(`leg/include/stablecops/leg/AnkleLeg.hpp`) owns the two drives of one leg and
runs the ankle_pd_torque pipeline (CST, joint-space PD through the mapper) on
an internal control thread; users set joint-side pitch/roll targets and
per-axis kp/kd, and read joint position/velocity, joint+motor torque and motor
position feedback — all derived from the CAN feedback through the mapper.

```bash
cmake --preset default -DSTABLECOPS_BUILD_MAPPER=ON
cmake --build --preset default
sudo ./canup.sh can0 can1
sudo build/examples/two_leg_demo --rt
```

## EtherCAT + CAN (motion-faster)

The repo vendors the EtherCAT library `mm/motion-faster` so one program can
read EtherCAT encoders while stableCOPS moves CAN motors. Off by default (pulls
SOEM + yaml-cpp via FetchContent, needs CMake >= 3.28):

```bash
cmake --preset default -DSTABLECOPS_BUILD_ECAT=ON
cmake --build --preset default
```

EtherCAT needs a raw NIC (root / `CAP_NET_RAW`). Adding
`-DSTABLECOPS_BUILD_MAPPER=ON` on top also builds the EtherCAT-validated mapper
tools (`validate_mapper`, `ankle_pd_torque`).

```bash
sudo build/examples/motion_faster_encoder enp0s31f6
sudo build/examples/motion_faster_can_move --ecat enp0s31f6 --can can0 --node 1 --velocity 0
```

## Real-time tuning

The bus loop thread can be tuned for deterministic cadence — opt-in, shared by
all drives on an interface (`MotorConfig::rt` or the `--rt*` flags):

```cpp
config.rt.enabled = true;     // SCHED_FIFO
config.rt.priority = 80;      // 1..99
config.rt.cpu = 2;            // pin to a core (-1 = unpinned); pair with isolcpus
config.rt.lock_memory = true; // mlockall keeps the cyclic path off the pager
```

This needs privileges; without them the loop logs one warning and keeps running
at normal priority. Grant per user via `/etc/security/limits.conf`:

```
<user>  -  rtprio   99
<user>  -  memlock  unlimited
```

Verify the achieved cadence with `cyclicStats()` (CLI `--stats`): interval
min/max/mean and worst-case jitter vs the profile's SYNC period. Bandwidth
rule of thumb: at 1 Mbit/s classic CAN, two drives at a 1 ms SYNC with the
cyclic superset layout are close to the practical bus limit — slow the
profile's `master.sync_period` (e.g. 2000 µs) before adding nodes.

## Repository layout

- `include/stablecops/` + `src/` — the library:
  - `config/` — `MotorConfig` + profile resolution, generated-summary loader.
  - `ds402/` — transport-independent DS402 decoding, feedback types, diagnostics.
  - `lely/` — the Lely adapter (`MotorDriver`); the only place Lely is used.
  - `app/` — the public `MotorDrive` handle, shared `Bus`, loop owner, RT tuning.
  - `log/` — the pluggable logging facade.
- `src/tools/` — `stablecops_master`, `stablecops_commissiond`.
- `tests/` — unit tests (`stablecops_tests`).
- `examples/`, `leg/examples/` — runnable programs (tables above).
- `config/motors/` — motor profiles (single source of truth).
- `eds/EDS files/` — immutable vendor EDS files.
- `tools/generate_canopen_config.py` — profile → artifacts generator.
- `generated/canopen/`, `dcf/master.dcf` — derived artifacts; never hand-edit.
- `docs_motor/` — vendor manuals (see [`eyou_rp_notes.md`](eyou_rp_notes.md)).
