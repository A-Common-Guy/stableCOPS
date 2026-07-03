# stableCOPS development guide

This guide covers building from source, the CLI tools, generating CANopen
artifacts, the runnable examples, real-time tuning, and the repo layout. For the
library API a consumer actually integrates against, see the top-level
[`README.md`](../README.md). For the data-driven EDS -> DCF pipeline, see
[`canopen_motor_pipeline.md`](canopen_motor_pipeline.md).

## Build from source

```bash
sudo apt-get update
sudo apt-get install pkg-config liblely-coapp-dev liblely-co-tools python3-dcf-tools

cmake --preset default
cmake --build --preset default
```

Artifacts land in `build/`: the shared library `libstablecops.so`, the tools
(`stablecops_master`, `stablecops_commissiond`), and the examples under
`build/examples/`.

Install (produces the versioned `.so`, headers, `stableCOPS` CMake package, and
sample data under `share/stablecops/`):

```bash
cmake --install build --prefix /your/prefix   # sudo for a system prefix
```

## Generate CANopen artifacts

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

This derives a normalized EDS, dcfgen YAML, PDO summary, and `dcf/master.dcf`
from the immutable vendor EDS. Full workflow:
[`canopen_motor_pipeline.md`](canopen_motor_pipeline.md).

## Bring up CAN

```bash
sudo ./canup.sh      # sets bitrate and txqueuelen 1000
```

## CLI tool: `stablecops_master`

A thin command-line front-end over the runtime, useful for bring-up and
diagnostics.

```bash
# Print resolved configuration without opening CAN:
build/stablecops_master --can can0

# Boot the drive:
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --run

# Boot and dump live CANopen/DS402 objects:
build/stablecops_master --can can0 --node 1 --inspect --run

# Receive cyclic PDO feedback without energising the drive:
build/stablecops_master --can can0 --node 1 --monitor --run

# Safely enable the DS402 power stage:
build/stablecops_master --can can0 --node 1 --enable --run

# Enable and hold the current CSP position:
build/stablecops_master --can can0 --node 1 --hold-position --run

# Select the operation mode at boot (csp|csv|cst cyclic, pp|pv|pt profile):
build/stablecops_master --can can0 --node 1 --mode csv --enable --run
build/stablecops_master --can can0 --node 1 --mode pv --profile-accel 200000 --profile-decel 200000 --enable --run

# Guarded CSP step (only after hold/enable is verified):
build/stablecops_master --can can0 --node 1 --csp-relative 1000 --max-position-step 1000 --run

# Several nodes on one shared SYNC, with a cyclic cadence/jitter readout:
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --run
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --rt --rt-cpu 2 --run
```

## CLI tool: `stablecops_commissiond` (browser UI)

Boots the runtime in monitor mode, configures cyclic PDO feedback, and serves a
local web page for status, typed object reads/writes, CiA402
enable/quick-stop/stop/fault-reset, setpoints, and encoder feedback:

```bash
build/stablecops_commissiond --can can0 --dcf dcf/master.dcf --nodes 1,2 --mode csp
# open http://127.0.0.1:8765/
```

Use `--mode pp|pv|pt|csv|cst` to match the profile/cyclic mode. Controlword and
target objects are driven through the PDO/CiA402 path once mapped, so use the UI
motion buttons for movement and the object panel for parameters/diagnostics.
Send the mode (0x6060) with the separate **Send Mode** action before enabling or
before the first setpoint, then confirm the displayed mode in the status panel.
With several nodes the daemon boots them on one shared bus/SYNC and the target-
node selector chooses which drive receives commands. The object panel loads the
generated normalized EDS (`--eds` to override) so you can search the vendor
registers (access type, PDO mapping, data type, default value).

## Examples

Each example is a small, self-contained program built on the `MotorDrive` API.
Defaults command **no motion** (a zero setpoint); passing a nonzero setpoint
SPINS the motor, so keep the joint clear.

| Example | Mode | What it shows |
| --- | --- | --- |
| [`pdo_feedback_monitor`](../examples/pdo_feedback_monitor.cpp) | monitor | Stream decoded feedback over PDO, power stage off |
| [`enable_and_hold`](../examples/enable_and_hold.cpp) | CSP | Enable and hold the current position, print feedback |
| [`csp_position`](../examples/csp_position.cpp) | CSP | Hold `start + offset` counts |
| [`csv_velocity`](../examples/csv_velocity.cpp) | CSV | Stream a constant target velocity |
| [`cst_torque`](../examples/cst_torque.cpp) | CST | Stream a constant target torque |
| [`pp_move`](../examples/pp_move.cpp) | PP | One relative profile-position move (drive runs its own trajectory) |
| [`pv_velocity`](../examples/pv_velocity.cpp) | PV | Target velocity via the drive's accel/decel ramp |
| [`pt_torque`](../examples/pt_torque.cpp) | PT | Target torque via the drive's torque slope |
| [`multi_drive`](../examples/multi_drive.cpp) | multi | Two drives on one shared bus, with a jitter readout |

```bash
build/examples/pdo_feedback_monitor --can can0 --node 1 --seconds 10
build/examples/enable_and_hold      --can can0 --node 1 --seconds 10
build/examples/csp_position         --can can0 --node 1 --amplitude 0 --seconds 5
build/examples/csv_velocity         --can can0 --node 1 --velocity 0 --seconds 5
build/examples/cst_torque           --can can0 --node 1 --torque 0 --seconds 5
build/examples/pp_move              --can can0 --node 1 --offset 0 --profile-velocity 50000 --seconds 5
build/examples/pv_velocity          --can can0 --node 1 --velocity 0 --seconds 5
build/examples/pt_torque            --can can0 --node 1 --torque 0 --torque-slope 1000 --seconds 5
build/examples/multi_drive          --can can0 --nodes 1,2 --seconds 10   # add --enable to energise + hold
```

## Real-time loop (latency / jitter)

The bus loop thread can be tuned for deterministic cadence. It is opt-in and
shared by all drives on an interface (`MotorConfig::rt`, or the `--rt*` CLI
flags). When enabled the thread is moved to `SCHED_FIFO`, optionally pinned to
one CPU, and process memory is locked (`mlockall`):

```cpp
config.rt.enabled = true;     // SCHED_FIFO
config.rt.priority = 80;      // 1..99
config.rt.cpu = 2;            // pin to a core (-1 = unpinned); pair with isolcpus
config.rt.lock_memory = true; // mlockall to keep the cyclic path off the pager
config.sync_period_us = 1000; // nominal period for jitter telemetry (match the DCF SYNC)
```

This needs privileges. Without them the loop logs a warning and keeps running at
normal priority (no hard failure). Grant them per user via
`/etc/security/limits.conf`:

```
<user>  -  rtprio   99
<user>  -  memlock  unlimited
```

or run with `CAP_SYS_NICE` / `sudo`. For the lowest jitter, keep other work off
the chosen core (boot with `isolcpus=<cpu>`) and read the achieved cadence from
`cyclicStats()` (CLI `--stats`): interval min/max/mean and worst-case jitter vs
`sync_period_us`.

### Bus bandwidth

At 1 Mbit/s classic CAN, two drives at a 1 ms SYNC with the cyclic superset
layout are already close to the practical bus limit (SYNC + 4 feedback TPDOs +
up to 4 command RPDOs per millisecond, before SDO traffic). If SocketCAN reports
`CAN transmit queue full`, bring the interface up with `canup.sh` (sets
`txqueuelen 1000`), then slow the profile (`master.sync_period`, e.g. `2000` us)
or reduce mapped PDO traffic before adding nodes.

## Linting & formatting

In-editor diagnostics, completion, and go-to come from **clangd**, which reads
`build/compile_commands.json` plus the repo configs (`.clangd`, `.clang-tidy`,
`.clang-format`). Static analysis is **clang-tidy**; formatting is
**clang-format**.

```bash
# Tools (user-space; or apt install clangd clang-tidy clang-format):
pip install --user clangd clang-tidy clang-format
cmake --preset default        # creates build/compile_commands.json

tools/lint.sh format          # rewrite files in place
tools/lint.sh format-check    # fail if anything is unformatted
tools/lint.sh tidy            # clang-tidy static analysis
tools/lint.sh                 # format-check + tidy
```

Note: pip-installed clang defaults to a gcc toolchain dir without libstdc++
headers, so `.clangd` and `tools/lint.sh` pin it to gcc-11
(`--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11`). Adjust for your system
(`ls -d /usr/include/c++/*`).

## Repository layout

- `include/stablecops/app/` + `src/app/`: the public `MotorDrive` handle, the
  shared `Bus`, the `CanopenApplication` loop owner, and real-time tuning.
- `include/stablecops/ds402/` + `src/ds402/`: transport-independent DS402 state
  decoding, feedback types, and the `DriveController` object facade.
- `include/stablecops/lely/` + `src/lely/`: the Lely adapter (`MotorDriver`) that
  drives the cyclic path; the only place Lely is used.
- `include/stablecops/config/` + `src/config/`: the generated PDO-summary loader.
- `include/stablecops/log/` + `src/log/`: the pluggable logging facade.
- `src/tools/`: `stablecops_master` and `stablecops_commissiond`.
- `examples/`: one small program per mode (table above).
- `eds/EDS files/`: immutable vendor EDS files.
- `config/motors/`: declarative motor profiles.
- `tools/generate_canopen_config.py`: EDS normalization, dcfgen YAML, DCF
  generation, and validation.
- `generated/canopen/`: derived EDS/YAML/summary artifacts.
- `dcf/master.dcf`: generated runtime DCF.
