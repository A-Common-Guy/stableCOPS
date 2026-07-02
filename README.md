# stableCOPS

Generic CANopen motor bring-up scaffold built on Lely CANopen and a small
transport-independent DS402 layer.

The intended flow is data-driven: keep vendor EDS files unchanged, describe the
motor in a profile, generate Lely artifacts, then boot and inspect the drive
before enabling motion commands. See `docs/canopen_motor_pipeline.md` for the
full workflow.

## Requirements

```bash
sudo apt-get update
sudo apt-get install pkg-config liblely-coapp-dev liblely-co-tools python3-dcf-tools
```

## Build

```bash
cmake --preset default
cmake --build --preset default
```

## Generate CANopen Artifacts

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

This derives a normalized EDS, dcfgen YAML, PDO summary, and `dcf/master.dcf`
from the immutable vendor EDS.

## Run

Bring up CAN:

```bash
sudo ./canup.sh
```

Print configuration without opening CAN:

```bash
build/stablecops_master --can can0
```

Boot the drive:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --run
```

Boot and inspect live CANopen/DS402 objects:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --inspect --run
```

Receive cyclic PDO feedback without energising the drive (monitor mode):

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --monitor --run
```

Safely enable the DS402 power stage:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --enable --run
```

Start the browser-based commissioning UI. This boots the same runtime in monitor
mode, configures cyclic PDO feedback, and serves a local web page for status,
typed object reads/writes, CiA402 enable/stop/fault-reset, setpoints, and encoder
feedback:

```bash
build/stablecops_commissiond --can can0 --dcf dcf/master.dcf --nodes 1,2 --mode csp
# open http://127.0.0.1:8765/
```

Use `--mode pp|pv|pt|csv|cst` when commissioning a matching profile/cyclic
mode. On the EYou RP firmware, controlword and target objects are driven through
the PDO/CiA402 path once mapped, so use the UI motion buttons for movement and
the object panel for parameters or diagnostics. The UI has a separate **Send
Mode (0x6060)** action; use it before enabling or before sending the first
setpoint, then confirm the displayed mode in the status panel. When several
nodes are passed, the daemon boots them on one shared bus/SYNC and the browser's
target-node selector chooses which drive receives commands.

Enable and hold the current CSP position:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --hold-position --run
```

Select the operation mode at boot. CSP/CSV/CST share one fixed PDO layout; the
mode (and any profile parameters) is chosen over SDO while pre-operational.
`--mode` accepts `csp|csv|cst` (cyclic) and `pp|pv|pt` (profile):

```bash
# Cyclic synchronous velocity:
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --mode csv --enable --run

# Profile velocity with a configured accel/decel ramp:
build/stablecops_master --can can0 --node 1 --mode pv --profile-accel 200000 --profile-decel 200000 --enable --run
```

Command a guarded CSP step only after hold/enable has been verified:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --csp-relative 1000 --max-position-step 1000 --run
```

Drive several nodes on one chain (one master, one shared SYNC) and print the
achieved cyclic cadence/jitter, optionally on a real-time loop pinned to a core:

```bash
# Two joints, monitor only, with a per-second jitter readout:
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --run

# Same, but on a SCHED_FIFO loop pinned to CPU 2 (needs privileges, see below):
build/stablecops_master --can can0 --nodes 1,2 --monitor --stats --rt --rt-cpu 2 --run
```

## Library API (MotorDrive)

`stablecops::app::MotorDrive` is a thread-safe handle that runs the Lely event
loop on its own thread, so an application can read feedback and issue setpoints
from its own thread without touching Lely internals:

```cpp
stablecops::app::MotorConfig config;
config.monitor_on_boot = true;          // or enable_on_boot / hold_position_on_boot
config.feedback_timeout = std::chrono::milliseconds{100};  // 0 disables the watchdog
stablecops::app::MotorDrive drive(config);
drive.start();
while (drive.feedbackLive()) {           // false if the drive stops talking (stale)
    auto fb = drive.feedback();          // thread-safe snapshot
    // ... use fb.position / fb.velocity / fb.torque / fb.state ...
    double angle = drive.positionDegrees();   // 0x6064 converted via counts_per_rev
}
drive.commandPosition(counts);          // posted to the loop thread (when enabled in CSP)
// In a cyclic mode chosen via config.operation_mode:
drive.commandVelocity(units);           // CSV / PV
drive.commandTorque(units);             // CST / PT
drive.moveToPosition(counts, relative); // PP (drive runs its own trajectory)

// Status + fault handling:
if (drive.faulted()) {
    drive.resetFault();                  // clear + recover to the configured state
}
bool ok = drive.enabled();               // true only with live feedback
drive.stop();                            // graceful de-energise of this node
```

Multiple drives & one shared bus:

You only ever hold `MotorDrive` objects. Each names a CAN interface and a node
id. Drives that name the **same** interface transparently share one hidden,
ref-counted bus (one Lely master, one loop thread, one SYNC); different
interfaces get fully independent buses (own loop thread + SYNC), so several
chains are just `MotorDrive`s with different interface names.

```cpp
// Two joints on can0 share a bus; a third on can1 gets its own bus + thread.
stablecops::app::MotorConfig c1; c1.can_interface = "can0"; c1.node_id = 1;
stablecops::app::MotorConfig c2; c2.can_interface = "can0"; c2.node_id = 2;
stablecops::app::MotorConfig k1; k1.can_interface = "can1"; k1.node_id = 1;
stablecops::app::MotorDrive j1(c1), j2(c2), other(k1);

j1.start();        // boots the whole can0 chain (j1 + j2) once
other.start();     // boots can1 independently
auto stats = j1.cyclicStats();   // measured cadence/jitter of the can0 bus
```

Lifecycle is static: construct all drives for a chain first, then `start()` any
one of them (siblings' `start()` are no-ops). The last `MotorDrive` released on
an interface tears that bus down (graceful stop + join). Bus-level config
(`can_interface`, `master_dcf_path`, `summary_path`, `master_node_id`,
`sync_period_us`, `rt`) must match across all drives on one interface.

Real-time loop (latency / jitter):

The bus loop thread can be tuned for deterministic cadence. It is opt-in and
shared by all drives on the interface (`MotorConfig::rt`, or the `--rt*` CLI
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
normal priority (no hard failure). Grant them per user via `/etc/security/limits.conf`:

```
<user>  -  rtprio   99
<user>  -  memlock  unlimited
```

or run the binary with `CAP_SYS_NICE` / via `sudo`. For the lowest jitter, keep
other work off the chosen core (boot with `isolcpus=<cpu>`) and read back the
achieved cadence from `cyclicStats()` (CLI: `--stats`) - it reports interval
min/max/mean and worst-case jitter vs `sync_period_us`.

Safety behaviour:

- **Feedback watchdog**: while the drive is energised, if no cyclic feedback
  arrives for `feedback_timeout` (default 100 ms) the power stage is dropped
  (graceful stop) and `feedbackLive()` goes false. Set `feedback_timeout` to 0 to
  disable. On the CLI: `--feedback-timeout ms`.
- **Fault handling**: faults are logged with statusword/error code as they occur.
  `faulted()` / `enabled()` / `errorCode()` report drive status, and `resetFault()`
  clears a latched fault and recovers to the configured operating state without a
  restart.
- **Position units**: `feedback().position` is the raw `0x6064` count (output-shaft
  referenced, seeded from the absolute encoder at power-on). `positionDegrees()` /
  `positionRadians()` convert it via `config.counts_per_rev` (default 524288 =
  `0x6091:02`). Calibrate by rotating the output exactly one turn and setting
  `counts_per_rev` to the observed count delta (CLI: `--counts-per-rev n`).

## Examples

Built under `build/examples/`:

```bash
# Stream decoded feedback from the drive over PDO, power stage stays off:
build/examples/pdo_feedback_monitor --can can0 --dcf dcf/master.dcf --node 1 --seconds 10

# Enable + hold the current CSP position while printing live feedback:
build/examples/enable_and_hold --can can0 --dcf dcf/master.dcf --node 1 --seconds 10
```

One simple example per cyclic mode (defaults command 0 = no motion; SPINS the
motor if a nonzero setpoint is passed):

```bash
# CSP: hold (start + offset) counts
build/examples/csp_position --can can0 --node 1 --offset 0 --seconds 5

# CSV: stream a constant target velocity
build/examples/csv_velocity --can can0 --node 1 --velocity 0 --seconds 5

# CST: stream a constant target torque
build/examples/cst_torque  --can can0 --node 1 --torque 0 --seconds 5
```

Profile modes (the drive runs its own trajectory/ramp; defaults are no motion):

```bash
# PP: one relative profile-position move (needs a profile velocity to move)
build/examples/pp_move     --can can0 --node 1 --offset 0 --profile-velocity 50000 --seconds 5

# PV: target velocity reached via the drive's accel/decel ramp
build/examples/pv_velocity --can can0 --node 1 --velocity 0 --seconds 5

# PT: target torque reached via the drive's torque slope
build/examples/pt_torque   --can can0 --node 1 --torque 0 --torque-slope 1000 --seconds 5
```

Multiple drives on one shared bus, with a cyclic jitter readout (add `--enable`
to energise + hold both joints; `--rt --rt-cpu N` for a real-time loop):

```bash
build/examples/multi_drive --can can0 --nodes 1,2 --seconds 10
```

## Linting & formatting

In-editor diagnostics, completion, and go-to come from **clangd**, which reads
`build/compile_commands.json` plus the repo configs (`.clangd`, `.clang-tidy`,
`.clang-format`). Static analysis is **clang-tidy**; formatting is
**clang-format**.

One-time setup:

```bash
# 1. Tools (user-space; no sudo needed). Or apt install clangd clang-tidy clang-format.
pip install --user clangd clang-tidy clang-format

# 2. Generate the compile database clangd needs.
cmake --preset default        # creates build/compile_commands.json (symlinked at repo root)

# 3. In Cursor/VS Code: install the "clangd" extension (llvm-vs-code-extensions.vscode-clangd)
#    and disable the Microsoft C/C++ IntelliSense engine so they don't fight.
```

Command-line usage (wraps the gcc-toolchain pin for you):

```bash
tools/lint.sh format        # rewrite files in place
tools/lint.sh format-check  # fail if anything is unformatted
tools/lint.sh tidy          # run clang-tidy static analysis
tools/lint.sh               # format-check + tidy

# Or invoke the tools directly:
clang-format -i src/ds402/State.cpp
clang-tidy -p build src/ds402/State.cpp
```

Note: the pip-installed clang defaults to a gcc toolchain dir without libstdc++
headers, so `.clangd` and `tools/lint.sh` pin it to gcc-11
(`--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11`). Adjust that path if your
libstdc++ lives elsewhere (`ls -d /usr/include/c++/*`).

## Layout

- `eds/EDS files/`: immutable vendor EDS files.
- `config/motors/`: declarative motor profiles.
- `tools/generate_canopen_config.py`: EDS normalization, dcfgen YAML emission, DCF generation, and validation.
- `generated/canopen/`: derived EDS/YAML/summary artifacts.
- `dcf/master.dcf`: generated runtime DCF.
- `include/stablecops/ds402/` and `src/ds402/`: generic DS402 objects, state decoding, and commands.
- `include/stablecops/lely/` and `src/lely/`: Lely adapter and boot callbacks.
- `src/tools/stablecops_commissiond.cpp`: single-binary browser commissioning UI
  built on `MotorDrive`.
