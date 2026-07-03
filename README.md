# stableCOPS

A small C++17 library for driving CANopen / CiA-402 servo motors over SocketCAN,
built on Lely CANopen. It hides the CANopen state machine, cyclic PDO exchange,
faults, and shutdown behind one thread-safe handle: **`MotorDrive`**.

- **Development, CLI tools, examples, tuning:** [`docs/development.md`](docs/development.md)
- **EDS -> DCF data pipeline:** [`docs/canopen_motor_pipeline.md`](docs/canopen_motor_pipeline.md)

## Install

```bash
sudo apt-get install pkg-config liblely-coapp-dev liblely-co-tools python3-dcf-tools
cmake --preset default
cmake --build --preset default
cmake --install build --prefix /your/prefix     # sudo for a system prefix
```

Consume it from CMake (no Lely or pkg-config needed on the consumer side - the
public API is dependency-clean):

```cmake
find_package(stableCOPS REQUIRED)
target_link_libraries(myapp PRIVATE stableCOPS::stablecops)
```

At runtime the target needs `liblely-coapp` installed (recorded as a `NEEDED`
dependency of `libstablecops.so`), SocketCAN up (`sudo ./canup.sh`), and CAN
privileges (root or `CAP_NET_ADMIN`).

## The interface: `MotorDrive`

You only ever hold `MotorDrive` objects. Each names a CAN interface and a node
id; drives on the **same** interface transparently share one bus (one master,
one loop thread, one SYNC). Construct all drives for a chain, then `start()`.

```cpp
#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/ds402/State.hpp"

using namespace stablecops;

app::MotorConfig config;                 // see fields below
config.can_interface = "can0";
config.node_id = 1;
config.operation_mode = ds402::OperationMode::CyclicSynchronousPosition;
config.enable_on_boot = true;            // energise + hold current position at boot

app::MotorDrive drive(config);
drive.start();                           // boots the chain; throws on failure

while (drive.feedbackLive()) {           // false once the drive stops talking
    auto fb = drive.feedback();          // thread-safe snapshot
    // fb.state / fb.position / fb.velocity / fb.torque / fb.error_code
    drive.commandPosition(fb.position);  // stream a CSP setpoint

    if (drive.faulted()) {
        drive.resetFault();              // clear + recover to the configured state
    }
}

drive.quickStop();                       // controlled ramp-down (holds energised)
drive.stop();                            // graceful de-energise (coasts)
```

### API at a glance

- **Lifecycle:** `start()`, `stop()` (coast), `quickStop()` (ramp + hold),
  `running()`.
- **Telemetry (any thread):** `feedback()`, `feedbackLive()`,
  `positionDegrees()` / `positionRadians()`, `cyclicStats()`.
- **Status / faults:** `enabled()`, `faulted()`, `errorCode()`, `resetFault()`.
- **Enable / mode:** `enableOperation(hold)`, `setOperationMode(mode)`
  (confirmed against 0x6061).
- **Cyclic setpoints:** `commandPosition()` (CSP), `commandVelocity()` (CSV/PV),
  `commandTorque()` (CST/PT).
- **Profile move:** `moveToPosition(counts, relative)` (PP; drive runs the ramp).
- **Homing:** `startHoming(config)`, `homingPhase()`, `homingResult()`.
- **Objects:** `readObject()` / `writeObject()` for arbitrary SDO/PDO objects.

### Key `MotorConfig` fields

- `can_interface`, `node_id` - which bus and drive.
- `master_dcf_path`, `summary_path` - generated artifacts, loaded **by path** at
  boot (point these at your own copies; installed samples live under
  `share/stablecops/`).
- `operation_mode` - CSP/CSV/CST or PP/PV/PT.
- `enable_on_boot` / `hold_position_on_boot` / `monitor_on_boot`.
- `profile_velocity` / `profile_acceleration` / `profile_deceleration` /
  `torque_slope` - profile-mode ramps.
- `counts_per_rev` - scaling for `positionDegrees()`/`positionRadians()`.
- `feedback_timeout` - staleness watchdog window (0 disables).
- `sync_period_us`, `rt` - cyclic period and optional real-time tuning.

## Safety behaviour

- **Feedback watchdog:** while energised, if no cyclic feedback arrives for
  `feedback_timeout` (default 100 ms) the power stage is dropped and
  `feedbackLive()` goes false.
- **Fault detection on three channels:** DS402 statusword / `0x603F` in the
  cyclic TPDO, the drive's EMCY messages (`emergency_error_code` /
  `error_register`), and node loss via the master's consumer heartbeat
  (`node_alive`, ~300 ms). `resetFault()` clears and recovers.
- **Stopping:** `stop()` de-energises (the joint coasts). For a loaded or
  vertical axis prefer `quickStop()`, which decelerates on the drive's quick-stop
  ramp and holds energised. Neither tears down the shared bus.
- **Position units:** `feedback().position` is the raw `0x6064` count; the
  degree/radian helpers scale it by `counts_per_rev`.

## Logging

The library never writes to stdout/stderr directly - it routes through
`stablecops::log`. By default whole lines go to stderr (errors/warnings) and
stdout (info); install a sink to redirect, prefix, or silence, and set a minimum
level:

```cpp
#include "stablecops/log/Log.hpp"
stablecops::log::setLevel(stablecops::log::Level::Warn);
stablecops::log::setSink([](stablecops::log::Level level, const std::string& line) {
    myLogger.log(stablecops::log::toString(level), line);
});
```

## Operator tools

Besides the library there are two command-line front-ends - a bring-up/diagnostic
tool (`stablecops_master`) and a browser commissioning UI
(`stablecops_commissiond`). See [`docs/development.md`](docs/development.md).
