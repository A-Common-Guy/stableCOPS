# stableCOPS
## Stable CANopen Motor Pipeline

Lely CANopen project scaffold for SocketCAN motors with a small generic CiA 402
drive-profile layer.

This repository keeps the existing CAN bring-up scripts and prepares a C++
CANopen master based on Lely CANopen.

Lely provides the CANopen application framework, event loop, SDO/PDO machinery,
and DCF tooling. It does not provide a complete standalone CiA 402 motor driver,
so this project keeps a small local DS402 layer and uses Lely as the CANopen
transport/runtime.

## Dependencies

Install Lely CANopen development packages and tools:

```bash
sudo apt-get update
sudo apt-get install pkg-config liblely-coapp-dev liblely-co-tools python3-dcf-tools
```

Lely's C++ tutorial links applications with `pkg-config --cflags --libs
liblely-coapp`, and this project follows that model through CMake.

Reference docs:

- [Lely CANopen](https://opensource.lely.com/canopen/)
- [Lely CANopen C++ tutorial](https://opensource.lely.com/canopen/docs/cpp-tutorial/)
- [Lely CANopen installation](https://opensource.lely.com/canopen/docs/installation/)

## Build Check

This builds the DS402 core, the Lely adapter, and the master executable.

```bash
cmake --preset default
cmake --build --preset default
```

Print configuration without opening CAN:

```bash
build/stablecops_master --can can0
```

Start the Lely master with the generated DCF:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --run
```

Boot the drive and inspect generic CANopen/DS402 objects:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --inspect --run
```

## CAN Interface

Bring up the existing `can0` interface:

```bash
sudo ./canup.sh
```

Bring it down:

```bash
sudo ./candown.sh
```

## Layout

- `canup.sh`, `candown.sh`: SocketCAN interface scripts kept from the original repo.
- `include/stablecops/ds402/`: CiA 402 constants, state decoding, and drive controller API.
- `src/ds402/`: transport-independent CiA 402 implementation.
- `include/stablecops/lely/`: Lely motor driver adapter.
- `src/lely/`: `FiberDriver` to DS402 object-access bridge.
- `src/app/`: Lely runtime ownership and application orchestration.
- `src/tools/`: command-line entry points.
- `eds/EDS files/`: immutable vendor EDS files.
- `config/motors/`: small motor profiles used by the generator.
- `generated/canopen/`: normalized EDS files, generated dcfgen YAML, and PDO summaries.
- `dcf/`: generated runtime DCF files.
- `docs/canopen_motor_pipeline.md`: one-motor onboarding workflow.

## Generated CANopen Configuration

The current hardware target is an EYou PHU servo module. The official vendor EDS
is kept unchanged, and generated artifacts are derived from a profile:

- `eds/EDS files/EYOU_ServoModule（PHU）.eds`: official vendor PHU EDS.
- `config/motors/eyou_phu.yml`: profile for one PHU motor at node 1.
- `generated/canopen/eyou_phu/eyou_phu.normalized.eds`: normalized PHU EDS for Lely tooling.
- `generated/canopen/eyou_phu/eyou_phu.summary.json`: profile-derived PDO summary.
- `dcf/master.dcf`: generated Lely master DCF with master node ID 127.

Regenerate the default DCF from the repository root:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

## Next Setup Steps

1. Confirm boot with the generated PHU DCF.
2. Inspect live identity, supported modes, and DS402 state with `--inspect`.
3. Add high-level commands on top of `stablecops::ds402::DriveController`.
4. Add explicit profile PDO remapping only when vendor defaults are insufficient.
5. Add MIT scaling/packing once the vendor object definitions for `0x2130..0x2133`
   are pinned down.
