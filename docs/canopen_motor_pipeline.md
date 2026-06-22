# Generic CANopen Motor Pipeline

This project treats a motor integration as data first:

1. Keep the vendor EDS unchanged under `eds/EDS files/`.
2. Add or update a small profile under `config/motors/`.
3. Generate the normalized EDS, dcfgen YAML, PDO summary, and `dcf/master.dcf`.
4. Boot the drive with the generated DCF.
5. Inspect live identity, state, supported modes, and feedback.
6. Enable motion commands only after the profile and live drive agree.

## Generate Artifacts

The default PHU profile is `config/motors/eyou_phu.yml`.

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

Generated files:

- `generated/canopen/eyou_phu/eyou_phu.normalized.eds`
- `generated/canopen/eyou_phu/eyou_phu.dcfgen.yml`
- `generated/canopen/eyou_phu/eyou_phu.summary.json`
- `dcf/master.dcf`

The normalized EDS is derived from the vendor EDS. Do not edit it by hand.

## Inspect A Drive

Bring up SocketCAN first:

```bash
sudo ./canup.sh
```

Then boot and inspect the drive:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --inspect --run
```

Inspection reads generic CANopen/DS402 objects:

- `0x1018` identity
- `0x6502` supported modes
- `0x6060` commanded mode
- `0x6061` displayed mode
- `0x6041` statusword and decoded DS402 state
- position, velocity, torque, and error code feedback

## Runtime Contract

The generated DCF and summary own profile-specific PDO knowledge. Runtime C++
code should not hardcode RPDO/TPDO map indexes or vendor-specific boot writes.

`stablecops::ds402::DriveController` works in object terms such as controlword,
statusword, operation mode, target position, velocity, and torque. Lely uses the
generated DCF for the CANopen network description, while SDO remains the generic
fallback for inspection and diagnostics.

For a new motor, start with `pdo_policy: vendor-default`. Add explicit remapping
to the profile only when the vendor default layout is insufficient and the drive
documentation confirms the remap sequence.
