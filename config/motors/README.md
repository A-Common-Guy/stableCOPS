# CANopen Motor Profiles

Motor profiles describe how a vendor EDS becomes a bootable Lely DCF without
editing the vendor file.

Required fields:

- `name`: stable profile name used for generated artifact names.
- `vendor_eds`: immutable vendor EDS path, relative to the repository root.
- `master.node_id`: CANopen node ID for the Lely master.
- `node.node_id`: CANopen node ID for the drive.
- `identity_policy`: `strict` keeps EDS identity defaults; `ignore` clears
  identity defaults in the normalized copy so live identity can differ.
- `pdo_policy`: `vendor-default` imports the PDO defaults from the EDS.
- `mode_policy`: `vendor-default` avoids forcing an operation mode during boot.

Generated artifacts are written under `generation.generated_dir`; the runtime
DCF is written under `generation.dcf_dir`.

Generate the default PHU configuration with:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```
