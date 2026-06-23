# CANopen Motor Pipeline

The motor pipeline has one rule: vendor data goes in, generated artifacts come
out. Runtime code stays generic unless a drive really needs vendor-specific
behavior.

## Onboard A Motor

1. Copy the vendor EDS into `eds/EDS files/` and leave it unchanged.
2. Add a profile under `config/motors/`.
3. Generate artifacts:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

4. Bring up SocketCAN:

```bash
sudo ./canup.sh
```

5. Boot and inspect:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --inspect --run
```

6. Enable motion commands only after the live identity, DS402 state, supported
   modes, and PDO summary match expectations.

## Enable And Hold

The first useful action after inspection is a safe DS402 state transition. This
primes the CSP target to the current position, sends `shutdown`, `switch on`,
and `enable operation`, then checks that the drive reaches operation enabled.

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --enable --run
```

For CSP bring-up, prefer holding the current position first:

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --hold-position --run
```

Only after that works, request small explicit CSP target changes. The target is
rejected if the requested step exceeds `--max-position-step`.

```bash
build/stablecops_master --can can0 --dcf dcf/master.dcf --master-node 127 --node 1 --csp-relative 1000 --max-position-step 1000 --run
```

Available boot actions:

- `--enable`: run the DS402 safe enable sequence.
- `--hold-position`: enable and keep the CSP target at the current position.
- `--csp-target counts`: enable and request an absolute CSP target.
- `--csp-relative counts`: enable and request a relative CSP target.
- `--max-position-step counts`: limit the allowed target delta.

## Profile Format

Profiles are small YAML files. The default PHU profile is
`config/motors/eyou_phu.yml`.

Required fields:

- `name`: stable name used for generated artifact filenames.
- `vendor_eds`: vendor EDS path relative to the repository root.
- `master.node_id`: Lely master CANopen node ID.
- `node.node_id`: drive CANopen node ID.
- `identity_policy`: `strict` or `ignore`.
- `pdo_policy`: start with `vendor-default`.
- `mode_policy`: start with `vendor-default`.

Generated artifacts are written to `generation.generated_dir`; the runtime DCF
is written to `generation.dcf_dir`.

## Generated Files

For the PHU profile, generation writes:

- `generated/canopen/eyou_phu/eyou_phu.normalized.eds`
- `generated/canopen/eyou_phu/eyou_phu.dcfgen.yml`
- `generated/canopen/eyou_phu/eyou_phu.summary.json`
- `dcf/master.dcf`

The normalized EDS, generated YAML, summary, and DCF are derived artifacts. Do
not hand-edit them; update the vendor EDS/profile and regenerate.

## Inspection

`--inspect` performs read-only SDO diagnostics after boot. It reads:

- `0x1018` identity
- `0x6502` supported modes
- `0x6060` commanded mode
- `0x6061` displayed mode
- `0x6041` statusword and decoded DS402 state
- position, velocity, torque, and error code feedback

## Runtime Contract

The generated DCF and summary own profile-specific PDO knowledge. Runtime C++
code does not hardcode RPDO/TPDO map indexes: at start-up the runtime loads the
generated `*.summary.json` (`--summary`, defaulting to the euservo_rp profile)
into a `stablecops::config::PdoMap`, and `OnConfig` programs the drive directly
from that map. Pick a different motor at runtime by pointing `--dcf`/`--summary`
at its generated artifacts.

`stablecops::ds402::DriveController` works in DS402 object terms: controlword,
statusword, operation mode, target position, velocity, and torque. SDO remains
the fallback for configuration and diagnostics.

### Command image and cyclic streaming

The drive groups command objects into each RPDO. The EYou firmware **rejects**
the EDS-default RxPDO1 (`0x6040` controlword + `0x60FF` target velocity +
`0x6060` modes) during CSP bring-up: even with a correct controlword on the
wire, the drive stays in *ready to switch on* and ignores every controlword
(including `0x80` disable-voltage), which means it is discarding the whole
frame. The vendor's proven CSP recipe (manual Table 5-5 / 3.6.1) keeps the mode
object out of the cyclic stream and uses:

- **RPDO1** = `0x6040` controlword + `0x607A` target position
- RPDO2 / RPDO3: disabled (CSP only needs the target position, now in RPDO1)
- **TPDO1** = `0x6041` statusword + `0x6061` mode display + `0x6077` torque
- **TPDO2** = `0x6064` position + `0x606C` velocity

This is the layout the generator emits (`eds_overrides` in the profile), records
in `*.summary.json`, and that `OnConfig` reads back from the summary (via
`config::PdoMap`) to program on the drive. Because a PDO is transmitted as a whole
frame, the master must never update one mapped object in isolation: doing so
would send stale or zero values for that object's PDO neighbours.

`MotorDriver` therefore keeps a single `CommandImage` and emits the *entire*
RPDO1 image (controlword + target position) on every `OnSync`. Writes to other
command objects update the image but are not streamed (they are not PDO-mapped
in CSP). Feedback objects (`0x6041`, `0x6061`, `0x6064`, `0x606C`, `0x6077`)
are cached from the received TPDOs in `OnSync` and returned without bus traffic.
SDO is used only for configuration, identity, error code, and the one-time image
seed at enable. The drive rejects SDO downloads to the command objects (vendor
abort `0x00000002`); those must go over PDO at runtime.

### Drive configuration (pre-operational)

The PHU/RP drives ship with every PDO set to transmission type `0`
(event-driven), so out of NVM they never stream TPDOs on SYNC and never run the
cyclic RPDO exchange. PDO parameters are only reliably SDO-writable while the
node is pre-operational. The vendor bring-up recipe (manual Table 5-5) therefore
configures the drive *before* it is started.

`MotorDriver::OnConfig` runs during the NMT boot "update configuration" step,
which executes while the node is still pre-operational. When a motion action is
requested (so `--inspect` stays read-only) it pushes, over SDO, for each active
PDO (RPDO1, TPDO1, TPDO2): disable the PDO (set the COB-ID valid bit), set the
transmission type to `1` (cyclic synchronous), rewrite the mapping from scratch,
then re-enable it. RPDO2 and RPDO3 are explicitly disabled (COB-ID valid bit)
because the master no longer transmits them in the CSP layout.

The COB-IDs and mappings written match `dcf/master.dcf` exactly (both are driven
from the profile's `eds_overrides`), so both ends agree. Rewriting the whole
mapping (not just the transmission type) also clears the stale/oversized mapping
the drive otherwise reports straight from NVM. If configuration fails, the boot
is aborted with the SDO abort code rather than starting a node that cannot
communicate.

The mode (`0x6060`) is deliberately **not** written here, and it is kept out of
the cyclic RxPDO. On this firmware the DS402 command objects (`0x6040`,
`0x6060`, `0x607A`, ...) are PDO-only: SDO downloads abort with the vendor code
`0x00000002`. The drive's persisted mode is already `8` (CSP), so it does not
need to be (re)written; streaming `0x6060` cyclically while the drive is not
enabled is exactly what makes the firmware reject RxPDO1.

### Enable sequence

`enableDrive` seeds the command image from the live drive (mode, actual
position, profile parameters), starts cyclic streaming, waits a few SYNC cycles,
pulses a fault reset (controlword bit 7, per the vendor recipe), then walks
`shutdown -> switch on -> enable operation`, confirming each transition from
cached statusword feedback. While bringing CSP up, the commanded target tracks
the measured position so the drive never sees a step at enable; once operation
enabled is reached, the target is frozen (held).

The enable path refuses to proceed on fault states, nonzero drive error codes,
transition timeouts, a missing SYNC stream, non-CSP target commands, or target
steps larger than the configured guard.

Add explicit PDO remapping to a profile only when the vendor default layout is
insufficient and the drive documentation confirms the remap sequence.

## Wire-Level Verification

When a transition stalls, confirm what is actually on the bus before changing
code. The command frames and feedback frames are the ground truth.

Watch the command RPDO the master sends (`0x201`), the feedback TPDOs the drive
sends (`0x181/0x281`), and SYNC (`0x080`). In the CSP layout the master no
longer transmits `0x301`/`0x401`:

```bash
candump -tz can0,201:7FF can0,181:7FF can0,281:7FF can0,080:7FF
```

What healthy bring-up looks like:

- `0x080` SYNC frames arrive at the configured cycle (1 ms for the PHU profile).
- `0x201` is a 6-byte frame: controlword (bytes 0-1) walking `06 00 -> 07 00 ->
  0F 00`, followed by the 4-byte target position (bytes 2-5) tracking the
  measured position during bring-up, then frozen.
- `0x181` statusword tracks the transitions (`31 06` ready to switch on ->
  `33 06` switched on -> `37 06` operation enabled), error code stays `0`.

The drive accepts SDO writes to **PDO communication/mapping** objects only while
pre-operational (this is what `OnConfig` relies on). The DS402 **command**
objects (`0x6040`, `0x6060`, `0x607A`, ...) are PDO-only and abort with
`0x00000002` in any state, which is why they go over PDO:

```bash
# SDO download 0x6060 = 8; aborts (581 ... 80) with abort code 0x00000002
# regardless of NMT state -- command objects are PDO-only on this firmware
cansend can0 601#2F60600008
candump -tz can0,581:7FF
```

After a successful bring-up, re-running `--inspect` should now report every
active PDO with `transmission type 0x01` and the expected mapped-object counts.

