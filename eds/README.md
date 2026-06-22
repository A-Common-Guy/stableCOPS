# eds

Place immutable vendor EDS files here, preferably under `EDS files/`.

Do not hand-edit vendor EDS files. If Lely needs a normalized copy, generate it
with `tools/generate_canopen_config.py` and keep the derived file under
`generated/canopen/`.

Current PHU setup:

- `EDS files/EYOU_ServoModule（PHU）.eds`: official vendor EDS, kept unchanged.

The generated PHU normalized EDS is written to
`generated/canopen/eyou_phu/eyou_phu.normalized.eds`.
