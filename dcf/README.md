# dcf

Generated runtime DCF files live here.

Current generated files:

- `master.dcf`: Lely master DCF for one EYou PHU joint at node 1, using the
  vendor-default PHU PDO layout from the official EDS.

The default file is generated from `config/motors/eyou_phu.yml`:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```

The original vendor EDS is kept unchanged under `eds/EDS files/`. The normalized
copy, generated dcfgen YAML, and PDO summary live under `generated/canopen/`.
