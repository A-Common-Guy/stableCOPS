# config

Local CANopen network notes and motor profiles belong here.

Motor profiles live under `config/motors/` and describe:

- immutable vendor EDS path
- master and motor node IDs
- identity-check policy
- PDO policy
- operation-mode policy

Generate the default PHU artifacts with:

```bash
python3 tools/generate_canopen_config.py --profile config/motors/eyou_phu.yml
```
