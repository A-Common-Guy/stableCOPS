# src/lely

Lely-specific CANopen adapter sources belong here.

This layer should own:

- Lely event loop setup.
- SocketCAN channel binding.
- `AsyncMaster` lifecycle.
- `FiberDriver` or equivalent node-driver callbacks.
- DS402 object access forwarding into `stablecops::ds402`.

Keep CiA 402 state-machine logic out of this directory; it belongs in `src/ds402`.
Keep vendor-specific PDO remapping and operation-mode policy out of this
directory; those belong in motor profiles and generated DCF artifacts.
