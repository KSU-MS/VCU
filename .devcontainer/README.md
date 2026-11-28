# Dev container

The same environment as `nix develop`, for people without Nix. The image is
built from `flake.nix` (`packages.devcontainer`), so there's one definition
of the toolchain, and CI publishes it to `ghcr.io/ksu-ms/vcu-dev` from `main`.

**VS Code:** install the Dev Containers extension, then "Reopen in Container".

**Plain Docker:**

```sh
docker run --rm -it --cap-add NET_ADMIN -v "$PWD":/workspaces/vcu \
  -w /workspaces/vcu ghcr.io/ksu-ms/vcu-dev:latest
pio test -e native          # SIL tests
pio run -e teensy41         # firmware
tools/run_e2e.sh            # OTA end to end on vcan (Linux hosts only)
```

**Build it yourself** (needs Nix):

```sh
nix build .#devcontainer && docker load < result
```

Limits:

- vcan/SIL only works on a Linux host with the `vcan` module loaded
  (`sudo modprobe vcan`); Docker Desktop's VM on Mac/Windows doesn't have it.
- Flashing over USB from inside a container is awkward; build in the
  container and flash from the host with Teensy Loader, or use
  `tools/ota/flash.py` over a CAN adapter.
- Tested with `docker run` (tests, firmware build, OTA end to end). Not yet
  tried from VS Code itself.
