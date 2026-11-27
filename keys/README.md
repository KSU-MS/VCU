# OTA signing keys

Firmware images must be signed (Ed25519) with a key whose public half was
built into the firmware and the bootloader, or they're refused.

`ota-dev.key` / `ota-dev.pub` are the **development key**. Its private half
is right here in the repo, so anyone can sign an image for a firmware built
with it. That's the default so the tests, the SIL and Renode work out of the
box, and the build says so (`OTA public key: DEV KEY ...`).

For the car, make a real key once and keep `car.key` off GitHub:

```sh
uv run tools/ota/image.py keygen keys/car          # car.key + car.pub
VCU_OTA_PUBLIC_KEY=keys/car.pub pio run -e teensy41   # firmware trusts car.pub
VCU_OTA_KEY=keys/car.key uv run tools/ota/image.py build ... # signs with car.key
```

`*.key` other than the dev key is ignored by git. Commit `car.pub`, it's
public. If a release is built in CI, the private key goes in a repository
secret, not a file.
