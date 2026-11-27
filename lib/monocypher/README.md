# monocypher (vendored)

[Monocypher](https://monocypher.org) 4.0.2 (git `0d85f98`), copied
unmodified: `src/monocypher.{c,h}` and `src/optional/monocypher-ed25519.{c,h}`.
Dual licensed BSD-2-Clause / CC0 (`LICENCE.md`).

Used to check OTA image signatures (Ed25519 over SHA-512, the standard
RFC 8032 variant from the optional file, so the host can sign with any
Ed25519 library) on the VCU and in the bootloader. Small, constant-time,
no allocation, and audited (2019, Cure53).
