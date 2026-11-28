# Bootloader

The first 64 KB of flash. On every reset it:

1. reads the boot state journal (`lib/vcu_boot/boot.hpp`),
2. verifies both firmware slots: header, linked for that slot, CRC, and the
   Ed25519 signature against the public key built in (`scripts/ota_key.py`),
3. lets `boot::decide()` pick one: a pending update gets one trial boot,
   an unconfirmed one is rolled back, a broken slot falls back to the other,
4. records that, starts the watchdog (4 s to boot, then the firmware sets
   1 s; `docs/ota.md`), and jumps to the chosen firmware's entry point (the
   second word of its IVT, as the boot ROM would).

All of that policy is in `lib/vcu_boot` and tested natively
(`test/test_boot`). This directory is only the bare-metal glue:

| File | What |
|---|---|
| `startup.c` | FlexSPI config block and IVT the boot ROM needs (PJRC's), vector table, reset handler |
| `flexspi.cpp` | Erase/program through FlexSPI IP commands, run from ITCM |
| `main.cpp` | The four steps above |
| `bootloader.ld` | Flash 0x60000000-0x6000FFFF, a little ITCM and DTCM |

Bare metal on purpose: no Teensy core, so clocks, caches and peripherals are
still as the boot ROM left them when the firmware starts, and its own
`ResetHandler` runs exactly as from a cold boot.

Built by `pio run -e bootloader`, but you normally don't: `pio run -e
teensy41` builds it and bundles it with slot A into the `firmware.hex` it
uploads.

If neither slot verifies, it waits forever. The Teensy's USB bootloader is a
separate chip and still works, so the fix is a USB flash.
