# Firmware update over CAN

Proof of concept, not yet run on a real Teensy. What has been checked:

- **Native tests** (`pio test -e native`): the protocol, interlocks, image
  signatures, the boot-state journal (with power cut at every byte of every
  write) and the bootloader's policy, on a NOR flash model.
- **Host SIL** (`tools/run_e2e.sh tools/sil`): the VCU as a Linux program on
  virtual CAN, updated, confirmed and rolled back by the real `flash.py`.
- **Renode** (`tools/run_e2e.sh renode`): the real bootloader and firmware on
  an emulated Teensy 4.1 with the W25Q64's timing: update A -> B and
  confirm, then B -> A unconfirmed and a reset rolls back to B. See
  `renode/README.md`.

## How it works

Two firmware slots and a small bootloader (`bootloader/`). An update goes
into the slot that isn't running, so the running firmware is never touched
and losing power at any point leaves something bootable.

```
0x60000000  bootloader (64 KB)        verifies slots, picks one, jumps to it
0x60010000  boot state (2 x 4 KB)     append-only journal: active / trial slot
0x60020000  slot A (1.875 MB)         signed image, firmware linked for +4 KB
0x60200000  slot B (1.875 MB)         same, firmware linked for B
0x607C0000  Teensy EEPROM emulation   untouched
```

1. Tester on the DAQ bus talks UDS over ISO-TP: request `0x7E0`, response
   `0x7E8` (not yet reserved in ksu-ms-dbc, do that first).
2. `22 F103` says which address an update must be linked for (the other
   slot's). `pio run -e teensy41 -e teensy41_b` builds both, and `flash.py`
   sends the matching one.
3. `10 02` programming session (only with TS off and the inverter disabled;
   the VCU holds TS_DISABLED while it's open, 5 s timeout), `34`
   RequestDownload, `36` TransferData in 1 KB blocks straight into the
   other slot, `37` exit.
4. `31 01 FF01` check: header, linked for that slot, CRC, and the Ed25519
   signature over SHA-512(header + firmware) against the public key built
   into the firmware (`scripts/ota_key.py`). Checked on what's in flash.
5. `11 01` ECUReset: the VCU records "boot the other slot once, as a trial"
   and reboots.
6. The bootloader verifies both slots again (same key, built into it),
   counts the trial boot and starts the new firmware.
7. Once the new version answers, `flash.py` sends `31 01 FF02` confirm and
   it becomes the active slot. If that never happens (it crashes, can't
   talk CAN, the laptop dies), the next reset goes back to the old slot.

While running an unconfirmed update, a new download is refused: the other
slot holds its fallback. `22 F102` reads the boot state (running slot,
active slot, trial slot, trial boots).

ISO-TP flow control asks for 64 frames per block. Frames land in
ksu-fw-common's 256-frame receive queue, filled by the FlexCAN interrupt.

```sh
nix develop
pio run -e teensy41 -t upload      # over USB: bootloader + slot A + fresh state
pio run -e teensy41 -e teensy41_b  # .pio/build/*/vcu-{a,b}.ota, signed
uv run tools/ota/flash.py .pio/build/teensy41/vcu-a.ota .pio/build/teensy41_b/vcu-b.ota \
    --interface socketcan --channel can0
tools/run_e2e.sh                   # SIL + Renode end to end, no root
```

`pio run -e teensy41` turns `firmware.hex` into the USB bundle (bootloader,
a boot state saying "slot A", slot A), so uploading it from VS Code or
Teensy Loader sets up the whole board, and always boots the firmware just
built. The plain slot A firmware is `app.hex`.

## Signing keys

Images are signed with `keys/ota-dev.key` unless `VCU_OTA_KEY` says
otherwise, and the firmware and bootloader trust `keys/ota-dev.pub` unless
`VCU_OTA_PUBLIC_KEY` says otherwise. The dev key's private half is in the
repo: fine on the bench, never on the car. See `keys/README.md`. The build
prints which key it trusts.

## Watchdog

The bootloader starts the i.MX RT's RTWDOG (WDOG3) on every boot, before it
jumps to the firmware, clocked from the 32.768 kHz crystal:

1. **Boot window, 4 s.** The firmware takes over a second to reach its
   loop: the Teensy core waits 300 ms for USB, then the logger's
   `Serial.begin()` waits up to 750 ms for a terminal (2 s if a laptop is
   plugged in). The bootloader leaves the watchdog reconfigurable once
   (`CS[UPDATE] = 1`).
2. **Running, 1 s, locked.** At the end of `VCU::begin()`,
   `hal::watchdog_start()` sets 1 s and `CS[UPDATE] = 0`: nothing can change
   or stop it now until the next reset. `VCU::tick()` feeds it as its last
   act, every pass of the loop.

A hang (main loop stuck, interrupts or not) resets the board. An update on
trial that hangs is rolled back by that reset, like any other. After a
watchdog reset the firmware logs "Reset by the watchdog" and reports it in
DID 0xF104 (1 = watchdog, 0 = anything else), `flash.reset_by_watchdog()`.

It's for recovery, not torque safety: a hung VCU stops sending torque
commands at once, and it's the inverter's own CAN timeout that zeros the
torque (bench checklist). 1 s is the time the car is without its VCU before
it reboots, and booting takes about another second.

Firmware loaded without the bootloader (a plain `app.hex`) never has the
watchdog: the boot ROM leaves it off and `watchdog_start()` only shortens
one that's already running.

## Known limitations

- A trial that hangs is reset by the watchdog (see below) and rolls back,
  but one that keeps its loop running while broken in some other way (CAN
  dead, wrong torque) stays until someone resets it: the confirm step in
  `flash.py` is the only check that an update works.
- Signed images stop someone on the DAQ bus from flashing their own
  firmware, but not from re-flashing an older image we signed (no
  anti-rollback counter), and only if the car's key isn't the dev key.
- Erasing the other slot pauses the VCU loop: the Teensy core's flash
  routines keep interrupts off while the chip is busy, 45 ms per 4 KB
  sector typically, up to 400 ms. Only during a programming session, with
  the car parked, but CAN frames on the other buses are lost meanwhile.
- The bootloader adds two signature checks to every boot (SHA-512 over each
  slot and an Ed25519 verify, a few ms each at the boot ROM's clock).
- Throughput: about 120 KiB/s in the SIL, where the bus is virtual and
  instant. On the car the 1 Mbit/s DAQ bus caps it near 60 KiB/s of
  payload, less whatever else is on the bus, plus the erase pauses.

## Bench checklist (needs hardware)

Renode covered the logic of all of these; they check the silicon, the real
flash chip and the Teensy's own USB bootloader.

1. `pio run -e teensy41 -t upload`: the board boots, `22 F189` returns the
   git version, `22 F102` says slot A, no trial. Do it twice, and once after
   an update to B: a USB flash must always come back as the new slot A.
2. Full update with `flash.py`, check it comes back as the new version,
   confirmed. Power cycle, still the new version.
3. `flash.py --no-confirm`, then power cycle: back to the old version.
4. Measure transfer time on the real 1 Mbit/s bus.
5. Check ACU/inverter frames dropped during erase pauses are harmless.
6. A programming session is refused with TS live.
7. Pull power mid-download and while the bootloader runs: always boots.
8. Measure the bootloader's added boot time.
9. Watchdog: boot with a laptop on USB and without, no terminal open:
   never a watchdog reset (`22 F104` says 0). Then a firmware whose
   `loop()` does `if (millis() > 10000) while (1);`: it resets every
   ~11 s and `22 F104` reads 1 in the 10 s it answers. With the inverter enabled, check how
   long it takes the inverter to drop torque when the VCU stops talking.
