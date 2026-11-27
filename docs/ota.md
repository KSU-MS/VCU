# Firmware update over CAN

Proof of concept, not yet run on a real Teensy. What has been checked:

- **Host SIL** (`pio test -e native`, `tools/run_e2e.sh tools/sil`): the protocol,
  staging logic, interlocks and failure handling, against a NOR flash model.
- **Renode** (`tools/run_e2e.sh renode`): the real ARM firmware on an emulated
  Teensy 4.1 updated end to end over CAN: staging through the Teensy core's
  flash routines, CRC read-back, FlasherX's `flash_move` and the reset into
  the new image. See `renode/README.md`.

## How it works

- Tester on the DAQ bus talks UDS over ISO-TP: request `0x7E0`, response
  `0x7E8` (not yet reserved in ksu-ms-dbc, do that first).
- Sequence: `10 02` programming session, `34` RequestDownload, `36`
  TransferData (1 KB blocks), `37` exit, `31 01 FF01` check image, `11 01`
  reset/apply. See `lib/ota/uds_server.hpp`.
- ISO-TP flow control asks for 3 frames per block: FlexCAN_T4 in polling
  mode only has 4 standard-ID receive mailboxes, emptied once per loop.
- Image = 64 byte header (magic, size, CRC32, git version, target board,
  header CRC) + raw flash image from `firmware.hex`. `tools/ota/image.py`.
- Staged in a 1 MB flash area at +4 MB, read back and CRC checked, then
  FlasherX's RAM-resident `flash_move` copies it over the firmware and reboots.
- Only allowed in STARTUP/TS_DISABLED with the inverter off; the VCU holds
  TS_DISABLED while a programming session is open (5 s timeout).

```sh
nix develop
uv run tools/ota/image.py build .pio/build/teensy41/firmware.hex -o vcu.ota
uv run tools/ota/flash.py vcu.ota --interface socketcan --channel can0
tools/run_e2e.sh tools/sil   # SIL VCU on vcan, flashed by the real tool, no root
tools/run_e2e.sh renode      # real ARM firmware in Renode, A -> B, no root
```

## Known limitations

- Not power-fail safe during the final copy: losing power then needs a USB
  reflash. An A/B bootloader would fix this. The copy erases and rewrites
  the firmware and then erases the staged copy, 4 KB at a time: for a 110 KB
  image about 4 s at the W25Q64's typical times, up to ~25 s at its
  datasheet max (`apply_time()` in `tools/ota/flash.py`, which is also how
  long the tool waits for the new version to answer).
- Interrupts aren't off for the whole copy: the Teensy core's flash routines
  re-enable them after every erase/program (`flash_wait()` in `eeprom.c`;
  FlasherX's `leave_interrupts_disabled` only applies to Teensy 3.x). An ISR
  that ran code or read constants from flash then would see a half-written
  image. The VCU's ISRs (systick, USB) run from RAM, but it's worth knowing
  before adding interrupt-driven CAN.
- No authentication: anyone on the DAQ bus can flash a parked car. Signing
  images (e.g. ed25519) is the next step.
- `flash_move` erases everything above the new image, including EEPROM
  emulation (the VCU doesn't use it today).
- Throughput is paced by the VCU loop (one 3-frame block per loop): about
  18 KiB/s in the SIL, likely half that on the car.

## Bench checklist (needs hardware)

Renode covered the logic of 1-3 and 6; these check the silicon and the
real flash chip.

1. Flash over USB, confirm `22 F189` returns the git version.
2. Stage + verify only (`flash.py --no-reset`), check CRC passes: proves
   erase/program/read-back on the real W25Q64.
3. Full update, confirm it reboots into the new version.
4. Measure transfer time on the real 1 Mbps bus.
5. Flash erase disables interrupts for tens of ms (up to hundreds worst
   case). Check ACU/inverter frames dropped during an update are harmless.
6. Confirm a programming session is refused with TS live.
7. Pull power mid-download (must boot old firmware) and mid-apply
   (expect USB recovery needed).
