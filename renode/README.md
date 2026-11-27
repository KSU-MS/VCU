# Renode: the real firmware on an emulated Teensy 4.1

Runs the actual `firmware.elf` (ARM, the same binary you'd flash) on
Renode's i.MX RT1064 platform, which is the same die as the Teensy's RT1062.
Complements the host SIL in `lib/vcu_sim`, which runs the logic but not the
real startup code, drivers, CAN controller or flash routines.

```sh
nix develop
pio run -e teensy41

# boot it, CAN bridged to vcan_acc/inv/daq (tools/sil/setup_vcan.sh)
renode --console --disable-gui -e "include @renode/vcan.resc; start"

# full OTA update A -> B over CAN, no root needed (vcan module loaded)
tools/run_e2e.sh renode
tools/run_e2e.sh renode --flash-timing=max   # flash datasheet max times
```

CI runs it on every PR (the `renode-ota` job).

## Status

Works end to end: an OTA update of the real ARM firmware over CAN.
`test_ota_e2e.py` boots build A, sends build B (the same source plus an
empty commit, so a different version) with `tools/ota/flash.py`, the VCU stages and CRC-checks it in
flash, FlasherX's `flash_move` copies it over A from RAM and resets, and
the board comes back reporting B's version. About 3.5 minutes; the
emulator runs ~6x slower than real time (`flash.py --timeout-scale`).

On the way it boots through the Teensy core's `ResetHandler`, the
firmware's constructors and `setup()`, runs `loop()` with pedals from the
ADS1018 model and brake/BSPD from the ADC model, and talks CAN to the host.

It found a real-hardware bug the SIL missed: a 1 KB ISO-TP burst overran
FlexCAN_T4's 4 standard-ID receive mailboxes (commit 9081b11), which led to
interrupt-driven receive in ksu-fw-common. Received frames are paced one
frame time apart, like a real bus (`NXP_FlexCAN_RxIdFix.cs`, FIX 3), so the
test also checks the receive interrupt keeps up.

The flash behaves like the Teensy 4.1's W25Q64JV: datasheet erase/program
times (typical, or max with `--flash-timing=max`), the busy bit, write enable,
page wrap, and blank flash reading 0xFF (the model chip-erases before the
ELF is loaded; with Renode's default of zeros FlasherX erased ~1900 "dirty"
sectors after every update). A command the real chip would ignore (sent
while it's busy, or without write enable) is logged as an error and fails
the test. The Teensy core and FlasherX get this right; the test keeps it
that way.

Not modelled: USB serial output, timing accuracy beyond "delays end", power
loss during `flash_move`, reads of the flash while it's busy (the real chip
returns garbage; the core's flash routines run from RAM with interrupts off
so nothing should read it, but nothing checks that either).

## What's modelled and how

| File | What | Why |
|---|---|---|
| `teensy41.repl` | 1064 platform + 8 MB flash as memory, FlexCAN1-3, stubs for CCM/IOMUXC_GPR/USB/PIT | peripherals the Teensy core touches that the 1064 platform lacks |
| `teensy41.resc` | loads everything; on every reset jumps to the IVT entry in flash | there's no boot ROM; reading flash (not the ELF) boots a freshly applied image |
| `vcan.resc` | bridges FlexCAN1/2/3 to `vcan_acc/inv/daq` | same buses as the SIL, so the same tools work |
| `NXP_FlexCAN_RxIdFix.cs` | Renode's FlexCAN with two receive fixes | see below |
| `TeensyFlexSPI.cs` | FlexSPI IP commands against flash, as a W25Q64JV: NOR rules, datasheet busy times, write enable | the Teensy core's flash routines, OTA staging and `flash_move` |
| `CortexM7_DWT.cs` | DWT cycle counter from executed instructions | Renode has no DWT; `delayMicroseconds()` spins on it forever |
| `ADS1018.cs` | the pedal/steering ADC on LPSPI4 | nothing was attached to the SPI bus |
| `imxrt_adc.py` | ADC1 for `analogRead()` | Renode's `IMXRT_ADC` crashes on channels > 7 (see below) |
| `imxrt_analog.py` | CCM_ANALOG PLLs that remember writes and report lock | the stock flipflop stub can't satisfy `usb_pll_start` |
| `test_ota_e2e.py` | the A -> B update test (pytest, via `tools/run_e2e.sh renode`) | |

One shortcut: `configure_cache` is skipped. Renode's MPU resolves the
Teensy's overlapping regions differently from the silicon and everything
faults, and Renode has no caches anyway. The hook is by symbol address
from the booted ELF, which only matters if an OTA image moves that
function (it doesn't between builds of the same code).

Inputs can be changed from the monitor while it runs:

```
sysbus.lpspi4.ads1018 AIN0 900          # throttle (firmware "APPS1")
sysbus WriteDoubleWord 0x400C5004 900   # brake on A10 (ADC1 channel 1)
```

## Renode bugs found (worth reporting upstream)

- `NXP_FlexCAN`: `FillReceivedFrame` never copies the received frame's ID
  into the mailbox, so firmware polling mailboxes sees every frame with ID
  0. Fixed in `NXP_FlexCAN_RxIdFix.cs`.
- `NXP_FlexCAN`: receive prefers a Full mailbox over an Empty one (it sorts
  by code, and Full < Empty), so back-to-back frames overwrite each other.
  Fixed in the same file.
- `IMXRT_ADC.TriggerConversion` writes `cdata[channel]`, but `cdata` has one
  entry per hardware trigger (8), not per channel (16), so conversions on
  channels 8-15 throw and take Renode down. Worked around with
  `imxrt_adc.py`.
- No Cortex-M DWT model. Worked around with `CortexM7_DWT.cs`.
