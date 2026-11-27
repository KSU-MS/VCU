# Renode: the real firmware on an emulated Teensy 4.1

Runs what a USB flash puts on the board (the A/B bootloader, a boot state
and the firmware in slot A: `pio run -e teensy41`'s `firmware.hex`) on
Renode's i.MX RT1064 platform, which is the same die as the Teensy's RT1062.
Complements the host SIL in `lib/vcu_sim`, which runs the logic but not the
real startup code, drivers, CAN controller or flash routines.

```sh
nix develop
pio run -e teensy41

# boot it, CAN bridged to vcan_acc/inv/daq (tools/sil/setup_vcan.sh)
renode --console --disable-gui -e "include @renode/vcan.resc; start"

# all of it, no root needed (vcan module loaded)
tools/run_e2e.sh renode
tools/run_e2e.sh renode/test_pedals_hil.py   # just the pedal rules
tools/run_e2e.sh renode --flash-timing=max   # flash datasheet max times
```

CI runs it on every PR (the `renode-ota` job).

## Speed

The firmware runs about 5x faster than real time. It used to run ~8x
slower: profiling (sampling the PC from the monitor) showed 98% of the time
in the ADS1018 driver's `delayMicroseconds(500)` per channel, spinning on
the DWT cycle counter, one slow peripheral access per iteration. Now a
tight poll of the cycle counter fast-forwards virtual time 20 us per read
(`CortexM7_DWT.cs`), the flash's busy bit does the same, and
`emulation SetAdvanceImmediately true` stops that skipped time being paced
to the host clock. Delays come out at most 20 us long.

(On the real board that driver blocks ~2 ms of every loop: worth a look.
It also makes the "200 Hz" torque command ~175 Hz, as `Periodic` reschedules
from when it fires.)

What's still slow is CAN from the host: each frame handed into the running
machine costs ~8 ms of wall time inside Renode's time synchronisation,
whatever the CPU speed, so an OTA transfer runs at ~1 KiB/s and
`test_ota_e2e.py` takes ~5 minutes. Batching frames into the machine and
delivering them on time syncs didn't change it; the next step would be
reading Renode's TimeDomainsManager / HandleTimeDomainEvent path. The
pedal tests send a handful of frames and take ~25 s.

## The pedal rules on the real firmware

`test_pedals_hil.py` drives the emulated VCU like a bench: the fake rest of
the car on CAN (`fakecar.py`: ACU, inverter and dash, encoded with
ksu-ms-dbc's DBC) takes it to Ready to Drive, then the tests set what's at
the J1 connector, a pedal position or a wiring fault (unplugged, shorted to
ground, 5 V or GLV), and check the torque commands the firmware sends.
`board.py` turns that into ADC counts through the board's front end, from
the schematic: the 100K pull-downs, A10's divider, the ADC ranges. So it
checks the real drivers and loop, not just the pedal logic.

The tests cite the rules they check (`docs/requirements.md`), and are timed
in torque commands (every 5 ms in virtual time): each fault must zero the
torque within 100 ms. Changing the APPS limit back to the old 30 points
makes `test_apps_disagreement_latches` fail, so they can tell.

## OTA

Works end to end: A/B updates of the real bootloader and firmware over CAN.
`test_ota_e2e.py` boots the USB bundle (build A in slot A), then with
`tools/ota/flash.py`:

1. sends build B (the same source plus an empty commit, so a different
   version, linked for slot B). The VCU writes it into slot B and checks its
   signature, the bootloader verifies it again and boots it as a trial, and
   `flash.py` confirms it.
2. sends build A back into slot A without confirming, then makes A's main
   loop hang (a one-shot hook on `VCU::tick` jumps to a `b .`): the watchdog
   resets the board, the bootloader rolls back to B, and B reports the
   watchdog reset (DID 0xF104).

About 5 minutes, nearly all of it the two transfers (see Speed).

On the way it runs the bootloader (bare metal, from the boot ROM's state),
then boots through the Teensy core's `ResetHandler` from a slot, the
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
bundle is loaded). A command the real chip would ignore (sent while it's
busy, or without write enable) is logged as an error and fails the test.
The bootloader and the Teensy core get this right; the test keeps it that
way.

Not modelled: USB serial output, timing accuracy beyond "delays end", power
loss, reads of the flash while it's busy (the real chip
returns garbage; the core's flash routines run from RAM with interrupts off
so nothing should read it, but nothing checks that either).

## What's modelled and how

| File | What | Why |
|---|---|---|
| `teensy41.repl` | 1064 platform + 8 MB flash as memory, FlexCAN1-3, stubs for CCM/IOMUXC_GPR/USB/PIT | peripherals the Teensy core touches that the 1064 platform lacks |
| `teensy41.resc` | loads everything (the USB bundle .hex, symbols from the slot A ELF); on every reset jumps to the IVT entry in flash | there's no boot ROM; that entry is the bootloader, which picks a slot |
| `vcan.resc` | bridges FlexCAN1/2/3 to `vcan_acc/inv/daq` | same buses as the SIL, so the same tools work |
| `NXP_FlexCAN_RxIdFix.cs` | Renode's FlexCAN with two receive fixes | see below |
| `TeensyFlexSPI.cs` | FlexSPI IP commands against flash, as a W25Q64JV: NOR rules, datasheet busy times, write enable | the bootloader's and the Teensy core's flash routines: OTA downloads, boot state |
| `CortexM7_DWT.cs` | DWT cycle counter from executed instructions, fast-forwarding busy waits | Renode has no DWT; `delayMicroseconds()` spins on it forever |
| `ADS1018.cs` | the pedal/steering ADC on LPSPI4 | nothing was attached to the SPI bus |
| `IMXRT_RTWDOG.cs` | the RTWDOG (WDOG3): unlock, refresh, write-once config, LPO timeout resets the machine; the SRC's reset cause bit | the 1064 platform has it as a stub that never fires |
| `imxrt_adc.py` | ADC1 for `analogRead()` | Renode's `IMXRT_ADC` crashes on channels > 7 (see below) |
| `imxrt_analog.py` | CCM_ANALOG PLLs that remember writes and report lock | the stock flipflop stub can't satisfy `usb_pll_start` |
| `test_ota_e2e.py` | the A -> B update test (pytest, via `tools/run_e2e.sh renode`) | |
| `test_pedals_hil.py`, `board.py`, `fakecar.py` | the pedal rules through the real firmware | see above |
| `harness.py` | runs Renode with its monitor on a TCP port | tests change inputs while it runs |

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
