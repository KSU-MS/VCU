# Renode: the real firmware on an emulated Teensy 4.1

Runs the actual `firmware.elf` (ARM, same binary you'd flash) on Renode's
i.MX RT1064 platform, which is the same die as the Teensy's RT1062.
Experimental; complements the host SIL in `lib/vcu_sim`, which runs the
logic but not the real startup code, drivers or flash routines.

```sh
nix develop -c pio run -e teensy41
nix shell nixpkgs#renode -c renode --console --disable-gui \
  -e "include @renode/teensy41.resc; start"
```

## Status

Boots through the Teensy core's `ResetHandler` (RAM copies, clocks, USB
init, startup delay), the firmware's global constructors (FlexCAN x3,
ADS1018, OTA service) and `setup()`, then runs `loop()` continuously:
pedals from the ADS1018 model, brake/BSPD from the ADC model, status and
pedal frames sent through the FlexCAN models.

Not yet: getting CAN frames in and out of the emulator, the FlexSPI flash
model (needed to run an OTA update end to end), USB serial output.

## What's modelled and how

| File | What | Why |
|---|---|---|
| `teensy41.repl` | 1064 platform + 8 MB flash as plain memory, FlexCAN1-3, stubs for CCM/IOMUXC_GPR/USB/PIT | peripherals the Teensy core touches that the 1064 platform lacks |
| `teensy41.resc` | load everything, jump to `ResetHandler` | there's no boot ROM to read the IVT |
| `imxrt_analog.py` | CCM_ANALOG PLLs that remember writes and report lock | the stock flipflop stub can't satisfy `usb_pll_start` |
| `cortexm7_dwt.py` | DWT cycle counter, advances on every read | Renode has no DWT; `delayMicroseconds()` spins on it forever |
| `imxrt_adc.py` | ADC1 for `analogRead()` | Renode's `IMXRT_ADC` crashes on channels > 7 (see below) |
| `ADS1018.cs` | the pedal/steering ADC on LPSPI4, compiled by Renode at load | nothing was attached to the SPI bus |

Shortcuts in `teensy41.resc`:

- `configure_cache` is skipped: Renode's MPU resolves the Teensy's
  overlapping regions differently from the silicon and everything faults.
  Renode has no caches anyway.
- `configure_external_ram` is skipped: the PSRAM probe waits on FlexSPI2
  commands that never complete with nothing attached. The VCU has no PSRAM.

Inputs can be changed from the monitor while it runs, e.g.:

```
sysbus.lpspi4.ads1018 AIN0 900          # throttle (firmware "APPS1")
sysbus WriteDoubleWord 0x400C5004 900   # brake on A10 (ADC1 channel 1)
```

## Renode issues found

- `IMXRT_ADC.TriggerConversion` writes `cdata[channel]`, but `cdata` has one
  entry per hardware trigger (8), not per channel (16), so conversions on
  channels 8-15 throw and take Renode down. On the silicon the result goes
  to the trigger's `R[n]`. Worth reporting upstream.
- No DWT model, so any Teensy code using `micros()`/`delayMicroseconds()`
  hangs without `cortexm7_dwt.py`.
