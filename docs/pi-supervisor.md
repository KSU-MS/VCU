# Pi as the controls brain, VCU as safety authority (proposal)

Not implemented yet. (Traction control now runs on the VCU itself,
`docs/traction-control.md`; the Pi's ceiling would apply after it.)
Direction for moving controllers (traction, launch,
power management, ...) off the Teensy onto the car's Raspberry Pi, which
today only does DAQ (`ksu_daq`), while keeping the car drivable and safe
if the Pi dies.

## The rule: the Pi requests, the Teensy decides

The Teensy stays the only thing that commands the inverter and keeps all of
today's safety logic (TS state machine, APPS/BSE plausibility, BSPD, power
limit). The Pi's input is a request the VCU validates and clamps, never an
authority.

## Design

1. **The Pi sends a torque ceiling, not a torque.** The VCU commands
   `min(driver pedal torque, Pi ceiling, VCU limits)`. TC, launch and power
   management only ever remove torque, so this covers them, and a broken
   Pi can cause loss of torque but never unintended acceleration. Anything
   that adds torque (regen, other pedal maps) would be a separate, more
   tightly bounded mode.

2. **Every Pi frame is checked.** Rolling counter + checksum (lightweight
   AUTOSAR E2E style). Accepted only if fresh (~30 ms at a 100-200 Hz
   command rate), the counter advanced, the checksum matches and the mode is
   valid.

3. **Fallback is automatic, re-engagement is deliberate.** Stale or invalid
   Pi input drops the VCU to local control (today's pedal x torque limit
   with the open-loop power limit), ramping from the Pi's last value rather
   than stepping. Control returns to the Pi only after it's been healthy for
   a while *and* the driver has lifted, so it never changes mid-corner. Mode
   and fallback reason go out in the VCU status frame (visible in Foxglove).

4. **The VCU publishes what controllers need.** Pedals, wheel speeds and
   inverter feedback at the control rate (pedals go out at 20 Hz today), on
   the bus the Pi shares with the VCU, with the Pi command on a low
   (high-priority) ID. The Pi never talks to the inverter.

5. **Controllers are a portable C++ library**, like `lib/ota`: no hardware
   dependencies, tested in the SIL. The Pi runs the full versions; the
   Teensy can build simplified ones for fallback. One codebase, one test
   suite, and a reason to write the new Pi stack in C++.

6. **On the Pi**, control runs as its own real-time process (PREEMPT_RT,
   `SCHED_FIFO`), separate from logging/telemetry, so a stuck logger can't
   starve it.

## Testing in the SIL

SIL VCU on vcan + a fake Pi process. Cases: Pi silent -> fallback within X
ms; bad checksum / stuck counter -> rejected; ceiling above pedal -> clamped
to pedal; Pi recovers mid-throttle -> handover waits for pedal lift; fallback
torque matches today's behaviour.

## Open questions

- Which bus is the Pi on? `ksu_daq/tools/up.sh` sets `can0` to 500 kbps,
  but the VCU's DAQ bus is 1 Mbps.
- CAN IDs for the Pi command and VCU mode/status need reserving in
  ksu-ms-dbc.
- Bus load once sensor frames go out at control rate alongside the
  ACC/INV mirroring onto DAQ.

## Suggested first step

VCU side only: request validation, ceiling, fallback and handover, with SIL
tests and a minimal C++ fake-Pi controller on vcan, before any Pi rewrite.
