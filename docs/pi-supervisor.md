# Pi as the controls brain, VCU as safety authority

Direction for moving controllers (traction, launch, power management, ...)
off the Teensy onto the car's Raspberry Pi, which today only does DAQ
(`ksu_daq`), while keeping the car drivable and safe if the Pi dies.

**The VCU side is done** (see "What the VCU does now" below): it takes a
torque ceiling from the Pi, checks every frame, falls back and hands back
as described here. Nothing on the Pi yet. Traction control meanwhile runs
on the VCU itself (`docs/traction-control.md`); the Pi's ceiling applies
after it.

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

## What the VCU does now

`include/pi_supervisor.hpp`, `src/pi_supervisor.cpp`; numbers in
`parameters.hpp`.

**The Pi's command**, `pi_torque_command` (0x010) on the DAQ bus, 100 Hz:

| Bytes | |
|---|---|
| 0-1 | torque ceiling, 0.1 Nm, little endian, at most `MAX_TORQUE_LIMIT_NM` |
| 2 | bit 0: the Pi wants control; other bits 0 |
| 3-5 | 0 |
| 6 | rolling counter in the low 4 bits, +1 each frame; high bits 0 |
| 7 | CRC-8 SAE J1850 (poly 0x1D, init and final xor 0xFF) over the ID as 2 bytes little endian, then bytes 0-6 |

`renode/fakepi.py` has a Python version of the frame and the CRC.

**Checks.** A frame is accepted if it's 8 bytes, the CRC matches, the
counter moved on by 1-3 (so up to two lost frames are fine; a repeat or a
jump isn't), and the values are in range. Any bad frame, or no good one
for `PI_COMMAND_TIMEOUT_MS` (30 ms), or the Pi clearing its request, drops
to local control at once.

**Torque.** In control, the VCU commands `min(pedal torque after traction
control, ceiling)`. Then the power and torque limits as always.

**Fallback.** Mid-throttle, the torque climbs from the Pi's last ceiling at
`PI_FALLBACK_RAMP_NM_PER_S` (1000 Nm/s, 0 to full in a quarter second)
rather than stepping.

**Handover.** Back to the Pi only after `PI_HEALTHY_MS` (500 ms) of good
frames asking for control, and the driver off the throttle.

**Status**, `vcu_pi_status` (0x0CD), 100 Hz: who has control, why it's
local (silent, not requested, bad checksum/counter/value, settling,
waiting for a lift), frames rejected, the driver's pedal request (what a
Pi controller needs to work from), the torque sent and the ceiling in
force.

**Tests.** `test/test_pi_supervisor` (19 cases, natively, including 3 s of
random good and garbage frames never letting more than the driver's
request through) and `renode/test_pi_supervisor_hil.py` (the real firmware,
with a fake Pi on vcan).

## Next, on the Pi

A C++ (or Python, to start) controller process that reads the DAQ bus,
sends `pi_torque_command`, and runs as a real-time process (item 6 above).
The VCU end won't need to change for it.

## Open questions

- Which bus is the Pi on? `ksu_daq/tools/up.sh` sets `can0` to 500 kbps,
  but the VCU's DAQ bus is 1 Mbps.
- CAN IDs: `pi_torque_command` (0x010) and `vcu_pi_status` (0x0CD) are on
  ksu-ms-dbc's `vcu-traction-pi` branch, which the firmware pins; they need
  merging.
- Bus load once sensor frames go out at control rate alongside the
  ACC/INV mirroring onto DAQ.

