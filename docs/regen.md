# Regen

The hub motor cars regenerate under the brake pedal, as kmslap's
`RegenPolicy` and hub torque controller do (python-data-analysis
`kmslap/driving/policies.py`, `kmslap/vcu/torque_control.py`). rwd_emrax
doesn't: its Cascadia has never been set up for regen, and its brake
sensor measures pressure, not travel.

- `vehicle::THIS.regen_brake_travel` (`include/vehicle.hpp`): which cars
  regenerate, and over how much brake travel.
- `Pedals::regen_fraction`, `Pedals::hydraulic_travel`
  (`include/pedal_handeler.hpp`): the brake pedal's two parts.
- `VCU::tick` (`src/vcu_tick.cpp`): the request, and the speed rule.
- `Drivetrain::command`, `Drivetrain::limit_regen` (`src/drivetrain.cpp`):
  the split and the pack's limits.
- `test/4wd_dti/test_regen/`: a test for each of the rules and limits.

## The brake pedal

T.3.2.4: the first 90% of the brake pedal's travel may regenerate
without the hydraulics, and the rest must work the hydraulics (with regen
staying on). On the hub cars `regen_brake_travel` is 0.9, kmslap's:
- **0.03 to 0.9 of the travel:** regen, rising from none to the most. The
  first 0.03 is a deadband for sensor noise.
- **0.9 to 1:** the hydraulics, regen staying at the most.

The rules that mean "the mechanical brakes" use the hydraulic part only
(`hydraulic_travel`, 0..1 over the last 10%):
- EV.4.7.1's brake + throttle cut, which latches until the throttle's under
  5%. Braking in the regen travel with the throttle on doesn't latch it;
- EV.10.6.2: ready to drive needs the hydraulics engaged.

**The brake overrides the throttle:** with the pedal anywhere past the
deadband, there's no drive torque.

**Assumed:** the hub car has a brake pedal *travel* sensor and a pedal box
with that free travel before the master cylinders. kmslap models it that
way (its logs' `vcu_bse_travel` is pedal travel). Today's car measures
brake pressure, which can't see travel that doesn't build pressure.

## How much, and where

- **The request:** the regen fraction of every regen-capable motor's most
  at its speed (its torque curve), as wheel torque, negative.
- **EV.3.3.3:** none at 5 km/h or less, fading in to full at 8 km/h
  (kmslap's `regen_min_speed`, `regen_fade_speed`). The speed is
  VectorNav's: without it, no regen.
- **The split:** the same load split as driving. Braking moves load to the
  front, so the front axle takes more.
- **Traction control and the Pi's ceiling** only ever limit drive torque;
  regen bypasses them.

## The pack's limits

On the total, every motor alike:
- **The BMS's charge current limit** (`Pack_CCL`, in its charger message
  0x1806E5F4): the power into the pack (motor power × 0.90) at most CCL ×
  pack voltage. Not heard for 1 s: no regen at all.
- **EV.3.3.2, 600 V:** regen fades out as the highest inverter DC voltage
  goes from 585 to 595 V, or stops at the BMS's maximum pack voltage if
  that's lower.
- **Each DTI's own limit:** its DC brake current (0x0B) is its share of the
  CCL, as its DC current limit is of the discharge limit
  (docs/vehicles.md), and its AC brake current (0x09) is its motor's peak.

## Each wheel's slip

kmslap's `RegenPolicy.slip_scale`, per wheel: a wheel turning slower than
the car (its motor speed against VectorNav's) keeps all its regen up to 8%
slip, none from 14%, and is filtered at 20 ms. It's the "regen only behind
slip control" kmslap asks of rear-only regen, for all four. It brings a
locking wheel back, not an ABS: the hydraulics have none.

## What the replay shows

`tools/replay/replay.py` reports regen against kmslap's
(`docs/vehicles.md`, "Checked against kmslap").
- **At 100% charge,** kmslap's pack is 604.8 V (144 cells at 4.2 V). That's
  over EV.3.3.2 before any regen, and kmslap's regen takes it to 616 V. The
  VCU gives none there, correctly: the car can't be charged that full and
  regenerate legally.
- **At 80% charge** (569 V at rest, 587 V at most regenerating) the VCU
  regenerates where kmslap does: −24.7 N·m of total motor torque on
  average to kmslap's −18.8, with more on the front (57 % to 54 %). The
  VCU takes back more, since kmslap's QP holds wheels back near their
  grip limit. Details in docs/vehicles.md.

## To settle before the car

- The brake pedal travel sensor and the free travel (above).
- The BMS sending its charger message (0x1806E5F4) while driving. Orion
  BMSs send it to the charger; check it's on the bus with no charger, or
  there's no regen.
- The DTI's 0x0B unit: its manual's table says %, its example A.
- Calibrating the deadband, and the 0.90 regen efficiency.
- The regen torque's sign at the DTI (negative AC current while moving
  forward), on the bench at low speed.
