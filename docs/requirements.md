# VCU requirements: FSAE Rules 2027

The rules the VCU firmware is responsible for, from **Formula SAE Rules 2027,
Version 1.0 (1 Sept 2026)**. Each `###` heading is a requirement; tests cite
it by putting its ID in brackets in their name, e.g.
`TEST_CASE("[EV.4.7.2] held until the APPS is under 5%")`.

`tools/requirements/test_traceability.py` (part of `uv run pytest`, so CI)
fails if a requirement here has no test, or a test cites an ID that isn't
here. `uv run tools/requirements/reqtrace.py` prints which tests cover what.

Most are tested twice: on the pedal logic natively (`test/`), and through
the real firmware in Renode with faults injected at the connector
(`renode/test_pedals_hil.py`).

The rule text below is paraphrased; the rulebook is the authority (page
numbers are the PDF's). When a new rulebook comes out, re-check each one and
update the IDs: the tests follow.

Status: **met** (implemented and tested), **partial** (tested, with a stated
gap), **assumption** (holds if something outside the firmware is true).

## Accelerator pedal (APPS)

### T.4.2.3 — Separate transfer functions, open circuit reads 0%
p.66. The APPS sensors must have different transfer functions (gradient
and/or offset), a pull-up or pull-down so an open circuit input reads 0%
travel, and above 10% travel each sensor's output must differ from the
other's by at least 10%.

VCU: APPS1 runs 670..1100 counts, APPS2 450..710: different offsets and
gradients, at least 237 counts (11.6% of the ADC's 2047) apart everywhere
above 10% travel. The board pulls each APPS input down with 100K (R46, R50,
from the schematic), so open circuit reads 0 counts: out of range
(T.4.2.10). **met**

### T.4.2.4 — Implausibility is more than 10% apart
p.66. A deviation of more than 10% pedal travel between the sensors, or any
other APPS failure, is an implausibility.

VCU: `Pedals::APPS_IMPLAUSIBLE_TRAVEL` = 0.10. **met**

### T.4.2.5 — Stop motor power if it lasts over 100 ms
p.66. An implausibility persisting more than 100 ms must immediately and
completely stop power to the motor; the motor controller doing it is
enough.

VCU: throttle is cut the moment the APPS disagree (stricter), and a
disagreement over 100 ms latches until the pedal is released. Torque goes
to the inverter as 0. **met**

### T.4.2.9 — Any APPS or wiring failure is detected, like an implausibility
p.66. Any failure of the APPS or its wiring must be detectable and treated
like an implausibility.

VCU: out-of-range readings (T.4.2.10), disagreement, a stuck sensor, both
failing alike, the two shorted together; a faulted sensor reads 0 travel,
never its last value. **met**

### T.4.2.10 — Open or short circuit gives an out-of-range signal
p.66-67. An analogue APPS has failed when an open or short circuit puts its
signal outside the operating range; pull resistors must make an open
circuit detectable.

VCU: `MIN_APPS_PEDAL` / `MAX_ACCELERATOR_PEDAL_1/2`. **met**

## Brake system encoder (BSE)

### T.4.3.3 — BSE failure over 100 ms stops motor power
p.67. Any failure of the BSE or its wiring persisting more than 100 ms must
be detected and treated like an implausibility: power to the motor stopped
immediately.

VCU: a BSE fault cuts throttle immediately (stricter), and a faulted BSE
reads no brake, so it can't help enter Ready to Drive. **met**

### T.4.3.4 — Open or short circuit gives an out-of-range signal
p.67. Same as T.4.2.10, for the BSE.

VCU: `MIN_BRAKE_PEDAL` / `MAX_BRAKE_PEDAL` (960: below the ~995 a short to
5 V reads through A10's divider). An open J1.10 reads 0 through the
divider's 12K to ground. **met**, **assumption** (full brake press reads
under 960 on the car)

## Brake / throttle plausibility

### EV.4.7.1 — Monitor brakes engaged and APPS over 25%
p.94. Monitor for the mechanical brakes being engaged (T.3.2.4: the
hydraulic brakes, not regen travel) and the APPS signalling more than 25%.

VCU: brake travel over `Pedals::BRAKE_ACTUATED_TRAVEL` (0.10) and the
higher APPS over 0.25. **met**, **assumption** (0.10 of the brake pressure
range is where the brakes engage: calibrate)

### EV.4.7.2 — Cut motor power until the APPS is under 5%
p.94. Both at once: power to the motor immediately and completely shut
down, and kept shut down until the APPS is under 5%, with or without brake.

VCU: `Pedals::PEDAL_RELEASED_TRAVEL` = 0.05. **met**

## Torque

### EV.3.1.3 — Control may only decrease the driver's requested torque
p.92. An algorithm or ECU that can adjust the requested wheel torque may
only decrease the total the driver requested, never increase it.

VCU: three things adjust it, and each only ever takes away:
- traction control returns at most the driver's torque, whatever its
  controller says (`TractionControl::step`, tested on random inputs);
- the Pi's torque ceiling is a minimum with it (`PiSupervisor::step`,
  tested on random and garbage frames);
- on cars with more than one motor, the split's total wheel torque is at
  most the ask (`torque_split::split`).

**met**

## Power

### EV.3.3.1 — At most 80 kW
p.92. The power measured by the Energy Meter must not exceed 80 kW
(a violation is 100 ms continuous, or over a 500 ms moving average,
EV.3.4.1).

VCU: torque is capped at `POWER_LIMIT_KW` × 0.90 / motor speed (shaft
power), trimmed in closed loop on measured DC bus V × I (`PowerLoop`):
each sample over 97% of the limit scales the cap to land on it, and below
it creeps back up. So it holds whatever the drivetrain loses (tested at
70-99% efficiency), and uses the budget rather than stopping at 72 kW.
With no fresh measurement it falls back to the open-loop cap.
- rwd_emrax: on the Cascadia's own measurement
  (`Inverter::regulate_power`).
- The hub motor cars: one loop on all the inverters' measured power added
  up (`Drivetrain::regulate_power`). When it binds, every motor's torque
  is scaled alike, so the power goes where the torque split puts it, not a
  fixed quarter each. Each DTI's DC current limit (0x0A) follows its share
  as well, and they add up to the limit over the pack voltage. One
  inverter's current not heard for 50 ms: the open-loop cap.

The car has no HV→LV converter, so the inverters are all the Energy Meter
sees. **met**, **assumption** (the inverters' DC current and voltage read
within ~3% of the Energy Meter; DTI's documents give no accuracy for
theirs, so check it on the bench)

**Test override:** the limit can be set over CAN (`vcu_set_parameter`,
parameter 0, kW) for testing the car past 80 kW. Only while parked (not
in TS enabled or ready to drive), only up to the motors' summed peak power
(EMRAX 100 kW; 4 × F-MOT 140 kW), and back to 80 kW at every power-up; 0
puts it back too. The VCU logs it when it's set, and the status frame
(`vcu_status`, 0x0C3) carries it as `vcu_power_limit_kw`. **Never at
competition**: it's a violation of this rule.

### EV.3.4.1 — A violation is 100 ms over, or over on a 500 ms average
p.92. Using more than the maximum power is a violation if it lasts 100 ms
continuously, or after a 500 ms moving average.

VCU: how the power tests judge a run: the model drivetrain's electrical
power, every ms, must never be over 80 kW for 100 ms straight or on any
500 ms average. **met**

### EV.3.3.3 — No regen between 0 and 5 km/h
p.92. The powertrain must not regenerate energy at 0-5 km/h.

VCU: never commands negative torque at all (no regen). **met**

## Activation

### EV.10.6.1 — Motors only respond to the APPS in Ready to Drive
p.111. Ready to Drive is when the motors respond to the APPS.

VCU: torque is 0 and the inverter disabled in every other state. **met**

### EV.10.6.2 — Ready to Drive needs TS active, brake held, driver action
p.111. Not possible until, at the same time: Tractive System Active, the
brake pedal pressed and held to engage the mechanical brakes, and a manual
driver action (a cockpit button).

VCU: from TRACTIVE_SYSTEM_ENERGIZED only, with brake travel over
`MINIMUM_BRAKE_FOR_RTD` (0.3) from a working BSE, and the dash RTD button.
**met**

### EV.10.7.2 — Ready to Drive sound for 1-3 s
p.111. Sounded continuously for at least 1 s and at most 3 s (and at least
80 dBA, recognisable: the buzzer's job).

VCU: `RTD_BUZZER_DURATION_MS` = 2151. **met** (timing), **assumption**
(buzzer loudness)

### EV.10.7.3 — No other sounds like it
p.111. The vehicle must not make other sounds similar to the Ready to
Drive sound.

VCU: the buzzer only sounds on the way into Ready to Drive. **met**

## Not the firmware's job

Rules around the VCU that hardware or other boards meet, listed so it's
clear they were considered: T.4.2.1-2, T.4.2.7-8 and T.4.3.1-2 (pedal
hardware, connectors for inspection), T.4.2.11 / T.4.3.5 (digital pedal
signals: ours are analogue), EV.3.4.2-3 (Energy Meter data and tampering), EV.6.2 (Ready to Move
light: hard-wired, software control not permitted), EV.8 (shutdown circuit,
BSPD: hardware; the VCU only reads the BSPD's state), EV.10.4.2 (TS active:
the shutdown circuit and accumulator).
