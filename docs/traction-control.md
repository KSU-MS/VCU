# Traction control

The VCU runs kmslap's launch/traction controller between the pedal map and
the inverter: `pedal torque -> traction control -> power and torque limits
-> inverter`. A launch is just traction control from a standstill, so the
old `LAUNCH_WAIT`/`LAUNCH` states (never more than stubs) are gone.

- `lib/vcu_control/`: the controller, vendored from
  `python-data-analysis/controller/` (kmslap). It's designed and tuned there,
  in the loop of kmslap's accel sim. See its README for the one local change.
- `include/traction_control.hpp`, `src/traction_control.cpp`: the VCU end.
  Its inputs, the on/off rules, and the guarantee that it only takes torque
  away.
- `test/test_traction/`: the VCU end against a simple launch model
  (`lib/vcu_sim/sim_launch.hpp`).

## Modes

| `TRACTION_MODE` | Mode | What it does |
|---|---|---|
| 0 | off | Driver torque straight through (the default, until it's been checked on the car) |
| 1 | slip | PI on rear slip (10% target), scaling the driver's torque |
| 2 | grip map | Torque capped at what the rear tires can take, from a learned friction coefficient and the rear load, plus the slip PI as trim. kmslap's own default |

## Turning it on and off

- **Dash button 6** toggles between off and the last mode that was on
  (slip, to begin with). One press, one toggle: holding it does nothing more.
- **Over CAN:** `vcu_set_parameter` (0x0C9), `TRACTION_MODE` (6), value 0-2.
  Anything else is ignored.

A change only takes effect off the throttle, so the torque never steps in
the middle of a corner. Until then the status frame says
`waiting_for_lift`.

## When it steps aside

It needs ground speed (VectorNav velocity, `0x1F7`) and rear wheel speed
(motor speed, inverter `0x0A5`). If either is older than
`TC_INPUT_TIMEOUT_MS` (200 ms) it can't control blind. The driver gets
their torque back straight away, like a car without TC, and the status frame
says which one is missing. Once both are back it waits for the driver to
lift, then takes over again.

The controller also checks the final drive: motor revs over rear wheel revs
(the AN1 wheel speed nodes, `0x38E`-`0x391`) must match `GEAR_RATIO`
within 8%. If not, `GEAR_RATIO` is wrong, or so is a sensor, and slip
would be wrong too. It turns itself off (`gear_ratio_fault`) until it's
chosen again.

Front wheel speeds and VectorNav acceleration improve its speed estimate,
but it runs without them.

Whatever the controller says, the VCU never sends more than the driver
asked for (tested with 2000 random inputs). APPS/BSE plausibility, the
power limit and the torque limit all still apply after it.

## Status frame

`0x0CC` on the DAQ bus at 20 Hz: mode chosen and active, the reasons above,
slip, ground speed, torque scale, learned grip. `vcu_traction_control` in
ksu-ms-dbc's `vcu-traction-pi` branch, which the firmware pins until it's
merged.

## Before it goes on the car

- **Final drive.** `GEAR_RATIO` in `parameters.hpp` is 4.18, but kmslap's
  car model says 3.0. Slip is motor rpm over the final drive, so one of
  them is wrong. With the wrong one the ratio check turns TC off within a
  few metres, so it fails safe, but it needs settling.
- **Tune the launch in kmslap with this car's numbers.** With the VCU's
  sensor rates (inverter and wheel speeds 100 Hz, VectorNav 50 Hz) and the
  4.18 final drive, both modes spin the tires for the first ~0.3 s of a
  full-throttle launch before they catch it. In the VCU's simple model
  that's slip up to ~2 at 0.2 m/s, then the slip PI cuts to zero and
  recovers over ~0.3 s. After that they hold 10% slip closely. kmslap's
  gains were swept for a 3.0 final drive, which puts 40% less torque on the
  wheel per Nm of motor torque. `test/test_traction` only checks from 1 s
  on.
- **VectorNav and wheel speed node rates** on the real buses, and which bus
  they're on. The VCU reads them from any of its three.
- **Learned values aren't saved** across power cycles yet (radii, accel bias,
  grip: `vcu_learned_t`). kmslap's design says EEPROM.
- **DBC:** merge ksu-ms-dbc's `vcu-traction-pi` branch (0x0CC, dash button
  6's meaning), then pin a numbered release again.
