# Vehicles: one firmware, five drivetrains

kmslap (python-data-analysis) simulates five drivetrains. The firmware is
built for one of them, named the same way everywhere: kmslap, ksu-ms-dbc's
`gen_tool/vehicles.json`, and `include/vehicle.hpp` here.

| Vehicle | Motors | Inverters | Builds |
|---|---|---|---|
| `rwd_emrax` | EMRAX 228 HV, chain + LSD (the car today) | Cascadia PM100DX | yes |
| `awd_emrax_dti` | EMRAX rear + 2 DTI F-MOT front hubs | PM100DX + 2 DTI HV500 | not yet |
| `awd_emrax_amk` | EMRAX rear + 2 AMK DD5 front hubs | PM100DX + AMK KW26 | not yet |
| `4wd_dti` | 4 DTI F-MOT hubs | 4 DTI HV500 | not yet |
| `4wd_amk` | 4 AMK DD5 hubs | AMK KW26 (4-in-1) | not yet |

## How a build picks one

`platformio.ini` has a `[vehicle_<name>]` section for each car. Each one
sets `-D VEHICLE_<NAME>` and the car's CAN library, `can_lib_<name>.tar.gz`
from ksu-ms-dbc's release, with the same `car.h` API whichever car it is.
`[vehicle]` says which one the envs build. `include/vehicle.hpp` describes
all five as data:
- each motor's position;
- its inverter kind;
- the motor (`include/motors.hpp`, kmslap's numbers);
- the gear ratio;
- the inverter's CAN node ID.

It also selects `vehicle::THIS` for the build. The VCU logs the vehicle's
name at boot.

The hub cars stop the build with an `#error` until their inverters exist.
`main.hpp` also checks at compile time that the chosen car is one it knows
how to wire up.

## Inverters

`include/inverter.hpp` is what the VCU needs from any inverter. The base
class holds the motor-independent parts:
- the torque clamps;
- the 80 kW power limit, open loop and trimmed on measured DC power;
- the current-limit arithmetic;
- the odometer.

A subclass supplies the wire protocol: decoding its feedback (`on_frame`),
sending torque, keep-alive and clearing faults. `CascadiaPM100` is the
existing code, moved and otherwise unchanged.

## Still to do (from the hub motors handoff)

1. **One hub car end to end, `4wd_dti`:**
   - the DTI HV500 inverter (current mode, extended IDs);
   - a VCU with several inverters;
   - torque split: fixed or load-based, never more than the driver's
     request in total, each motor within its curve, 80 kW across all of
     them;
   - a timeout per inverter.

   Tested against a kmslap simulated log replayed into the SIL.
2. **The Pi's per-motor torques** (`pi_motor_torque_command`, 0x11), on the
   same checks as `pi_torque_command`.
3. **AMK KW26:** the enable handshake, torque by saturating the speed loop,
   setpoints at least every 10 ms.
4. **Regen:**
   - pure-regen brake travel (T.3.2.4);
   - none under 5 km/h (EV.3.3.3);
   - capped by the BMS charge limit;
   - rear-only regen only behind slip control.

Open questions are in the handoff: DTI current units (peak or rms), AMK
current scaling, node IDs and buses, and the hub gearbox ratio.
