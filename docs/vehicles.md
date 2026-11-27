# Vehicles: one firmware, five drivetrains

kmslap (python-data-analysis) simulates five drivetrains. The firmware is
built for one of them, named the same way everywhere: kmslap, ksu-ms-dbc's
`gen_tool/vehicles.json`, and `include/vehicle.hpp` here.

| Vehicle | Motors | Inverters | Builds |
|---|---|---|---|
| `rwd_emrax` | EMRAX 228 HV, chain + LSD (the car today) | Cascadia PM100DX | yes |
| `awd_emrax_dti` | EMRAX rear + 2 DTI F-MOT front hubs | PM100DX + 2 DTI HV500 | not yet |
| `awd_emrax_amk` | EMRAX rear + 2 AMK DD5 front hubs | PM100DX + AMK KW26 | not yet |
| `4wd_dti` | 4 DTI F-MOT hubs | 4 DTI HV500 | yes: tested in the sim, firmware builds |
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

The cars whose inverters aren't written yet (the AMK ones, and the EMRAX +
DTI mix) stop the build with an `#error`. `main.hpp` and the sim's `Car`
build each car's inverters from its table, and check there's one per
drive.

`env:native_4wd_dti` runs `test/4wd_dti/` with the 4wd_dti CAN library;
`env:native` ignores that folder. `env:teensy41_4wd_dti` builds its
firmware. CI does all three.

## Inverters

`include/inverter.hpp` is what the VCU needs from any inverter. The base
class holds the motor-independent parts:
- the torque clamps;
- the 80 kW power limit, open loop and trimmed on measured DC power;
- the current-limit arithmetic;
- the odometer.

A subclass supplies the wire protocol: decoding its feedback (`on_frame`),
sending torque, keep-alive and clearing faults.
- `CascadiaPM100` is the existing code, moved and otherwise unchanged.
- `DtiHv500` is current mode over DTI's CAN map v2.5, with extended IDs.
  Its frames are packed by hand, because dbcc names its functions per
  node; `test/4wd_dti` checks them against the DBC for every node.
  **Assumed:** the AC current command is in A peak, so
  I = sqrt(2) × T / kt. Confirm against DTI's manual.

## Several motors: `Drivetrain`

The VCU drives a `Drivetrain` of inverters (`include/drivetrain.hpp`).
- **Request units:** motor N·m on `rwd_emrax`, as always; total wheel N·m
  on the hub cars.
- **The split** (`torque_split.hpp`): kmslap's `load` mode by default,
  `fixed` (front 45 %) by `TORQUE_SPLIT_BY_LOAD 0` in `parameters.hpp`.
  - Each axle gets its share, capped at its motors' limits, with the
    overflow moved to the other axle.
  - Each motor gets its part of its axle, then it's converted through its
    ratio and gear efficiency.
  - The total is never more than the ask (EV.3.1.3).
  - The load split takes speed and acceleration from VectorNav, and falls
    back to the static weight when VectorNav is quiet.
- **80 kW:** shared by capability (20 kW per hub on 4wd_dti), each
  inverter closed loop on its own measured DC power. The sim judges the
  total by EV.3.4.1.
- **Safety:**
  - Any inverter silent is a fault, which takes the car to the fault
    state.
  - Every inverter must read the precharged voltage.
  - HV counts as present if any inverter is at 60 V or more, or any is
    silent.
- **Not yet on the hub cars:** traction control. It needs a rear-axle
  motor's speed, so on a 4wd car it steps aside. Per-wheel slip control is
  later.

## Still to do (from the hub motors handoff)

1. **`4wd_dti` against kmslap:** replay a `kmslap mcap --drivetrain
   4wd_dti` log into a 4wd_dti SIL and compare the VCU's hub commands with
   kmslap's.
   - Each motor's limit is its peak torque here; kmslap uses its torque
     curve against speed.
   - The DTI's broadcast rate should be 100 Hz, like the sim's.
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
