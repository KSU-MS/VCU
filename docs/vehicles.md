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

- **The motors' torque curve** (`motors::max_torque_nm`, kmslap's
  `HubMotor.max_torque`): peak torque up to peak power (35 kW on the F-MOT),
  then constant power, and none at top speed. Each drive's capability is
  the smaller of its torque limit and its curve at the speed its inverter
  reports, so full pedal asks for what the motors can give there, as in
  kmslap, and a motor at top speed gets nothing (its axle-mate and the
  other axle take its share). `rwd_emrax` has no curve: the Cascadia's path
  is unchanged.

## Checked against kmslap: the replay

`sil/replay_main.cpp` (`env:replay_4wd_dti`) runs the 4wd_dti VCU through a
drive log on simulated time, as fast as it goes. `tools/replay/replay.py`
makes the log from a `kmslap mcap --drivetrain 4wd_dti` file:
- what the rest of the car did (pedals, each inverter's voltage and speed,
  VectorNav, wheel speeds) on a 10 ms grid;
- kmslap's own hub commands (`vcu_motor_torque_commands`) beside it.

It runs the VCU and reports how the two compare:

    kmslap mcap michigan2026 --drivetrain 4wd_dti   # python-data-analysis
    pio run -e replay_4wd_dti
    uv run tools/replay/replay.py ../python-data-analysis/out/mcap/michigan2026_4wd_dti.mcap

The inverters' DC currents are the one input that has to answer the VCU:
its 80 kW loop closes on them. The log's currents answer kmslap's torques,
so by default the replay makes them from the VCU's own torques (at 90 %
efficiency). With the log's currents (`--logged-dc`) the loop sees power
it can't reduce and winds its torque down to the floor.

`tools/replay/test_replay.py` runs 31 s of that log
(`tools/replay/data/`) in CI and checks:
- ready to drive throughout, no faults;
- no torque with the brake on, no regen;
- every motor within its curve;
- EV.3.4.1 across all four;
- the front axle's share within 5 points of kmslap's.

**What the comparison shows** (michigan2026, the whole log):
- The front axle's share is close: VCU 44 %, kmslap 48 %.
- The torque curve brought the VCU nearer kmslap: the RMS difference in
  total motor torque while driving went from 13.4 to 12.2 N·m.
- kmslap's log comes from its QP mode, which the VCU doesn't have. It
  vectors torque (outside wheels up to ~28 kW, inside ones regenerating)
  and holds wheels back near their grip limit, so per motor the two differ
  by ~10 N·m RMS. The VCU gives more torque at launch and out of slow
  corners, where the QP is grip limited.
- About 18 % of the log is kmslap regenerating; the VCU gives nothing there.
- **The 80 kW share is fixed at 20 kW a hub.** kmslap puts 25–28 kW
  through the loaded hubs; the VCU can't. A share that follows the split
  would let it.

## Still to do (from the hub motors handoff)

1. **The 80 kW share following the torque split**, not a fixed 20 kW a
   hub (above). And `kmslap mcap --torque-mode load`, for a log in the
   VCU's own split.
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
