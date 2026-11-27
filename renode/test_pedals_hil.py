"""The pedal rules, checked on the real firmware in Renode.

The native tests (test/test_pedals) check the pedal logic on its own; these
check the whole chain on the emulated VCU: a voltage, or a wiring fault, at
the J1 connector, through the board's front end (renode/board.py: the
schematic's pull-downs, dividers and ADC ranges), the real ADS1018 and ADC
drivers, the firmware's loop, out to the torque command the inverter gets
on vcan_inv. The rest of the car is renode/fakecar.py.

Timing is counted in torque commands, which the VCU sends every 5 ms in
Ready to Drive: virtual time, so independent of how fast the host is. The
exact 100 ms boundaries are the native tests' job; here a fault must zero
the torque within 100 ms plus a command or two of slack for when the input
change lands.

    tools/run_e2e.sh renode/test_pedals_hil.py
"""
import shutil

import pytest

import board
from conftest import needs_vcan
from bench import (COMMAND_PERIOD_MS, MAX_TORQUE_NM, WALL_TIMEOUT, Bench,
                   drive_to_rtd)
from fakecar import READY_TO_DRIVE, TRACTIVE_SYSTEM_FAULT, FakeCar
from harness import BUILD_A, Renode

pytestmark = [
    needs_vcan,
    pytest.mark.skipif(shutil.which("renode") is None, reason="needs renode (nix develop)"),
    pytest.mark.skipif(not (BUILD_A / "firmware.hex").exists(),
                       reason="build it with: pio run -e teensy41"),
]

# Real time (not marked faster_than_realtime), like a bench: the fake boards
# on the host send every 100 ms of wall time, and the VCU times out a board
# that goes quiet for SAFETY_DATA_TIMEOUT_MS of its time. Running ahead of
# the host clock would make them look quiet.
@pytest.fixture(scope="module")
def bench(tmp_path_factory, renode_pacing):
    renode = Renode(tmp_path_factory.mktemp("renode") / "renode.log", renode_pacing)
    car = FakeCar()
    try:
        b = Bench(renode, car)
        drive_to_rtd(b)
        yield b
    finally:
        car.close()
        renode.stop()
        print(f"\n  renode log: {renode.log}")


def test_ready_to_drive_from_the_dash(bench):
    """[EV.10.6.2] reached with TS up, the brake held and the RTD button"""
    assert bench.car.status["vcu_statemachine_state"] == READY_TO_DRIVE
    assert bench.car.commands[-1].enable


def test_throttle_makes_torque(bench):
    """[EV.10.6.1] the motor answers the APPS in ready to drive"""
    bench.pedals(throttle=0.5)
    assert bench.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)
    bench.pedals()
    assert bench.torque() == 0



#
# Faults. Each starts from 50% throttle in ready to drive and ends with the
# pedals released and working again.

# 100 ms of commands, and a couple for the input change to land
WITHIN_100_MS = 100 // COMMAND_PERIOD_MS + 2


@pytest.fixture
def driving(bench):
    bench.pedals(throttle=0.5)
    assert bench.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)
    yield bench
    bench.pedals()
    bench.wait_commands(10)


def cut_and_stays_cut(bench):
    n = bench.commands_until_zero(WITHIN_100_MS)
    assert n is not None, f"torque still on {WITHIN_100_MS} commands (100 ms) after the fault"
    assert all(c.torque_nm == 0 for c in bench.wait_commands(40)), "torque came back"


def recovers_after_release(bench):
    bench.pedals()
    bench.wait_commands(10)
    bench.pedals(throttle=0.5)
    assert bench.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)


@pytest.mark.parametrize("sensor, fault", [
    ("apps1", board.UNPLUGGED),
    ("apps2", board.UNPLUGGED),
    ("apps1", board.SHORT_TO_5V),
    ("apps2", board.SHORT_TO_GROUND),
    ("apps2", board.SHORT_TO_GLV),
    ("apps1", board.Input(0.05)),  # under the sensor's range, over 0
], ids=str)
def test_apps_wiring_fault(driving, sensor, fault):
    """[T.4.2.3][T.4.2.9][T.4.2.10] an APPS wiring fault cuts torque within 100 ms"""
    driving.apply(**{sensor: fault})
    cut_and_stays_cut(driving)
    recovers_after_release(driving)


@pytest.mark.parametrize("fault", [board.UNPLUGGED, board.SHORT_TO_5V, board.SHORT_TO_GLV],
                         ids=str)
def test_brake_sensor_fault(driving, fault):
    """[T.4.3.3][T.4.3.4] a brake sensor wiring fault cuts torque within 100 ms"""
    driving.apply(brake=fault)
    cut_and_stays_cut(driving)
    recovers_after_release(driving)


def test_apps_disagreement_latches(driving):
    """[T.4.2.4][T.4.2.5] APPS 20 points apart: cut, and latched past 100 ms"""
    driving.apply(apps2=board.apps2(0.3))
    cut_and_stays_cut(driving)
    driving.hold(150, apps2=board.apps2(0.3))
    # Agreeing again isn't enough: the driver has to lift off
    driving.pedals(throttle=0.5)
    assert all(c.torque_nm == 0 for c in driving.wait_commands(20))
    recovers_after_release(driving)


def test_brief_apps_disagreement_doesnt_latch(driving):
    """[T.4.2.5] a 50 ms disagreement cuts torque only while it lasts"""
    driving.hold(50, apps2=board.apps2(0.3))
    driving.pedals(throttle=0.5)
    assert driving.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)


def test_brake_and_throttle(driving):
    """[EV.4.7.1][EV.4.7.2] brake + 50% throttle: cut until the APPS is under 5%"""
    driving.apply(brake=board.brake(0.5))
    cut_and_stays_cut(driving)
    driving.apply(brake=board.brake(0))
    assert all(c.torque_nm == 0 for c in driving.wait_commands(20))
    driving.pedals(throttle=0.06)
    assert all(c.torque_nm == 0 for c in driving.wait_commands(20))
    driving.pedals(throttle=0.04)
    driving.wait_commands(10)
    driving.pedals(throttle=0.5)
    assert driving.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)


def test_brake_with_light_throttle(bench):
    """[EV.4.7.1] the brake with 20% throttle doesn't trip it"""
    bench.pedals(throttle=0.2, brake=0.5)
    assert bench.torque() == pytest.approx(0.2 * MAX_TORQUE_NM, rel=0.05)
    bench.pedals()
    bench.wait_commands(10)




#
# The other boards going quiet (no rule: SAFETY_DATA_TIMEOUT_MS)

@pytest.mark.parametrize("board_name", ["acu", "inverter"])
def test_a_silent_board_stops_the_torque(bench, board_name):
    """The ACU or inverter going quiet: the fault state, no more torque"""
    bench.pedals(throttle=0.5)
    assert bench.torque() > 100
    setattr(bench.car, f"{board_name}_talking", False)
    try:
        if not bench.car.wait_state(TRACTIVE_SYSTEM_FAULT, WALL_TIMEOUT):
            bench.renode.fail(f"no fault state with a silent {board_name}: {bench.car.status}")
        # From here it only pings the inverter: nothing asked, disabled
        assert all(c.torque_nm == 0 and not c.enable for c in bench.wait_commands(3))
    finally:
        setattr(bench.car, f"{board_name}_talking", True)
        bench.pedals()
    drive_to_rtd(bench)
