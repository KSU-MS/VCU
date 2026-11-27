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
import time

import pytest

import board
from conftest import needs_vcan
from fakecar import READY_TO_DRIVE, FakeCar
from harness import BUILD_A, Renode

pytestmark = [
    needs_vcan,
    pytest.mark.skipif(shutil.which("renode") is None, reason="needs renode (nix develop)"),
    pytest.mark.skipif(not (BUILD_A / "firmware.hex").exists(),
                       reason="build it with: pio run -e teensy41"),
]

COMMAND_PERIOD_MS = 5
MAX_TORQUE_NM = 240
A10_INPUT = 0x400C4000 + 0x1000 + 4 * 1  # imxrt_adc.py, A10 is ADC1 channel 1
# Generous: the emulated VCU's speed relative to the host varies
WALL_TIMEOUT = 180


class Bench:
    """The emulated VCU with its pedal inputs on the bench"""

    def __init__(self, renode: Renode, car: FakeCar):
        self.renode, self.car = renode, car
        self.pins = {"apps1": board.apps1(0), "apps2": board.apps2(0), "brake": board.brake(0)}
        self.apply()

    def _set(self, **pins: board.Input):
        # Paused, so they all change at the same instant of virtual time
        # however slow the host is (the emulator runs faster than real time)
        self.pins.update(pins)
        m = self.renode.monitor
        m.run("pause")
        m.run(f"sysbus.lpspi4.ads1018 AIN0 {board.ads1018_counts(self.pins['apps1'])}")
        m.run(f"sysbus.lpspi4.ads1018 AIN1 {board.ads1018_counts(self.pins['apps2'])}")
        m.run(f"sysbus WriteDoubleWord {A10_INPUT:#x} {board.a10_counts(self.pins['brake'])}")

    def apply(self, **pins: board.Input):
        """Change the inputs and carry on"""
        self._set(**pins)
        self.renode.monitor.run("start")

    def hold(self, ms: int, **pins: board.Input):
        """These inputs for exactly `ms` of virtual time, then stay paused:
        the next apply() takes over at that instant"""
        self._set(**pins)
        self.renode.monitor.run(f'emulation RunFor "00:00:{ms / 1000:06.3f}"')

    def pedals(self, throttle: float = 0.0, brake: float = 0.0):
        """Working sensors, at these positions"""
        self.apply(apps1=board.apps1(throttle), apps2=board.apps2(throttle),
                   brake=board.brake(brake))

    def wait_commands(self, n: int):
        """Let the firmware send n more torque commands; returns them"""
        start = self.car.command_count()
        deadline = time.monotonic() + WALL_TIMEOUT
        while self.car.command_count() < start + n:
            if self.renode.proc.poll() is not None:
                self.renode.fail("renode exited")
            if time.monotonic() > deadline:
                self.renode.fail(f"VCU stopped sending torque commands after {start}")
            time.sleep(0.05)
        return self.car.commands_since(start)[:n]

    def torque(self) -> float:
        """Settled torque command"""
        return self.wait_commands(10)[-1].torque_nm

    def commands_until_zero(self, within: int) -> int | None:
        """How many commands until one asks for no torque (None: not within)"""
        for i, c in enumerate(self.wait_commands(within)):
            if c.torque_nm == 0:
                return i
        return None


@pytest.fixture(scope="module")
def bench(tmp_path_factory):
    renode = Renode(tmp_path_factory.mktemp("renode") / "renode.log")
    car = FakeCar()
    try:
        b = Bench(renode, car)
        # The driver's start-up: TS up, brake held, RTD button
        car.ts_healthy = True
        b.pedals(brake=0.6)
        deadline = time.monotonic() + 4 * WALL_TIMEOUT
        car.rtd_button = True
        while car.status.get("vcu_statemachine_state") != READY_TO_DRIVE:
            if time.monotonic() > deadline:
                renode.fail(f"never reached ready to drive (status {car.status})")
            time.sleep(0.2)
        car.rtd_button = False
        b.pedals()
        b.wait_commands(20)
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


