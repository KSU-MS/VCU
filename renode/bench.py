"""The emulated VCU on the bench: its pedal inputs, the fake car on CAN,
and getting it to Ready to Drive. Shared by the Renode tests that drive it."""
import time

import board
from fakecar import READY_TO_DRIVE, FakeCar
from harness import Renode

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
        """Settled torque command: 200 ms on, so nothing from before the
        last input change is still on its way"""
        return self.wait_commands(40)[-1].torque_nm

    def commands_until_zero(self, within: int) -> int | None:
        """How many commands until one asks for no torque (None: not within)"""
        for i, c in enumerate(self.wait_commands(within)):
            if c.torque_nm == 0:
                return i
        return None


def drive_to_rtd(b: Bench):
    """The driver's start-up: TS up, brake held, RTD button"""
    car = b.car
    car.ts_healthy = True
    b.pedals(brake=0.6)
    car.rtd_button = True
    if not car.wait_state(READY_TO_DRIVE, 4 * WALL_TIMEOUT):
        b.renode.fail(f"never reached ready to drive (status {car.status})")
    car.rtd_button = False
    b.pedals()
    b.wait_commands(20)
