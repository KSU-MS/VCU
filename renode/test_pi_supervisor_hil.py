"""The Pi's torque ceiling, checked on the real firmware in Renode.

The native tests (test/test_pi_supervisor) check the rules; these check the
emulated VCU takes the Pi's frames off vcan_daq, CRC and counter included,
and that its torque command on vcan_inv follows: limited by the Pi while
it's in control, back to the driver's when it goes quiet or sends a bad
frame. The Pi is renode/fakepi.py, the rest of the car renode/fakecar.py.

    tools/run_e2e.sh renode/test_pi_supervisor_hil.py
"""
import shutil

import pytest

from bench import COMMAND_PERIOD_MS, MAX_TORQUE_NM, Bench, drive_to_rtd
from conftest import needs_vcan
from fakecar import FakeCar
from fakepi import LOCAL, PI, FakePi, crc8
from harness import BUILD_A, Renode

pytestmark = [
    needs_vcan,
    pytest.mark.skipif(shutil.which("renode") is None, reason="needs renode (nix develop)"),
    pytest.mark.skipif(not (BUILD_A / "firmware.hex").exists(),
                       reason="build it with: pio run -e teensy41"),
]

# vcu_pi_status reasons (PiSupervisor::reason)
SILENT, BAD_CHECKSUM = 1, 3
PI_RAMP_NM_PER_S = 1000  # PI_FALLBACK_RAMP_NM_PER_S


def test_crc_matches_the_firmware():
    # CRC-8 SAE J1850's check value, with "12" as the ID like the native test
    assert crc8(ord("1") | ord("2") << 8, b"3456789") == 0x4B


# Real time like the pedal tests: the fake boards and the Pi send by the
# host's clock
@pytest.fixture(scope="module")
def bench(tmp_path_factory, renode_pacing):
    renode = Renode(tmp_path_factory.mktemp("renode") / "renode.log", renode_pacing,
                    monitor_port=3458)
    car = FakeCar()
    pi = FakePi()
    try:
        b = Bench(renode, car)
        b.pi = pi
        drive_to_rtd(b)
        yield b
    finally:
        pi.close()
        car.close()
        renode.stop()
        print(f"\n  renode log: {renode.log}")


@pytest.fixture
def pi_in_control(bench):
    """The Pi healthy and in control at a 60 Nm ceiling, driver off throttle"""
    pi = bench.pi
    pi.ceiling_nm, pi.engage, pi.talking = 60, True, True
    bench.pedals()
    if not pi.wait_mode(PI, 30):
        bench.renode.fail(f"the Pi never got control (status {pi.status})")
    yield bench
    pi.talking = False
    bench.pedals()
    bench.wait_commands(20)


def test_the_pis_ceiling_limits_the_torque(pi_in_control):
    b = pi_in_control
    b.pedals(throttle=0.8)
    assert b.torque() == pytest.approx(60, abs=0.5)
    assert b.pi.status["vcu_pi_driver_torque"] == pytest.approx(0.8 * MAX_TORQUE_NM, rel=0.03)


def test_under_the_ceiling_the_driver_has_it(pi_in_control):
    b = pi_in_control
    b.pi.ceiling_nm = 200
    b.pedals(throttle=0.5)
    assert b.torque() == pytest.approx(0.5 * MAX_TORQUE_NM, rel=0.03)


def test_the_pi_going_quiet_ramps_back_to_the_driver(pi_in_control):
    b = pi_in_control
    b.pedals(throttle=0.8)
    assert b.torque() == pytest.approx(60, abs=0.5)

    b.pi.talking = False
    commands = b.wait_commands(100)  # 500 ms
    torques = [c.torque_nm for c in commands]
    steps = [after - before for before, after in zip(torques, torques[1:])]
    assert min(steps) > -0.5, "the torque went down"
    assert max(steps) <= PI_RAMP_NM_PER_S * COMMAND_PERIOD_MS / 1000 + 1, "it stepped"
    assert torques[-1] == pytest.approx(0.8 * MAX_TORQUE_NM, rel=0.03)
    assert b.pi.status["vcu_pi_mode"] == LOCAL
    assert b.pi.status["vcu_pi_reason"] == SILENT


def test_a_bad_checksum_hands_control_back(pi_in_control):
    b = pi_in_control
    b.pedals(throttle=0.8)
    assert b.torque() == pytest.approx(60, abs=0.5)
    rejected = b.pi.status["vcu_pi_rejected_frames"]

    b.pi.corrupt_next = True
    assert b.pi.wait_mode(LOCAL, 30)
    assert b.pi.status["vcu_pi_rejected_frames"] == rejected + 1
    # Good frames again, but it stays the driver's until a lift
    assert b.torque() == pytest.approx(0.8 * MAX_TORQUE_NM, rel=0.03)
