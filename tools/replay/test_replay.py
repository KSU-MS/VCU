"""The 4wd_dti VCU through 31 s of a kmslap log (michigan2026, a launch, top
speed, two stops, corners): data/michigan2026_4wd_dti.csv.gz, made with

    uv run tools/replay/replay.py <kmslap mcap> --start 3.5 --duration 31.5 \\
        --save-inputs excerpt.csv && gzip -9 -n excerpt.csv

These check what has to hold whatever split kmslap used; replay.py's report
is the comparison itself. Needs `pio run -e replay_4wd_dti` (skipped
without it).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import replay  # noqa: E402

LOG = Path(__file__).parent / "data/michigan2026_4wd_dti.csv.gz"
PEAK_NM = 31.6          # DTI F-MOT (include/motors.hpp)
PEAK_POWER_W = 35e3
MAX_RPM = 20000
EFFICIENCY = 0.90       # what the replay's DC currents assume

pytestmark = pytest.mark.skipif(not replay.PROGRAM.exists(),
                                reason="build it with: pio run -e replay_4wd_dti")


@pytest.fixture(scope="module")
def run():
    inputs = replay.read_csv(LOG)
    return inputs, replay.run_vcu(inputs, efficiency=EFFICIENCY)


def torques(vcu):
    return np.stack([vcu[f"torque_{w}"] for w in replay.WHEELS])


def test_drives_the_whole_log_without_a_fault(run):
    """[EV.10.6.2] Ready to drive from start to end on real-rate traffic"""
    _, vcu = run
    assert np.all(vcu["state"] == replay.READY_TO_DRIVE)
    assert not np.any(vcu["fault"])
    assert np.all(np.stack([vcu[f"enable_{w}"] for w in replay.WHEELS]) == 1)


def test_no_torque_with_the_brake_on_and_no_regen(run):
    inputs, vcu = run
    t = torques(vcu)
    assert t.min() >= 0               # no regen yet
    braking = inputs["brake"] > 0.02  # kmslap brakes with the throttle off
    assert braking.any()
    assert t[:, braking].max() == pytest.approx(0, abs=0.05)


def test_every_motor_within_its_torque_curve(run):
    inputs, vcu = run
    t = torques(vcu)
    for i, w in enumerate(replay.WHEELS):
        rpm = np.abs(inputs[f"rpm_{w}"])
        omega = np.maximum(rpm * 2 * np.pi / 60, 1e-6)
        curve = np.where(rpm >= MAX_RPM, 0.0, np.minimum(PEAK_NM, PEAK_POWER_W / omega))
        assert np.all(t[i] <= curve + 0.05), w


def test_80_kw_across_the_four(run):
    """[EV.3.4.1] No 500 ms average over 80 kW, nor 100 ms straight over it"""
    inputs, vcu = run
    t = torques(vcu)
    omega = np.stack([inputs[f"rpm_{w}"] for w in replay.WHEELS]) * 2 * np.pi / 60
    watts = (t * omega).sum(0) / EFFICIENCY
    period = np.median(np.diff(inputs["t_s"]))
    over = watts > 80e3
    longest = max((int(r.sum()) for r in np.split(over, np.flatnonzero(~over))), default=0)
    assert longest * period < 0.1
    n = int(round(0.5 / period))
    assert np.convolve(watts, np.ones(n) / n, "valid").max() <= 80e3
    assert watts.max() > 0.8 * 80e3   # and it got near the limit


def test_split_by_load_close_to_kmslap(run):
    """The front axle's share while driving: kmslap's QP and the VCU's load
    split both follow the axle loads. Not closer than 7 points: the QP
    weighs grip and yaw too, and puts more on the front (48 % here to the
    VCU's 43)"""
    inputs, vcu = run
    report = replay.compare(inputs, vcu, EFFICIENCY)
    assert report.drive_rows > 1000
    assert report.drive_front_vcu == pytest.approx(report.drive_front_ref, abs=0.07)


def test_power_follows_the_split(run):
    """[EV.3.3.1] With one loop on the total, a loaded hub takes more than a
    quarter of 80 kW (kmslap's go to ~27 kW here)"""
    inputs, vcu = run
    report = replay.compare(inputs, vcu, EFFICIENCY)
    assert max(report.hub_kw_vcu.values()) > 22
    assert report.total_kw_vcu <= 80
