"""The 4wd_dti VCU through 31 s of a kmslap log (michigan2026 at 80 % charge,
so there's room to regenerate under 600 V: a launch, top speed, two stops,
corners): data/michigan2026_4wd_dti_soc80.csv.gz, made with

    kmslap mcap michigan2026 --drivetrain 4wd_dti --soc 0.8   # kmslap
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

LOG = Path(__file__).parent / "data/michigan2026_4wd_dti_soc80.csv.gz"
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


def test_the_brake_regenerates_and_never_drives(run):
    """[T.3.2.4] Braking, every hub regenerates or does nothing: no drive"""
    inputs, vcu = run
    t = torques(vcu)
    braking = inputs["brake"] > 0.05   # kmslap brakes with the throttle off
    assert braking.sum() > 300
    assert t[:, braking].max() <= 0.05
    assert t[:, braking].sum(0).mean() < -10   # and it does regenerate


def test_no_regen_at_5_kmh_or_less(run):
    """[EV.3.3.3] None at or under 5 km/h on VectorNav's speed"""
    inputs, vcu = run
    t = torques(vcu)
    slow = np.hypot(inputs["vn_n"], inputs["vn_e"]) <= 5 / 3.6
    assert slow.any()
    assert t[:, slow].min() >= -0.05


def test_regen_like_kmslaps(run):
    """Where kmslap regenerates, the VCU does too, about as much (kmslap's QP
    holds back for grip: the VCU's is ~35 % more here), more on the front"""
    inputs, vcu = run
    report = replay.compare(inputs, vcu, EFFICIENCY)
    assert report.regen_rows > 300
    assert report.regen_vcu_mean == pytest.approx(report.regen_ref_mean, rel=0.5)
    assert report.regen_front_vcu > report.drive_front_vcu + 0.1


def test_every_motor_within_its_torque_curve(run):
    inputs, vcu = run
    t = torques(vcu)
    for i, w in enumerate(replay.WHEELS):
        rpm = np.abs(inputs[f"rpm_{w}"])
        omega = np.maximum(rpm * 2 * np.pi / 60, 1e-6)
        curve = np.where(rpm >= MAX_RPM, 0.0, np.minimum(PEAK_NM, PEAK_POWER_W / omega))
        assert np.all(t[i] <= curve + 0.05), w
        assert np.all(t[i] >= -curve - 0.05), w   # regenerating too


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
    VCU's 42)"""
    inputs, vcu = run
    report = replay.compare(inputs, vcu, EFFICIENCY)
    assert report.drive_rows > 1000
    assert report.drive_front_vcu == pytest.approx(report.drive_front_ref, abs=0.07)


def test_power_follows_the_split(run):
    """[EV.3.3.1] With one loop on the total, a loaded hub takes more than a
    quarter of 80 kW (kmslap's go to ~21 kW here, ~25 at full charge)"""
    inputs, vcu = run
    report = replay.compare(inputs, vcu, EFFICIENCY)
    assert max(report.hub_kw_vcu.values()) > 22
    assert report.total_kw_vcu <= 80
