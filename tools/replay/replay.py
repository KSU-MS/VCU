"""A kmslap drive log played into the 4wd_dti VCU, and its hub commands
compared with the ones kmslap modelled.

    kmslap mcap michigan2026 --drivetrain 4wd_dti     # in python-data-analysis
    pio run -e replay_4wd_dti
    uv run tools/replay/replay.py out/mcap/michigan2026_4wd_dti.mcap

The log is in the car logger's format: one protobuf topic per CAN message,
signals by their DBC names. What the rest of the car did (pedals, the
inverters' feedback, VectorNav, wheel speeds) goes in on a 10 ms grid,
each signal holding its last logged value; sil/replay_main.cpp runs the VCU
through it on simulated time. kmslap's commands are its
vcu_motor_torque_commands, N m at each motor.

The two aren't expected to agree everywhere: kmslap's log may come from a
different torque split (its QP by default) and has regen, which the VCU
doesn't do yet. The report says where they differ and by how much.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PROGRAM = ROOT / ".pio/build/replay_4wd_dti/program"
WHEELS = ("fl", "fr", "rl", "rr")
POLE_PAIRS = 4        # DTI F-MOT (include/motors.hpp)
HUB_RATIO = 13.0      # include/vehicle.hpp
HUB_GEAR_EFF = 0.97   # include/vehicle.hpp
READY_TO_DRIVE = 4

# input column: (topic, field, scale)
INPUTS = {
    "throttle": ("vcu_pedals_travel_data", "vcu_apps1_travel", 0.01),
    "brake": ("vcu_pedals_travel_data", "vcu_bse_travel", 0.01),
    "pack_v": ("msgid_0x6b1_data", "Pack_Inst_Voltage", 1.0),
    "vn_n": ("vectornav_velocity_data", "vn_v_n", 1.0),
    "vn_e": ("vectornav_velocity_data", "vn_v_e", 1.0),
    "vn_ax": ("vectornav_acceleration_data", "vn_accel_x", 1.0),
    **{k: v for w in WHEELS for k, v in {
        f"bus_v_{w}": (f"inv_{w}_erpm_duty_voltage_data", "Actual_InputVoltage", 1.0),
        f"rpm_{w}": (f"inv_{w}_erpm_duty_voltage_data", "Actual_ERPM", 1.0 / POLE_PAIRS),
        f"dc_a_{w}": (f"inv_{w}_ac_dc_current_data", "Actual_DCCurrent", 1.0),
        f"wheel_rpm_{w}": (f"an1_{w}_wheelspeed_data", "wheel_rpm", 1.0),
    }.items()},
}
REFERENCE = {f"ref_{w}": ("vcu_motor_torque_commands_data", f"vcu_motor_torque_{w}", 1.0) for w in WHEELS}


def read_mcap(path: Path, columns: dict, period: float = 0.01) -> dict[str, np.ndarray]:
    """The columns on a grid of `period` s from the log's start, each holding
    its last logged value (its first before it's been logged). Adds t_s."""
    from mcap.reader import make_reader
    from mcap_protobuf.decoder import DecoderFactory

    wanted = {}
    for name, (topic, field, scale) in columns.items():
        wanted.setdefault(topic, []).append((name, field, scale))
    series = {name: ([], []) for name in columns}
    t_first = None
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, channel, message, decoded in reader.iter_decoded_messages(topics=list(wanted)):
            t = message.log_time * 1e-9
            t_first = t if t_first is None else min(t_first, t)
            for name, field, scale in wanted[channel.topic]:
                # proto3 leaves zeros out: a missing field is 0
                series[name][0].append(t)
                series[name][1].append(float(getattr(decoded, field, 0.0)) * scale)
    missing = [n for n, (ts, _) in series.items() if not ts]
    if missing:
        raise ValueError(f"{path}: nothing logged for {missing}")
    t_last = max(ts[-1] for ts, _ in series.values())
    grid = np.arange(t_first, t_last, period)
    out = {"t_s": grid - t_first}
    for name, (ts, vs) in series.items():
        ts, vs = np.asarray(ts), np.asarray(vs)
        order = np.argsort(ts, kind="stable")
        ts, vs = ts[order], vs[order]
        idx = np.clip(np.searchsorted(ts, grid, side="right") - 1, 0, len(ts) - 1)
        out[name] = vs[idx]
    return out


def window(cols: dict, start: float, duration: float | None) -> dict:
    t = cols["t_s"]
    keep = (t >= start) & ((t < start + duration) if duration is not None else True)
    return {k: v[keep] for k, v in cols.items()}


def write_csv(path: Path, cols: dict):
    names = list(cols)
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(names)
        for row in zip(*(cols[n] for n in names)):
            w.writerow([f"{x:.4g}" if n != "t_s" else f"{x:.3f}" for n, x in zip(names, row)])


def read_csv(path: Path) -> dict[str, np.ndarray]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", newline="") as f:
        rows = list(csv.reader(f))
    return {name: np.array([float(r[i]) for r in rows[1:]]) for i, name in enumerate(rows[0])}


def run_vcu(inputs: dict, program: Path = PROGRAM, logged_dc: bool = False,
            efficiency: float = 0.90) -> dict[str, np.ndarray]:
    """The VCU through `inputs` (INPUTS' columns and t_s): its state, fault
    and each hub's torque (N m at the motor) and enable, row by row.
    logged_dc: the inverters report the log's DC currents, not ones made from
    the VCU's own torques at `efficiency` (sil/replay_main.cpp)."""
    if not program.exists():
        raise FileNotFoundError(f"{program}: build it with `pio run -e replay_4wd_dti`")
    with tempfile.TemporaryDirectory() as tmp:
        src, dst = Path(tmp) / "in.csv", Path(tmp) / "out.csv"
        write_csv(src, {k: inputs[k] for k in ["t_s", *INPUTS]})
        flags = ["--logged-dc"] if logged_dc else ["--efficiency", str(efficiency)]
        subprocess.run([str(program), *flags, str(src), str(dst)], check=True)
        return read_csv(dst)


@dataclass
class Report:
    rows: int
    ready_to_drive: float          # share of the rows in ready to drive
    faults: list                   # fault codes seen
    drive_rows: int                # kmslap driving, not braking
    drive_total_vcu: float         # mean total motor torque, N m, over those
    drive_total_ref: float
    drive_total_rms: float         # RMS of the difference
    drive_front_vcu: float         # front axle's share of the torque
    drive_front_ref: float
    regen_rows: int                # kmslap regenerating
    regen_ref_mean: float          # its mean total (negative), N m
    vcu_negative_max: float        # the most regen the VCU asked for (should be 0)
    vcu_torque_on_brake: float     # most total torque the VCU gave with the brake on
    overlap_rows: int              # throttle > 25 % with the brake on (EV.4.7)
    per_wheel_rms: dict

    def text(self) -> str:
        lines = [
            f"{self.rows} rows, {self.ready_to_drive:.1%} of them in ready to drive"
            + (f", faults {sorted(set(self.faults))}" if self.faults else ", no faults"),
            f"driving ({self.drive_rows} rows): total motor torque VCU {self.drive_total_vcu:.1f} N m, "
            f"kmslap {self.drive_total_ref:.1f} N m, RMS difference {self.drive_total_rms:.1f} N m",
            f"  front axle's share: VCU {self.drive_front_vcu:.1%}, kmslap {self.drive_front_ref:.1%}",
            "  per motor RMS difference: " + ", ".join(f"{w} {v:.1f}" for w, v in self.per_wheel_rms.items()),
            f"regen ({self.regen_rows} rows): kmslap averages {self.regen_ref_mean:.1f} N m; "
            f"the VCU's most negative {self.vcu_negative_max:.1f} N m (no regen yet)",
            f"brake on: the most total torque the VCU gave {self.vcu_torque_on_brake:.1f} N m",
            f"throttle over 25 % with the brake on: {self.overlap_rows} rows",
        ]
        return "\n".join(lines)


def compare(inputs: dict, vcu: dict) -> Report:
    vcu_t = np.stack([vcu[f"torque_{w}"] for w in WHEELS])
    ref_t = np.stack([inputs[f"ref_{w}"] for w in WHEELS])
    vcu_total, ref_total = vcu_t.sum(0), ref_t.sum(0)
    brake_on = inputs["brake"] > 0.02
    driving = (ref_total > 1.0) & ~brake_on
    regen = ref_total < -1.0

    def front_share(t, rows):
        total = t[:, rows].sum()
        return float(t[:2, rows].sum() / total) if total > 0 else float("nan")

    def mean(x):
        return float(x.mean()) if x.size else 0.0

    return Report(
        rows=len(vcu_total),
        ready_to_drive=float(np.mean(vcu["state"] == READY_TO_DRIVE)),
        faults=[int(f) for f in np.unique(vcu["fault"]) if f],
        drive_rows=int(driving.sum()),
        drive_total_vcu=mean(vcu_total[driving]),
        drive_total_ref=mean(ref_total[driving]),
        drive_total_rms=float(np.sqrt(mean((vcu_total - ref_total)[driving] ** 2))),
        drive_front_vcu=front_share(vcu_t, driving),
        drive_front_ref=front_share(ref_t, driving),
        regen_rows=int(regen.sum()),
        regen_ref_mean=mean(ref_total[regen]),
        vcu_negative_max=float(min(vcu_t.min(), 0.0)),
        vcu_torque_on_brake=float(vcu_total[brake_on].max()) if brake_on.any() else 0.0,
        overlap_rows=int(((inputs["throttle"] > 0.25) & brake_on).sum()),
        per_wheel_rms={w: float(np.sqrt(mean((vcu_t[i] - ref_t[i])[driving] ** 2))) for i, w in enumerate(WHEELS)},
    )


def plot(inputs: dict, vcu: dict, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = inputs["t_s"]
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(14, 12))
    axes[0].plot(t, inputs["throttle"], label="throttle")
    axes[0].plot(t, inputs["brake"], label="brake")
    axes[0].legend(loc="upper right")
    for ax, w in zip(axes[1:], WHEELS):
        ax.plot(t, inputs[f"ref_{w}"], label="kmslap")
        ax.plot(t, vcu[f"torque_{w}"], label="VCU")
        ax.set_ylabel(f"{w} N m")
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("s")
    fig.tight_layout()
    fig.savefig(out)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", type=Path, help="a kmslap mcap (or a CSV saved with --save-inputs, .csv or .csv.gz)")
    p.add_argument("--start", type=float, default=0.0, help="s into the log")
    p.add_argument("--duration", type=float, help="s of it (default all)")
    p.add_argument("--save-inputs", type=Path, help="write the gridded inputs and reference here (CSV)")
    p.add_argument("--save-output", type=Path, help="write the VCU's commands here (CSV)")
    p.add_argument("--plot", type=Path, help="a PNG of the commands, side by side (needs matplotlib)")
    p.add_argument("--logged-dc", action="store_true",
                   help="the inverters report the log's DC currents (they don't answer the VCU's torques)")
    p.add_argument("--efficiency", type=float, default=0.90,
                   help="motor and inverter, for the DC currents made from the VCU's torques")
    args = p.parse_args(argv)

    if args.log.suffixes[-2:] == [".csv", ".gz"] or args.log.suffix == ".csv":
        inputs = read_csv(args.log)
    else:
        inputs = read_mcap(args.log, {**INPUTS, **REFERENCE})
    inputs = window(inputs, args.start, args.duration)
    if args.save_inputs:
        write_csv(args.save_inputs, inputs)
    vcu = run_vcu(inputs, logged_dc=args.logged_dc, efficiency=args.efficiency)
    if args.save_output:
        write_csv(args.save_output, vcu)
    if args.plot:
        plot(inputs, vcu, args.plot)
    print(compare(inputs, vcu).text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
