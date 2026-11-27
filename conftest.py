"""Shared by the end-to-end tests in tools/sil and renode/.

They need the vcan_acc/inv/daq buses, which tools/run_e2e.sh sets up (without
root, in a network namespace); without them they're skipped.
"""
import socket
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools/ota"))

BUSES = ("vcan_acc", "vcan_inv", "vcan_daq")


def have_vcan() -> bool:
    # Not /sys/class/net, which shows the host's interfaces from inside the
    # network namespace run_e2e.sh uses
    try:
        return all(socket.if_nametoindex(b) for b in BUSES)
    except OSError:
        return False


needs_vcan = pytest.mark.skipif(not have_vcan(), reason="no vcan buses, use tools/run_e2e.sh")


def pytest_addoption(parser):
    parser.addoption("--flash-timing", choices=("typ", "max"), default="typ",
                     help="Renode tests: W25Q64 datasheet typical or max erase/program times")
