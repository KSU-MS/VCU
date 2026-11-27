"""End to end: the SIL VCU on vcan, flashed by tools/ota/flash.py.

Needs vcan_acc/vcan_inv/vcan_daq to exist and the SIL binary to be built.
tools/run_e2e.sh sets all that up (without root, in a network namespace);
these are skipped otherwise.
"""
import random
import subprocess
import time

import can
import pytest

import flash as ota_flash
import image as ota_image
from conftest import ROOT, needs_vcan

SIL = ROOT / ".pio/build/sil/program"

pytestmark = [
    needs_vcan,
    pytest.mark.skipif(not SIL.exists(), reason="build it with: pio run -e sil"),
]


class Sil:
    def __init__(self, *args: str):
        self.proc = subprocess.Popen(
            [str(SIL), *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.output = []
        self.wait_for("VCU SIL up")

    def wait_for(self, text: str, timeout: float = 5.0) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                break
            self.output.append(line)
            if text in line:
                return line
        raise AssertionError(f"SIL never printed {text!r}:\n{''.join(self.output)}")

    def stop(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)


@pytest.fixture
def daq():
    with can.Bus(interface="socketcan", channel="vcan_daq") as bus:
        yield bus


def firmware(size=120 * 1024, seed=1) -> bytes:
    return random.Random(seed).randbytes(size)


def test_flash_and_reboot_into_new_version(daq):
    sil = Sil("--version", "aaaa0001")
    try:
        sil.wait_for("TRACTIVE_SYSTEM_DISABLED")
        img = ota_image.build(firmware(), 0xBBBB0002)
        with ota_flash.connect(daq) as client:
            assert ota_flash.read_version(client) == 0xAAAA0001
            started = time.monotonic()
            ota_flash.flash(client, img, quiet=True)
            elapsed = time.monotonic() - started
            assert ota_flash.read_version(client) == 0xBBBB0002
        sil.wait_for("applied update, rebooting into bbbb0002")
        print(f"\n  {len(img)} bytes in {elapsed:.1f} s ({len(img) / elapsed / 1024:.0f} KiB/s)")
    finally:
        sil.stop()


def test_refuses_while_tractive_system_is_live(daq):
    sil = Sil("--healthy-ts")
    try:
        sil.wait_for("TRACTIVE_SYSTEM_ENERGIZED")
        with ota_flash.connect(daq) as client:
            with pytest.raises(ota_flash.FlashError, match="isn't parked"):
                ota_flash.flash(client, ota_image.build(firmware(4096), 2), quiet=True)
    finally:
        sil.stop()


def test_rejects_image_for_another_board(daq):
    sil = Sil("--version", "aaaa0001")
    try:
        img = ota_image.build(firmware(8192), 3, target="teensy41-dash")
        with ota_flash.connect(daq) as client:
            with pytest.raises(ota_flash.FlashError, match="different board"):
                ota_flash.flash(client, img, quiet=True)
            # And it's still running the old firmware
            assert ota_flash.read_version(client) == 0xAAAA0001
    finally:
        sil.stop()
