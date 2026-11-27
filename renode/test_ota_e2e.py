"""End to end: an OTA update of the real ARM firmware in Renode.

Build A (this checkout) runs on the emulated Teensy; build B (the same source
with an empty commit on top, so it reports a different version) goes over
vcan_daq with tools/ota/flash.py, gets staged and verified, FlasherX's
flash_move copies it over A and resets, and the board has to come back as B.

The flash behaves like the W25Q64JV (TeensyFlexSPI.cs); any command the real
chip would have ignored fails the test.

    tools/run_e2e.sh renode [--flash-timing=max]

Takes a few minutes: the emulator runs ~6x slower than real time.
"""
import shutil
import subprocess
import time

import can
import pytest
from udsoncan.exceptions import TimeoutException

import flash as ota_flash
import image as ota_image
from conftest import ROOT, needs_vcan

FIRMWARE_A = ROOT / ".pio/build/teensy41/firmware.elf"
# The emulated VCU runs several times slower than real time
TIMEOUT_SCALE = 10

pytestmark = [
    needs_vcan,
    pytest.mark.skipif(shutil.which("renode") is None, reason="needs renode (nix develop)"),
    pytest.mark.skipif(not FIRMWARE_A.exists(), reason="build it with: pio run -e teensy41"),
]


def run(*cmd, cwd=ROOT):
    subprocess.run(cmd, cwd=cwd, check=True)


@pytest.fixture(scope="module")
def image_b(tmp_path_factory) -> bytes:
    tree = tmp_path_factory.mktemp("b") / "src"
    run("git", "worktree", "add", "-q", "--detach", str(tree), "HEAD")
    try:
        # The version is the commit hash; this moves no branch
        run("git", "-c", "user.name=ota-e2e", "-c", "user.email=ota-e2e@localhost",
            "commit", "-q", "--allow-empty", "-m", "ota e2e: build B", cwd=tree)
        # Reuse A's libdeps: this may run in a network namespace with no network
        shutil.copytree(ROOT / ".pio/libdeps", tree / ".pio/libdeps")
        run("pio", "run", "-s", "-e", "teensy41", cwd=tree)
        version, _ = ota_image.git_version(tree)
        firmware = ota_image.read_intel_hex(tree / ".pio/build/teensy41/firmware.hex")
        return ota_image.build(firmware, version)
    finally:
        run("git", "worktree", "remove", "--force", str(tree))


@pytest.fixture
def daq():
    with can.Bus(interface="socketcan", channel="vcan_daq") as bus:
        yield bus


@pytest.fixture
def vcu(request, tmp_path, daq):
    """Renode running build A, bridged to the vcan buses. Yields a connected
    UDS client once the firmware answers."""
    worst_case = request.config.getoption("--flash-timing") == "max"
    log = tmp_path / "renode.log"
    script = (f"$elf=@{FIRMWARE_A}; include @{ROOT}/renode/vcan.resc; "
              f"sysbus.flexspi1 WorstCaseTiming {str(worst_case).lower()}; "
              'logLevel 3; emulation RunFor "00:15:00"; quit')
    with open(log, "w") as out:
        renode = subprocess.Popen(
            ["renode", "--console", "--disable-gui", "--plain", "-e", script],
            stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    try:
        with ota_flash.connect(daq, timeout_scale=TIMEOUT_SCALE) as client:
            wait_for_boot(client, renode, log)
            yield client
    finally:
        renode.kill()
        renode.wait()
        print(f"\n  renode log: {log}")

    ignored = [line for line in log.read_text().splitlines() if "W25Q64 would ignore" in line]
    assert not ignored, "flash commands the real W25Q64 would have ignored:\n" + "\n".join(ignored[:20])


def wait_for_boot(client, renode, log, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if renode.poll() is not None:
            pytest.fail(f"renode exited:\n{log.read_text()[-4000:]}")
        try:
            ota_flash.read_version(client)
            return
        except TimeoutException:
            pass
    pytest.fail(f"VCU never answered on vcan_daq:\n{log.read_text()[-4000:]}")


def test_update_a_to_b(vcu, image_b):
    version_a, _ = ota_image.git_version(ROOT)
    version_b = ota_image.verify(image_b).fw_version
    assert ota_flash.read_version(vcu) == version_a

    ota_flash.flash(vcu, image_b,
                    reboot_timeout=ota_flash.apply_time(len(image_b)) * TIMEOUT_SCALE)

    assert ota_flash.read_version(vcu) == version_b
