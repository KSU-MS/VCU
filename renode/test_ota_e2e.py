"""End to end: A/B updates of the real ARM firmware in Renode.

The emulated Teensy gets what a USB flash would put on it (`pio run -e
teensy41`'s firmware.hex): the bootloader, a fresh boot state and slot A,
build A of this checkout. Build B is the same source with an empty commit
on top (so it reports a different version), built for slot B; uncommitted
changes are part of both.

The test, in one emulator session:

  1. A -> B over vcan_daq with tools/ota/flash.py, confirmed: the bootloader
     verified B's signature, booted it as a trial, and it stays.
  2. B -> A again, not confirmed, then a reset: the bootloader rolls back
     to B.

The flash behaves like the W25Q64JV (TeensyFlexSPI.cs); any command the real
chip would have ignored fails the test.

    tools/run_e2e.sh renode [--flash-timing=max]

About 5 minutes: CAN from the host is slow in Renode (renode/README.md).
"""
import os
import shutil
import subprocess
import sys
import time

import can
import pytest
from elftools.elf.elffile import ELFFile
from udsoncan.exceptions import TimeoutException

import flash as ota_flash
import image as ota_image
from conftest import ROOT, needs_vcan

BUILD_A = ROOT / ".pio/build/teensy41"
# Generous: the emulated VCU's speed relative to the host varies
TIMEOUT_SCALE = 10

pytestmark = [
    # Everything on the host side is a request and a timeout
    pytest.mark.faster_than_realtime,
    needs_vcan,
    pytest.mark.skipif(shutil.which("renode") is None, reason="needs renode (nix develop)"),
    pytest.mark.skipif(not (BUILD_A / "vcu-a.ota").exists(),
                       reason="build it with: pio run -e teensy41"),
]


def run(*cmd, cwd=ROOT, **kw):
    subprocess.run(cmd, cwd=cwd, check=True, **kw)


def symbol(elf, name) -> int:
    with open(elf, "rb") as f:
        for s in ELFFile(f).get_section_by_name(".symtab").iter_symbols():
            if s.name == name:
                return s["st_value"] & ~1  # no Thumb bit: hooks match the PC
    raise KeyError(name)


@pytest.fixture(scope="module")
def build_b(tmp_path_factory):
    """(vcu-b.ota bytes, firmware.elf path) for build B"""
    out = tmp_path_factory.mktemp("b")
    tree = out / "src"
    run("git", "worktree", "add", "-q", "--detach", str(tree), "HEAD")
    try:
        # This checkout as it is, committed in the worktree: same code as A,
        # but the version is the commit hash, so B reports a different one.
        # Moves no branch.
        def ls(*flags):
            out = subprocess.run(["git", "ls-files", "-z", *flags], cwd=ROOT,
                                 check=True, capture_output=True).stdout
            return [f for f in out.decode().split("\0") if f]
        for f in ls("-d"):
            (tree / f).unlink(missing_ok=True)
        for f in ls("-co", "--exclude-standard"):
            if (ROOT / f).is_file():
                (tree / f).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(ROOT / f, tree / f)
        run("git", "add", "-A", cwd=tree)
        run("git", "-c", "user.name=ota-e2e", "-c", "user.email=ota-e2e@localhost",
            "commit", "-q", "--allow-empty", "-m", "ota e2e: build B", cwd=tree)
        # Reuse A's libdeps: this may run in a network namespace with no
        # network. Same for Python: sign with the interpreter running us.
        shutil.copytree(ROOT / ".pio/libdeps/teensy41", tree / ".pio/libdeps/teensy41_b")
        env = {**os.environ, "VCU_PYTHON": sys.executable}
        run("pio", "run", "-s", "-e", "teensy41_b", cwd=tree, env=env)
        shutil.copy(tree / ".pio/build/teensy41_b/vcu-b.ota", out / "vcu-b.ota")
        shutil.copy(tree / ".pio/build/teensy41_b/firmware.elf", out / "b.elf")
        return (out / "vcu-b.ota").read_bytes(), out / "b.elf"
    finally:
        run("git", "worktree", "remove", "--force", str(tree))


@pytest.fixture
def daq():
    with can.Bus(interface="socketcan", channel="vcan_daq") as bus:
        yield bus


class Renode:
    def __init__(self, proc, log):
        self.proc, self.log = proc, log

    def fail(self, why):
        pytest.fail(f"{why}:\n{self.log.read_text()[-4000:]}")


@pytest.fixture
def renode(request, tmp_path, build_b, renode_pacing):
    """Renode running the USB bundle, bridged to the vcan buses"""
    worst_case = request.config.getoption("--flash-timing") == "max"
    _, b_elf = build_b
    log = tmp_path / "renode.log"
    script = (f"$bundle=@{BUILD_A / 'firmware.hex'}; $elf=@{BUILD_A / 'firmware.elf'}; "
              f"include @{ROOT}/renode/vcan.resc; {renode_pacing}; "
              # teensy41.resc skips slot A's configure_cache; B's is elsewhere
              f'cpu AddHook {symbol(b_elf, "configure_cache"):#x} "cpu.PC = cpu.LR"; '
              f"sysbus.flexspi1 WorstCaseTiming {str(worst_case).lower()}; "
              'logLevel 3; emulation RunFor "00:30:00"; quit')
    with open(log, "w") as out:
        proc = subprocess.Popen(
            ["renode", "--console", "--disable-gui", "--plain", "-e", script],
            stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    try:
        yield Renode(proc, log)
    finally:
        proc.kill()
        proc.wait()
        print(f"\n  renode log: {log}")

    ignored = [line for line in log.read_text().splitlines() if "W25Q64 would ignore" in line]
    assert not ignored, "flash commands the real W25Q64 would have ignored:\n" + "\n".join(ignored[:20])


def wait_for_version(client, renode, version, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if renode.proc.poll() is not None:
            renode.fail("renode exited")
        try:
            if ota_flash.read_version(client) == version:
                return
        except TimeoutException:
            pass
    renode.fail(f"VCU never answered as {version:08x}")


def test_update_confirm_and_rollback(renode, daq, build_b):
    image_a = (BUILD_A / "vcu-a.ota").read_bytes()
    image_b, _ = build_b
    version_a = ota_image.verify(image_a).fw_version
    version_b = ota_image.verify(image_b).fw_version
    images = [image_a, image_b]
    reboot_timeout = ota_flash.REBOOT_TIMEOUT * TIMEOUT_SCALE

    with ota_flash.connect(daq, timeout_scale=TIMEOUT_SCALE) as vcu:
        # The bootloader verified slot A and started it
        wait_for_version(vcu, renode, version_a, timeout=120)
        assert ota_flash.boot_status(vcu) == (0, 0, 0xFF, 0)

        # 1. A -> B, confirmed
        ota_flash.flash(vcu, images, reboot_timeout=reboot_timeout)
        assert ota_flash.read_version(vcu) == version_b
        assert ota_flash.boot_status(vcu) == (1, 1, 0xFF, 0)

        # 2. B -> A, not confirmed: running A as a trial...
        ota_flash.flash(vcu, images, confirm_update=False, reboot_timeout=reboot_timeout)
        assert ota_flash.read_version(vcu) == version_a
        assert ota_flash.boot_status(vcu) == (0, 1, 0, 1)

        # ...until a reset, when the bootloader goes back to B
        vcu.ecu_reset(1)
        wait_for_version(vcu, renode, version_b, timeout=reboot_timeout)
        assert ota_flash.boot_status(vcu) == (1, 1, 0xFF, 0)
