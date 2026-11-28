"""Run the emulated VCU and talk to Renode's monitor while it runs.

The monitor is Renode's command line, served on a TCP port (-P) so a test can
change the emulated board's inputs mid-run: what the pedal sensors read, a
reset. Needs loopback up (tools/run_e2e.sh brings it up in its namespace).
"""
from __future__ import annotations

import re
import socket
import subprocess
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUILD_A = ROOT / ".pio/build/teensy41"

# Telnet option negotiation (IAC ...) and terminal colour codes
_NOISE = re.compile(rb"\xff[\xfb-\xfe].|\xff[\xf0-\xfa]|\x1b\[[0-9;]*[A-Za-z]")
_PROMPT = re.compile(rb"\((?:monitor|[\w-]+)\) $")
_PROMPT_TEXT = re.compile(r"\((?:monitor|[\w-]+)\) \s*$")


class Monitor:
    def __init__(self, port: int, timeout: float = 60):
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)
        self.sock.settimeout(timeout)
        self._read_to_prompt()

    def _read_to_prompt(self) -> str:
        buf = b""
        while not _PROMPT.search(_NOISE.sub(b"", buf)):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("Renode monitor closed")
            buf += chunk
        return _NOISE.sub(b"", buf).decode(errors="replace")

    def run(self, command: str) -> str:
        """Run a monitor command, return what it printed. The command is
        bracketed by echoed markers, so output left over from anything
        before it (the start-up script prints here too) is ignored."""
        self._n = getattr(self, "_n", 0) + 1
        begin, end = f"<<{self._n}<<", f">>{self._n}>>"
        self.sock.sendall(f'echo "{begin}"\n{command}\necho "{end}"\n'.encode())
        # The monitor echoes each command line too, so each marker shows up
        # twice: in its echo command, then printed. Wait for both ends.
        out = ""
        while out.count(end) < 2:
            out += self._read_to_prompt()
        body = out[:out.index(end)].rsplit(begin, 1)[1]
        return "\n".join(l for l in body.splitlines()
                         if l.strip() and command not in l and "echo" not in l
                         and not re.fullmatch(r"\s*\([\w-]+\)\s*", l))

    def close(self):
        self.sock.close()


class Renode:
    """Renode running the USB bundle on the emulated VCU, CAN on vcan"""

    def __init__(self, log: Path, pacing: str, monitor_port: int = 3456, extra: str = ""):
        """`pacing`: conftest.py's renode_pacing fixture"""
        self.log = log
        script = (f"$bundle=@{BUILD_A / 'firmware.hex'}; $elf=@{BUILD_A / 'firmware.elf'}; "
                  f"include @{ROOT}/renode/vcan.resc; {pacing}; {extra} "
                  'logLevel 3; start')
        with open(log, "w") as out:
            self.proc = subprocess.Popen(
                ["renode", "--disable-gui", "--plain",
                 "-P", str(monitor_port), "-e", script],
                stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        try:
            self.monitor = Monitor(monitor_port)
        except Exception:
            self.stop()
            raise

    def stop(self):
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait()

    def fail(self, why: str):
        pytest.fail(f"{why}:\n{self.log.read_text()[-4000:]}")
