"""The Raspberry Pi on the DAQ bus, as far as the VCU sees it: a torque
ceiling (pi_torque_command) with the rolling counter and CRC the VCU checks
(include/pi_supervisor.hpp), and the VCU's vcu_pi_status read back.

It answers each vcu_pi_status (100 Hz of the VCU's time) with a command,
like a controller working off the VCU's data, rather than sending by the
host's clock: every frame from the host costs Renode several ms of wall
time (renode/README.md), so 100 Hz of host time would slow the emulation
to a crawl and make the Pi look slow to the VCU.
"""
from __future__ import annotations

import threading
import time

import can

from fakecar import DBC

COMMAND = DBC.get_message_by_name("pi_torque_command")
STATUS = DBC.get_message_by_name("vcu_pi_status")
LOCAL, PI = 0, 1  # vcu_pi_mode


def crc8(frame_id: int, data: bytes) -> int:
    """CRC-8 SAE J1850 over the ID (2 bytes, little endian) and the data"""
    crc = 0xFF
    for byte in bytes([frame_id & 0xFF, frame_id >> 8 & 0xFF]) + data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1 ^ 0x1D if crc & 0x80 else crc << 1) & 0xFF
    return crc ^ 0xFF


def command(ceiling_nm: float, engage: bool, counter: int) -> bytes:
    data = bytearray(COMMAND.encode({"pi_torque_ceiling": ceiling_nm, "pi_engage": int(engage),
                                     "pi_counter": counter & 0x0F, "pi_checksum": 0}))
    data[7] = crc8(COMMAND.frame_id, bytes(data[:7]))
    return bytes(data)


class FakePi:
    def __init__(self, channel="vcan_daq"):
        self.bus = can.Bus(interface="socketcan", channel=channel)
        self.ceiling_nm = 240.0
        self.engage = True
        self.talking = False
        self.corrupt_next = False  # one frame with a wrong checksum
        self.status: dict = {}
        self.status_at = 0.0
        self._counter = 0
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._listen, daemon=True)]
        for t in self._threads:
            t.start()

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self.bus.shutdown()

    def _send(self):
        data = bytearray(command(self.ceiling_nm, self.engage, self._counter))
        self._counter += 1
        if self.corrupt_next:
            self.corrupt_next = False
            data[7] ^= 0xFF
        self.bus.send(can.Message(arbitration_id=COMMAND.frame_id, data=bytes(data),
                                  is_extended_id=False))

    def _listen(self):
        while not self._stop.is_set():
            m = self.bus.recv(timeout=0.2)
            if m is not None and m.arbitration_id == STATUS.frame_id:
                self.status = STATUS.decode(m.data, decode_choices=False)
                self.status_at = time.monotonic()
                if self.talking:
                    self._send()

    def wait_mode(self, mode: int, timeout: float) -> bool:
        """Wait for a status, sent after this was called, saying `mode`"""
        since, deadline = time.monotonic(), time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.status_at > since and self.status.get("vcu_pi_mode") == mode:
                return True
            time.sleep(0.02)
        return False
