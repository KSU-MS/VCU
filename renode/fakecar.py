"""The rest of the car on the virtual CAN buses, for the firmware in Renode:
the ACU and inverter saying the tractive system is up, the dash's RTD
button, and a record of every torque command the VCU sends.

Frames are encoded with ksu-ms-dbc's DBC (tools/can/ksu_can.dbc, the release
the firmware's can_lib comes from).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import can
import cantools

DBC = cantools.database.load_file(Path(__file__).resolve().parents[1] / "tools/can/ksu_can.dbc")

PRECHARGE_OK_STATE = 2
READY_TO_DRIVE = 4  # VCU state enum (include/vcu.hpp)
TRACTIVE_SYSTEM_FAULT = 7


@dataclass
class TorqueCommand:
    at: float  # host monotonic time
    torque_nm: float
    enable: bool


class FakeCar:
    def __init__(self, acc_channel="vcan_acc", inv_channel="vcan_inv"):
        self.acc = can.Bus(interface="socketcan", channel=acc_channel)
        self.inv = can.Bus(interface="socketcan", channel=inv_channel)
        self.commands: list[TorqueCommand] = []
        self.status: dict = {}  # the VCU's latest vcu_status (1 Hz), decoded
        self.status_at = 0.0    # when it arrived (host monotonic)
        self.ts_healthy = False
        self.rtd_button = False
        # Clear to make that board go quiet (dead, or its CAN wire broken)
        self.acu_talking = True
        self.inverter_talking = True
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._listen, daemon=True),
                         threading.Thread(target=self._send_periodic, daemon=True)]
        for t in self._threads:
            t.start()

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self.acc.shutdown()
        self.inv.shutdown()

    def _send(self, bus, name, **signals):
        msg = DBC.get_message_by_name(name)
        data = msg.encode({s.name: signals.get(s.name, 0) for s in msg.signals})
        bus.send(can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False))

    def _send_periodic(self):
        # Like the real boards: status every 100 ms
        while not self._stop.wait(0.1):
            if self.ts_healthy and self.acu_talking:
                self._send(self.acc, "acu_shutdown_status",
                           acu_imd_relay_state=1, acu_bms_relay_state=1)
                self._send(self.acc, "precharge_status", precharge_state=PRECHARGE_OK_STATE)
                self._send(self.acc, "MSGID_0X6B1", Pack_Summed_Voltage=450, Pack_Current=0)
            if self.ts_healthy and self.inverter_talking:
                self._send(self.inv, "M167_Voltage_Info", INV_DC_Bus_Voltage=450)
            self._send(self.inv, "dash_buttons", dash_button5status=int(self.rtd_button))

    def _listen(self):
        command = DBC.get_message_by_name("M192_Command_Message")
        status_id = DBC.get_message_by_name("vcu_status").frame_id
        while not self._stop.is_set():
            m = self.inv.recv(timeout=0.2)
            if m is None:
                continue
            if m.arbitration_id == command.frame_id:
                d = command.decode(m.data, decode_choices=False)
                with self._lock:
                    self.commands.append(TorqueCommand(
                        time.monotonic(), d["VCU_INV_Torque_Command"],
                        bool(d["VCU_INV_Inverter_Enable"])))
            elif m.arbitration_id == status_id:
                self.status = DBC.get_message_by_frame_id(status_id).decode(
                    m.data, decode_choices=False)
                self.status_at = time.monotonic()

    def commands_since(self, index: int) -> list[TorqueCommand]:
        with self._lock:
            return self.commands[index:]

    def wait_state(self, state: int, timeout: float) -> bool:
        """Wait for a status, sent after this was called, saying `state`"""
        since, deadline = time.monotonic(), time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.status_at > since and self.status.get("vcu_statemachine_state") == state:
                return True
            time.sleep(0.1)
        return False

    def command_count(self) -> int:
        with self._lock:
            return len(self.commands)
