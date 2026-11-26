#!/usr/bin/env python3
"""Flash the VCU over CAN.

Talks UDS over ISO-TP to the VCU on the DAQ bus (request 0x7E0, response
0x7E8). Works with anything python-can supports:

    # SIL VCU on a virtual bus (see tools/sil)
    python tools/ota/flash.py vcu.ota --interface socketcan --channel vcan_daq

    # The car, through a CANable/PCAN/etc on the DAQ connector
    python tools/ota/flash.py vcu.ota --interface socketcan --channel can0
    python tools/ota/flash.py vcu.ota --interface pcan --channel PCAN_USBBUS1 --bitrate 1000000

The image must come from tools/ota/image.py. The VCU refuses unless the car
is parked (tractive system off).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import can
import isotp
import udsoncan
from udsoncan.client import Client
from udsoncan.common.MemoryLocation import MemoryLocation
from udsoncan.connections import PythonIsoTpConnection
from udsoncan.exceptions import NegativeResponseException, TimeoutException

sys.path.insert(0, str(Path(__file__).resolve().parent))
import image as ota_image  # noqa: E402

REQUEST_ID = 0x7E0
RESPONSE_ID = 0x7E8

DID_SOFTWARE_VERSION = 0xF189
ROUTINE_CHECK_IMAGE = 0xFF01
PROGRAMMING_SESSION = 0x02

# lib/ota/ota_image.hpp ImageError, in order
IMAGE_ERRORS = [
    "ok", "too short", "bad magic", "unsupported header version",
    "header crc mismatch", "image is for a different board",
    "size doesn't match header", "image too big", "image crc mismatch",
]


class FlashError(Exception):
    pass


def connect(bus: can.BusABC) -> Client:
    stack = isotp.CanStack(
        bus,
        address=isotp.Address(isotp.AddressingMode.Normal_11bits,
                              txid=REQUEST_ID, rxid=RESPONSE_ID),
        params={
            "tx_padding": 0xCC,
            "tx_data_min_length": 8,
            "blocking_send": True,
        },
    )
    config = dict(udsoncan.configs.default_client_config)
    config.update({
        "data_identifiers": {DID_SOFTWARE_VERSION: ">I"},
        # Flash erases make some TransferData responses slow, don't trust the
        # 50 ms P2 the VCU advertises
        "use_server_timing": False,
        "p2_timeout": 2.0,
        "request_timeout": 5.0,
        "server_address_format": 32,
        "server_memorysize_format": 32,
    })
    return Client(PythonIsoTpConnection(stack), config=config)


def read_version(client: Client) -> int:
    return client.read_data_by_identifier(DID_SOFTWARE_VERSION).service_data.values[
        DID_SOFTWARE_VERSION]


def progress(done: int, total: int, started: float) -> None:
    width = 40
    frac = done / total
    rate = done / max(time.monotonic() - started, 1e-6) / 1024
    bar = "#" * int(frac * width)
    print(f"\r  [{bar:<{width}}] {frac:6.1%} {rate:6.1f} KiB/s", end="", flush=True)


def flash(client: Client, image: bytes, *, reset: bool = True,
          reboot_timeout: float = 10.0, quiet: bool = False) -> None:
    say = (lambda *a, **k: None) if quiet else print
    header = ota_image.verify(image)

    before = read_version(client)
    say(f"VCU running {before:08x}, flashing {header.fw_version:08x} "
        f"({header.image_size} bytes)")

    try:
        client.change_session(PROGRAMMING_SESSION)
    except NegativeResponseException as e:
        if e.response.code == 0x22:
            raise FlashError("VCU refused programming: car isn't parked "
                             "(tractive system must be off)") from e
        raise

    resp = client.request_download(MemoryLocation(
        address=0, memorysize=len(image), address_format=32, memorysize_format=32))
    block = resp.service_data.max_length - 2  # minus SID and counter

    started = time.monotonic()
    counter = 1
    for off in range(0, len(image), block):
        client.transfer_data(counter, image[off:off + block])
        counter = (counter + 1) & 0xFF
        if not quiet:
            progress(min(off + block, len(image)), len(image), started)
    say()

    client.request_transfer_exit()

    status = client.start_routine(ROUTINE_CHECK_IMAGE).service_data.routine_status_record
    err = status[0] if status else 0xFF
    if err != 0:
        name = IMAGE_ERRORS[err] if err < len(IMAGE_ERRORS) else f"error {err}"
        raise FlashError(f"VCU rejected the image: {name}")
    say("  image verified on the VCU")

    if not reset:
        say("  not resetting (--no-reset), image stays staged")
        return

    client.ecu_reset(1)
    say("  applying and rebooting...")

    deadline = time.monotonic() + reboot_timeout
    while time.monotonic() < deadline:
        time.sleep(0.5)
        try:
            now = read_version(client)
        except (TimeoutException, NegativeResponseException):
            continue
        if now == header.fw_version:
            say(f"VCU now running {now:08x}")
            return
    raise FlashError("VCU didn't come back with the new version")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("image", type=Path, help="OTA image from tools/ota/image.py")
    p.add_argument("--interface", default="socketcan", help="python-can interface")
    p.add_argument("--channel", default="vcan_daq")
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument("--no-reset", action="store_true",
                   help="stage and verify, but don't apply")
    args = p.parse_args(argv)

    image = args.image.read_bytes()
    try:
        ota_image.verify(image)
    except ValueError as e:
        print(f"{args.image}: not a valid image: {e}", file=sys.stderr)
        return 1

    with can.Bus(interface=args.interface, channel=args.channel,
                 bitrate=args.bitrate) as bus, connect(bus) as client:
        try:
            flash(client, image, reset=not args.no_reset)
        except (FlashError, NegativeResponseException, TimeoutException) as e:
            print(f"\nflash failed: {e}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
