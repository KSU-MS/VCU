#!/usr/bin/env python3
"""Flash the VCU over CAN.

Talks UDS over ISO-TP to the VCU on the DAQ bus (request 0x7E0, response
0x7E8). Works with anything python-can supports:

    # SIL VCU on a virtual bus (see tools/sil)
    uv run tools/ota/flash.py vcu-a.ota vcu-b.ota --channel vcan_daq

    # The car, through a CANable/PCAN/etc on the DAQ connector
    uv run tools/ota/flash.py vcu-a.ota vcu-b.ota --interface socketcan --channel can0
    uv run tools/ota/flash.py vcu-a.ota vcu-b.ota --interface pcan --channel PCAN_USBBUS1

The VCU has two firmware slots and updates the one it isn't running from,
so an update has to be built for that slot: give it both images
(`pio run -e teensy41 -e teensy41_b`) and it sends the right one.

After the reboot the new firmware runs as a trial; once it answers with the
new version this confirms it. If it never answers (or with --no-confirm),
the next reset goes back to the old firmware.

The VCU refuses unless the car is parked (tractive system off).
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
DID_BOOT_STATUS = 0xF102
DID_DOWNLOAD_ADDRESS = 0xF103
DID_RESET_CAUSE = 0xF104
RESET_CAUSE_WATCHDOG = 1
ROUTINE_CHECK_IMAGE = 0xFF01
ROUTINE_CONFIRM = 0xFF02
NO_SLOT = 0xFF
# How long the bootloader (two signature checks) and the firmware's startup
# take, generously
REBOOT_TIMEOUT = 10.0
PROGRAMMING_SESSION = 0x02

# lib/vcu_boot/ota_image.hpp ImageError, in order
IMAGE_ERRORS = [
    "ok", "too short", "bad magic", "unsupported header version",
    "header crc mismatch", "image is for a different board",
    "size doesn't match header", "image too big", "image crc mismatch",
    "image is linked for a different address",
    "bad signature (not signed with the key this VCU trusts)",
]


class FlashError(Exception):
    pass


def connect(bus: can.BusABC, timeout_scale: float = 1.0) -> Client:
    """timeout_scale stretches every timeout, for targets slower than real
    time (the Renode-emulated VCU runs several times slower)."""
    stack = isotp.CanStack(
        bus,
        address=isotp.Address(isotp.AddressingMode.Normal_11bits,
                              txid=REQUEST_ID, rxid=RESPONSE_ID),
        params={
            "tx_padding": 0xCC,
            "tx_data_min_length": 8,
            "blocking_send": True,
            "rx_flowcontrol_timeout": int(1000 * timeout_scale),
            "rx_consecutive_frame_timeout": int(1000 * timeout_scale),
        },
    )
    config = dict(udsoncan.configs.default_client_config)
    config.update({
        "data_identifiers": {
            DID_SOFTWARE_VERSION: ">I",
            DID_BOOT_STATUS: "BBBB",
            DID_DOWNLOAD_ADDRESS: ">I",
            DID_RESET_CAUSE: "B",
        },
        # Flash erases make some TransferData responses slow, don't trust the
        # 50 ms P2 the VCU advertises
        "use_server_timing": False,
        "p2_timeout": 2.0 * timeout_scale,
        "request_timeout": 5.0 * timeout_scale,
        "server_address_format": 32,
        "server_memorysize_format": 32,
    })
    return Client(PythonIsoTpConnection(stack), config=config)


def read_version(client: Client) -> int:
    # udsoncan decodes a struct format DID as a tuple
    (version,) = client.read_data_by_identifier(DID_SOFTWARE_VERSION).service_data.values[
        DID_SOFTWARE_VERSION]
    return version


def progress(done: int, total: int, started: float) -> None:
    width = 40
    frac = done / total
    rate = done / max(time.monotonic() - started, 1e-6) / 1024
    bar = "#" * int(frac * width)
    print(f"\r  [{bar:<{width}}] {frac:6.1%} {rate:6.1f} KiB/s", end="", flush=True)


def boot_status(client: Client) -> tuple[int, int, int, int]:
    """(running slot, active slot, trial slot or 0xFF, trial boots)"""
    return client.read_data_by_identifier(DID_BOOT_STATUS).service_data.values[DID_BOOT_STATUS]


def download_address(client: Client) -> int:
    (addr,) = client.read_data_by_identifier(DID_DOWNLOAD_ADDRESS).service_data.values[
        DID_DOWNLOAD_ADDRESS]
    return addr


def reset_by_watchdog(client: Client) -> bool:
    """Did the watchdog reset the VCU last (its firmware hung)?"""
    (cause,) = client.read_data_by_identifier(DID_RESET_CAUSE).service_data.values[
        DID_RESET_CAUSE]
    return cause == RESET_CAUSE_WATCHDOG


def pick_image(images: list[bytes], load_address: int) -> bytes:
    """The one built for the slot the VCU will download into"""
    for image in images:
        if ota_image.verify(image).load_address == load_address:
            return image
    have = ", ".join(f"0x{ota_image.verify(i).load_address:08X}" for i in images)
    raise FlashError(f"the VCU needs an image linked for 0x{load_address:08X} (the slot "
                     f"it isn't running from), got {have}: pass both vcu-a.ota and vcu-b.ota")


def confirm(client: Client) -> None:
    status = client.start_routine(ROUTINE_CONFIRM).service_data.routine_status_record
    if not status or status[0] != 0:
        raise FlashError("VCU couldn't record the confirmation")


def flash(client: Client, images: bytes | list[bytes], *, reset: bool = True,
          confirm_update: bool = True, reboot_timeout: float = REBOOT_TIMEOUT,
          quiet: bool = False) -> None:
    say = (lambda *a, **k: None) if quiet else print
    if isinstance(images, bytes):
        images = [images]

    before = read_version(client)
    image = pick_image(images, download_address(client))
    header = ota_image.verify(image)
    say(f"VCU running {before:08x}, flashing {header.fw_version:08x} "
        f"({header.image_size} bytes, for 0x{header.load_address:08X})")

    try:
        client.change_session(PROGRAMMING_SESSION)
    except NegativeResponseException as e:
        if e.response.code == 0x22:
            raise FlashError("VCU refused programming: car isn't parked "
                             "(tractive system must be off)") from e
        raise

    try:
        resp = client.request_download(MemoryLocation(
            address=0, memorysize=len(image), address_format=32, memorysize_format=32))
    except NegativeResponseException as e:
        if e.response.code == 0x22:
            raise FlashError("VCU is running an update that isn't confirmed yet; "
                             "confirm it (--confirm-only) or reset it first") from e
        raise
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
        say("  not resetting (--no-reset), image stays downloaded")
        return

    client.ecu_reset(1)
    say("  rebooting into it (trial)...")

    deadline = time.monotonic() + reboot_timeout
    while time.monotonic() < deadline:
        time.sleep(0.5)
        try:
            now = read_version(client)
        except (TimeoutException, NegativeResponseException):
            continue
        if now == header.fw_version:
            break
    else:
        raise FlashError("VCU didn't come back with the new version; it goes back "
                         "to the old one on its next reset")

    if not confirm_update:
        say(f"VCU running {now:08x} as a trial, not confirmed (--no-confirm): "
            f"the next reset goes back to {before:08x}")
        return
    confirm(client)
    say(f"VCU now running {now:08x}, confirmed")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("images", type=Path, nargs="*",
                   help="OTA images from tools/ota/image.py, ideally one per slot")
    p.add_argument("--interface", default="socketcan", help="python-can interface")
    p.add_argument("--channel", default="vcan_daq")
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument("--no-reset", action="store_true",
                   help="download and verify, but don't reboot into it")
    p.add_argument("--no-confirm", action="store_true",
                   help="leave the update as a trial: the next reset rolls back")
    p.add_argument("--confirm-only", action="store_true",
                   help="just confirm the firmware that's running now")
    p.add_argument("--timeout-scale", type=float, default=1.0,
                   help="stretch all timeouts, e.g. 10 for the Renode-emulated VCU")
    args = p.parse_args(argv)

    images = []
    for path in args.images:
        try:
            ota_image.verify(path.read_bytes())
        except ValueError as e:
            print(f"{path}: not a valid image: {e}", file=sys.stderr)
            return 1
        images.append(path.read_bytes())
    if not images and not args.confirm_only:
        p.error("no images given")

    with can.Bus(interface=args.interface, channel=args.channel,
                 bitrate=args.bitrate) as bus, connect(bus, args.timeout_scale) as client:
        try:
            if args.confirm_only:
                confirm(client)
                print(f"VCU running {read_version(client):08x}, confirmed")
                return 0
            flash(client, images, reset=not args.no_reset,
                  confirm_update=not args.no_confirm,
                  reboot_timeout=REBOOT_TIMEOUT * args.timeout_scale)
        except (FlashError, NegativeResponseException, TimeoutException) as e:
            print(f"\nflash failed: {e}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
