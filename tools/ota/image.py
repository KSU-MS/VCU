#!/usr/bin/env python3
"""Build, sign and inspect VCU OTA images.

An image is what gets written to flash, byte for byte, with the firmware
landing at the address it was linked for (see lib/vcu_boot/ota_image.hpp,
which this must match):

    0x0000  64 byte header
    0x0040  Ed25519 signature over SHA-512(header || firmware)
    0x0080  0xFF padding
    0x1000  firmware, linked to run at the header's load address

    uv run tools/ota/image.py build .pio/build/teensy41/firmware.hex -o vcu.ota
    uv run tools/ota/image.py info vcu.ota
    uv run tools/ota/image.py keygen keys/car      # keys/car.key + keys/car.pub

Images are signed with keys/ota-dev.key unless --key or VCU_OTA_KEY says
otherwise. That key is public (it's in the repo): fine on the bench, and a
firmware built for the car should use its own key (see keys/README.md).
"""
from __future__ import annotations

import argparse
import hashlib
import os
import struct
import subprocess
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
    Ed25519PublicKey,
)
from cryptography.hazmat.primitives.serialization import Encoding, PublicFormat

MAGIC = b"KVCU"
HEADER_VERSION = 2
HEADER_SIZE = 64
SIGNATURE_OFFSET = HEADER_SIZE
SIGNATURE_SIZE = 64
FIRMWARE_OFFSET = 0x1000
TARGET = "teensy41-vcu"
FLASH_BASE = 0x60000000

REPO = Path(__file__).resolve().parents[2]
DEV_KEY = REPO / "keys" / "ota-dev.key"

# magic, header_version, header_size, image_size, image_crc32, fw_version,
# target[16], flags, load_address, reserved[16]   (header_crc32 follows)
_BODY = struct.Struct("<4sHHIII16sII16s")
assert _BODY.size + 4 == HEADER_SIZE

FLAG_DIRTY = 1 << 0


@dataclass
class Header:
    image_size: int
    image_crc32: int
    fw_version: int
    load_address: int
    target: str = TARGET
    flags: int = 0

    def pack(self) -> bytes:
        body = _BODY.pack(
            MAGIC,
            HEADER_VERSION,
            HEADER_SIZE,
            self.image_size,
            self.image_crc32,
            self.fw_version,
            self.target.encode().ljust(16, b"\0")[:16],
            self.flags,
            self.load_address,
            bytes(16),
        )
        return body + struct.pack("<I", zlib.crc32(body))

    @classmethod
    def unpack(cls, data: bytes) -> "Header":
        if len(data) < HEADER_SIZE:
            raise ValueError("too short for a header")
        body, (crc,) = data[: _BODY.size], struct.unpack_from("<I", data, _BODY.size)
        magic, hver, hsize, size, icrc, ver, target, flags, load, _ = _BODY.unpack(body)
        if magic != MAGIC:
            raise ValueError(f"bad magic {magic!r}")
        if (hver, hsize) != (HEADER_VERSION, HEADER_SIZE):
            raise ValueError(f"unsupported header version {hver}/{hsize}")
        if zlib.crc32(body) != crc:
            raise ValueError("header crc mismatch")
        return cls(size, icrc, ver, load, target.rstrip(b"\0").decode(), flags)


def load_private_key(path: Path) -> Ed25519PrivateKey:
    """A 32 byte Ed25519 seed, as hex"""
    return Ed25519PrivateKey.from_private_bytes(bytes.fromhex(path.read_text().strip()))


def public_key_bytes(key: Ed25519PrivateKey) -> bytes:
    return key.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)


def build(firmware: bytes, fw_version: int, *, load_address: int,
          key: Ed25519PrivateKey, target: str = TARGET, flags: int = 0) -> bytes:
    header = Header(len(firmware), zlib.crc32(firmware), fw_version, load_address,
                    target, flags).pack()
    signature = key.sign(hashlib.sha512(header + firmware).digest())
    pad = b"\xFF" * (FIRMWARE_OFFSET - HEADER_SIZE - SIGNATURE_SIZE)
    return header + signature + pad + firmware


def verify(image: bytes, public_key: bytes | None = None) -> Header:
    """Same checks the VCU and bootloader do (signature only if a key is given)"""
    header = Header.unpack(image)
    firmware = image[FIRMWARE_OFFSET:]
    if len(image) < FIRMWARE_OFFSET or len(firmware) != header.image_size:
        raise ValueError(f"size mismatch: header says {header.image_size}, got {len(firmware)}")
    if zlib.crc32(firmware) != header.image_crc32:
        raise ValueError("image crc mismatch")
    if public_key is not None:
        signature = image[SIGNATURE_OFFSET:SIGNATURE_OFFSET + SIGNATURE_SIZE]
        digest = hashlib.sha512(image[:HEADER_SIZE] + firmware).digest()
        try:
            Ed25519PublicKey.from_public_bytes(public_key).verify(signature, digest)
        except InvalidSignature:
            raise ValueError("bad signature") from None
    return header


def read_intel_hex(path: Path, base: int | None = None) -> tuple[int, bytes]:
    """Flatten an Intel HEX file into (start address, bytes), gaps 0xFF. The
    start is the lowest address in the file unless `base` is given."""
    memory: dict[int, int] = {}
    upper = 0
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"{path}:{lineno}: not an Intel HEX record")
        raw = bytes.fromhex(line[1:])
        if sum(raw) & 0xFF:
            raise ValueError(f"{path}:{lineno}: bad checksum")
        count, addr, rtype, data = raw[0], (raw[1] << 8) | raw[2], raw[3], raw[4:-1]
        if len(data) != count:
            raise ValueError(f"{path}:{lineno}: bad length")
        if rtype == 0x00:
            for i, b in enumerate(data):
                memory[upper + addr + i] = b
        elif rtype == 0x01:
            break
        elif rtype == 0x04:
            upper = int.from_bytes(data, "big") << 16
        elif rtype == 0x02:
            upper = int.from_bytes(data, "big") << 4
        # 0x03/0x05 are start addresses, not data

    if not memory:
        raise ValueError(f"{path}: no data")
    lo, hi = min(memory), max(memory)
    if base is None:
        base = lo
    if lo < base:
        raise ValueError(f"{path}: data at 0x{lo:08X} is below 0x{base:08X}")
    out = bytearray(b"\xFF" * (hi - base + 1))
    for a, b in memory.items():
        out[a - base] = b
    return base, bytes(out)


def git_version(repo: Path) -> tuple[int, bool]:
    """(AUTO_VERSION, dirty) the same way pio-git-hash-gen computes them"""
    short = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "-C", str(repo), "status", "--porcelain"],
        check=True, capture_output=True, text=True,
    ).stdout
    return int(short, 16), status != ""


def default_key_path() -> Path:
    return Path(os.environ.get("VCU_OTA_KEY", DEV_KEY))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="package and sign a firmware.hex/.bin as an OTA image")
    b.add_argument("firmware", type=Path)
    b.add_argument("-o", "--output", type=Path, required=True)
    b.add_argument("--version", type=lambda s: int(s, 16),
                   help="hex version, default: git short hash like AUTO_VERSION")
    b.add_argument("--target", default=TARGET)
    b.add_argument("--load-address", type=lambda s: int(s, 0),
                   help="where the firmware is linked to run; default: the .hex's "
                        "lowest address, required for a .bin")
    b.add_argument("--key", type=Path, default=None,
                   help="Ed25519 private key (hex seed), default $VCU_OTA_KEY or keys/ota-dev.key")

    i = sub.add_parser("info", help="print and verify an image")
    i.add_argument("image", type=Path)
    i.add_argument("--public-key", type=Path, default=REPO / "keys" / "ota-dev.pub",
                   help="check the signature against this key (hex)")

    k = sub.add_parser("keygen", help="make a signing key pair: PREFIX.key (secret) and PREFIX.pub")
    k.add_argument("prefix", type=Path)

    args = p.parse_args(argv)

    if args.cmd == "keygen":
        key = Ed25519PrivateKey.generate()
        secret = args.prefix.with_suffix(".key")
        secret.touch(mode=0o600, exist_ok=False)
        secret.write_text(key.private_bytes_raw().hex() + "\n")
        args.prefix.with_suffix(".pub").write_text(public_key_bytes(key).hex() + "\n")
        print(f"{secret} (keep secret) and {args.prefix.with_suffix('.pub')}")
        return 0

    if args.cmd == "build":
        if args.firmware.suffix.lower() == ".hex":
            start, firmware = read_intel_hex(args.firmware)
            load_address = args.load_address if args.load_address is not None else start
            if load_address != start:
                raise SystemExit(f"{args.firmware} starts at 0x{start:08X}, not 0x{load_address:08X}")
        else:
            if args.load_address is None:
                raise SystemExit("--load-address is required for a .bin")
            firmware, load_address = args.firmware.read_bytes(), args.load_address

        if args.version is None:
            version, dirty = git_version(REPO)
        else:
            version, dirty = args.version, False

        key_path = args.key or default_key_path()
        image = build(firmware, version, load_address=load_address,
                      key=load_private_key(key_path), target=args.target,
                      flags=FLAG_DIRTY if dirty else 0)
        args.output.write_bytes(image)
        print(f"{args.output}: {len(firmware)} bytes at 0x{load_address:08X}, version "
              f"{version:08x}{' (dirty)' if dirty else ''}, signed with {key_path}")
        return 0

    image = args.image.read_bytes()
    try:
        h = verify(image, bytes.fromhex(args.public_key.read_text().strip()))
    except ValueError as e:
        print(f"{args.image}: INVALID: {e}", file=sys.stderr)
        return 1
    print(f"{args.image}: ok, signed by {args.public_key}\n  target   {h.target}\n"
          f"  version  {h.fw_version:08x}{' (dirty)' if h.flags & FLAG_DIRTY else ''}\n"
          f"  load at  0x{h.load_address:08X}\n"
          f"  size     {h.image_size} bytes\n  crc32    {h.image_crc32:08x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
