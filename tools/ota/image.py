#!/usr/bin/env python3
"""Build and inspect VCU OTA images.

An image is a 64 byte header followed by the raw Teensy flash contents, i.e.
exactly what teensy_loader would write starting at 0x60000000. The header
layout must match lib/ota/ota_image.hpp.

    python tools/ota/image.py build .pio/build/teensy41/firmware.hex -o vcu.ota
    python tools/ota/image.py info vcu.ota
"""
from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path

MAGIC = b"KVCU"
HEADER_VERSION = 1
HEADER_SIZE = 64
TARGET = "teensy41-vcu"
FLASH_BASE = 0x60000000

# magic, header_version, header_size, image_size, image_crc32, fw_version,
# target[16], flags, reserved[20]   (header_crc32 follows)
_BODY = struct.Struct("<4sHHIII16sI20s")
assert _BODY.size + 4 == HEADER_SIZE

FLAG_DIRTY = 1 << 0


@dataclass
class Header:
    image_size: int
    image_crc32: int
    fw_version: int
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
            bytes(20),
        )
        return body + struct.pack("<I", zlib.crc32(body))

    @classmethod
    def unpack(cls, data: bytes) -> "Header":
        if len(data) < HEADER_SIZE:
            raise ValueError("too short for a header")
        body, (crc,) = data[: _BODY.size], struct.unpack_from("<I", data, _BODY.size)
        magic, hver, hsize, size, icrc, ver, target, flags, _ = _BODY.unpack(body)
        if magic != MAGIC:
            raise ValueError(f"bad magic {magic!r}")
        if (hver, hsize) != (HEADER_VERSION, HEADER_SIZE):
            raise ValueError(f"unsupported header version {hver}/{hsize}")
        if zlib.crc32(body) != crc:
            raise ValueError("header crc mismatch")
        return cls(size, icrc, ver, target.rstrip(b"\0").decode(), flags)


def build(firmware: bytes, fw_version: int, *, target: str = TARGET, flags: int = 0) -> bytes:
    header = Header(len(firmware), zlib.crc32(firmware), fw_version, target, flags)
    return header.pack() + firmware


def verify(image: bytes) -> Header:
    """Same checks the VCU does before it will apply an image"""
    header = Header.unpack(image)
    payload = image[HEADER_SIZE:]
    if len(payload) != header.image_size:
        raise ValueError(f"size mismatch: header says {header.image_size}, got {len(payload)}")
    if zlib.crc32(payload) != header.image_crc32:
        raise ValueError("image crc mismatch")
    return header


def read_intel_hex(path: Path, base: int = FLASH_BASE) -> bytes:
    """Flatten an Intel HEX file into the bytes from `base` upwards, gaps 0xFF"""
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
    if lo < base:
        raise ValueError(f"{path}: data at 0x{lo:08X} is below flash base 0x{base:08X}")
    out = bytearray(b"\xFF" * (hi - base + 1))
    for a, b in memory.items():
        out[a - base] = b
    return bytes(out)


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


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="package a firmware.hex/.bin as an OTA image")
    b.add_argument("firmware", type=Path)
    b.add_argument("-o", "--output", type=Path, required=True)
    b.add_argument("--version", type=lambda s: int(s, 16),
                   help="hex version, default: git short hash like AUTO_VERSION")
    b.add_argument("--target", default=TARGET)

    i = sub.add_parser("info", help="print and verify an image")
    i.add_argument("image", type=Path)

    args = p.parse_args(argv)

    if args.cmd == "build":
        if args.firmware.suffix.lower() == ".hex":
            firmware = read_intel_hex(args.firmware)
        else:
            firmware = args.firmware.read_bytes()

        if args.version is None:
            version, dirty = git_version(Path(__file__).resolve().parent)
        else:
            version, dirty = args.version, False

        image = build(firmware, version, target=args.target,
                      flags=FLAG_DIRTY if dirty else 0)
        args.output.write_bytes(image)
        print(f"{args.output}: {len(firmware)} bytes, version {version:08x}"
              f"{' (dirty)' if dirty else ''}, target {args.target}")
        return 0

    image = args.image.read_bytes()
    try:
        h = verify(image)
    except ValueError as e:
        print(f"{args.image}: INVALID: {e}", file=sys.stderr)
        return 1
    print(f"{args.image}: ok\n  target   {h.target}\n  version  {h.fw_version:08x}"
          f"{' (dirty)' if h.flags & FLAG_DIRTY else ''}\n"
          f"  size     {h.image_size} bytes\n  crc32    {h.image_crc32:08x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
