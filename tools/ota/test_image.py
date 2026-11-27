"""uv run pytest tools/ota"""
import sys
import zlib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import image  # noqa: E402

FW = bytes(range(256)) * 4

# Same image as the golden test in test/test_ota, so the Python packer and
# the C++ parser can't drift apart without both suites noticing
GOLDEN_HEADER = bytes.fromhex(
    "4b5643550100400000040000264c0bb7efbeadde7465656e737934312d766375"
    "0000000000000000000000000000000000000000000000000000000010ecb3bb"
)


def test_header_matches_firmware_layout():
    assert image.build(FW, 0xDEADBEEF)[:64] == GOLDEN_HEADER


def test_round_trip():
    img = image.build(FW, 0x1234, flags=image.FLAG_DIRTY)
    h = image.verify(img)
    assert (h.image_size, h.fw_version, h.target, h.flags) == (len(FW), 0x1234, image.TARGET, 1)
    assert h.image_crc32 == zlib.crc32(FW)


@pytest.mark.parametrize("corrupt, message", [
    (lambda b: b"XXXX" + b[4:], "magic"),
    (lambda b: b[:20] + bytes([b[20] ^ 1]) + b[21:], "header crc"),
    (lambda b: b[:-1], "size mismatch"),
    (lambda b: b[:100] + bytes([b[100] ^ 1]) + b[101:], "image crc"),
])
def test_verify_rejects(corrupt, message):
    with pytest.raises(ValueError, match=message):
        image.verify(corrupt(image.build(FW, 1)))


def test_intel_hex(tmp_path):
    # Two records at 0x60000000 with a gap, an extended linear address record
    hexfile = tmp_path / "fw.hex"
    hexfile.write_text(
        ":020000046000" + "9A\n"
        ":0400000001020304F2\n"
        ":02000600AABB93\n"
        ":00000001FF\n"
    )
    assert image.read_intel_hex(hexfile) == b"\x01\x02\x03\x04\xff\xff\xaa\xbb"


def test_intel_hex_bad_checksum(tmp_path):
    hexfile = tmp_path / "fw.hex"
    hexfile.write_text(":0400000001020304F3\n:00000001FF\n")
    with pytest.raises(ValueError, match="checksum"):
        image.read_intel_hex(hexfile)
