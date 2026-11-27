"""uv run pytest tools/ota"""
import hashlib
import sys
import zlib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import image  # noqa: E402

FW = bytes(range(256)) * 4
LOAD = 0x60021000
DEV_KEY = image.load_private_key(image.DEV_KEY)
DEV_PUB = bytes.fromhex((image.REPO / "keys" / "ota-dev.pub").read_text().strip())

# Same image as the golden test in test/test_ota, so the Python packer and
# the C++ checks can't drift apart without both suites noticing. Ed25519 is
# deterministic, so the signature is fixed too.
GOLDEN_HEADER = bytes.fromhex(
    "4b5643550200400000040000264c0bb7efbeadde7465656e737934312d766375"
    "00000000000000000010026000000000000000000000000000000000eedf7ff8"
)
GOLDEN_SIGNATURE = bytes.fromhex(
    "ea171265b8c28ac9c1b7226d979f0c0053edaa12aa68ed58b8db1d4b521f6cd1"
    "ba2b3ef99e28c47c97e4598a294df87776f7d3692a51e46fd6f41cbffda5e708"
)


def golden():
    return image.build(FW, 0xDEADBEEF, load_address=LOAD, key=DEV_KEY)


def test_dev_key_pair_matches():
    assert image.public_key_bytes(DEV_KEY) == DEV_PUB


def test_header_matches_firmware_layout():
    assert golden()[:64] == GOLDEN_HEADER


def test_signature_matches_firmware_check():
    assert golden()[64:128] == GOLDEN_SIGNATURE


def test_layout():
    img = golden()
    assert len(img) == image.FIRMWARE_OFFSET + len(FW)
    assert img[128:image.FIRMWARE_OFFSET] == b"\xff" * (image.FIRMWARE_OFFSET - 128)
    assert img[image.FIRMWARE_OFFSET:] == FW


def test_round_trip():
    img = image.build(FW, 0x1234, load_address=LOAD, key=DEV_KEY, flags=image.FLAG_DIRTY)
    h = image.verify(img, DEV_PUB)
    assert (h.image_size, h.fw_version, h.load_address, h.target, h.flags) == (
        len(FW), 0x1234, LOAD, image.TARGET, 1)
    assert h.image_crc32 == zlib.crc32(FW)


def test_signature_covers_header_and_firmware():
    img = golden()
    digest = hashlib.sha512(img[:64] + FW).digest()
    DEV_KEY.public_key().verify(img[64:128], digest)  # raises if not


def flip(offset):
    return lambda b: b[:offset] + bytes([b[offset] ^ 1]) + b[offset + 1:]


@pytest.mark.parametrize("corrupt, message", [
    (lambda b: b"XXXX" + b[4:], "magic"),
    (flip(20), "header crc"),
    (lambda b: b[:-1], "size mismatch"),
    (flip(image.FIRMWARE_OFFSET + 100), "image crc"),
    (flip(70), "bad signature"),
])
def test_verify_rejects(corrupt, message):
    with pytest.raises(ValueError, match=message):
        image.verify(corrupt(golden()), DEV_PUB)


def test_rejects_another_key():
    other = image.Ed25519PrivateKey.generate()
    img = image.build(FW, 1, load_address=LOAD, key=other)
    with pytest.raises(ValueError, match="bad signature"):
        image.verify(img, DEV_PUB)


def test_resigned_header_change_still_needs_the_key():
    # Changing the load address and fixing up the header CRC isn't enough
    img = bytearray(golden())
    h = image.Header.unpack(bytes(img))
    h.load_address = 0x60201000
    img[:64] = h.pack()
    with pytest.raises(ValueError, match="bad signature"):
        image.verify(bytes(img), DEV_PUB)


def test_intel_hex(tmp_path):
    # Two records at 0x60000000 with a gap, an extended linear address record
    hexfile = tmp_path / "fw.hex"
    hexfile.write_text(
        ":020000046000" + "9A\n"
        ":0400000001020304F2\n"
        ":02000600AABB93\n"
        ":00000001FF\n"
    )
    assert image.read_intel_hex(hexfile) == (0x60000000, b"\x01\x02\x03\x04\xff\xff\xaa\xbb")


def test_intel_hex_bad_checksum(tmp_path):
    hexfile = tmp_path / "fw.hex"
    hexfile.write_text(":0400000001020304F3\n:00000001FF\n")
    with pytest.raises(ValueError, match="checksum"):
        image.read_intel_hex(hexfile)
