# PlatformIO extra script: build in the public key OTA images must be signed
# with (lib/vcu_boot/ota_key.cpp). The VCU and the bootloader reject images
# that don't verify against it.
#
# VCU_OTA_PUBLIC_KEY=path/to/key.pub picks the key, default keys/ota-dev.pub.
# The dev key's private half is in the repo, so anyone can sign for a
# firmware built with it: fine on the bench, never on the car.
import os
from pathlib import Path

Import("env")

project = Path(env.subst("$PROJECT_DIR"))
path = Path(os.environ.get("VCU_OTA_PUBLIC_KEY", project / "keys" / "ota-dev.pub"))
key = bytes.fromhex(path.read_text().strip())
if len(key) != 32:
    raise SystemExit(f"{path}: an Ed25519 public key is 32 bytes, got {len(key)}")

if path.resolve() == (project / "keys" / "ota-dev.pub").resolve():
    print("OTA public key: DEV KEY (keys/ota-dev.pub), anyone can sign images for this build")
else:
    print(f"OTA public key: {path}")

env.Append(CPPDEFINES=[("VCU_OTA_PUBLIC_KEY", ",".join(f"0x{b:02x}" for b in key))])
