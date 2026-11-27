# PlatformIO post script for the slot builds (teensy41, teensy41_b): sign the
# firmware as an OTA image, and for slot A turn firmware.hex into the USB
# bundle (bootloader + fresh boot state + slot A) so `-t upload` sets up a
# whole board. The plain slot A firmware is kept as app.hex.
#
# Signs with $VCU_OTA_KEY, default the dev key (see keys/README.md). Runs
# tools/ota/image.py with $VCU_PYTHON if set, else through uv.
import os
import shutil
import subprocess
from pathlib import Path

Import("env")


def slot_of(env):
    for d in env["CPPDEFINES"]:
        if isinstance(d, (tuple, list)) and d[0] == "VCU_SLOT":
            return "ab"[int(d[1])]
    raise SystemExit("vcu_images.py: VCU_SLOT isn't defined")


def make_images(source, target, env):
    project = Path(env.subst("$PROJECT_DIR"))
    build = Path(env.subst("$BUILD_DIR"))
    slot = slot_of(env)
    image_py = [os.environ["VCU_PYTHON"]] if "VCU_PYTHON" in os.environ else [
        "uv", "run", "--frozen", "--project", str(project), "python"]
    image_py.append(str(project / "tools/ota/image.py"))

    def run(cmd):
        print(" ".join(str(c) for c in cmd))
        subprocess.run([str(c) for c in cmd], check=True, cwd=project)

    hexfile = build / "firmware.hex"
    app = build / "app.hex"
    ota = build / f"vcu-{slot}.ota"
    shutil.copyfile(hexfile, app)
    run(image_py + ["build", app, "-o", ota])

    if slot == "a":
        run([env.subst("$PYTHONEXE"), "-m", "platformio", "run", "-s",
             "-e", "bootloader", "-d", project])
        run(image_py + ["bundle", ota, "--bootloader",
                        project / ".pio/build/bootloader/firmware.hex", "-o", hexfile])


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", make_images)
