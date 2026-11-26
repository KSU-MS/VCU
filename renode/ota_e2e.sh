#!/usr/bin/env bash
# Full OTA update of the real ARM firmware in Renode:
#
#   build A (HEAD) runs on the emulated Teensy, build B (another commit, so
#   it reports a different version) goes over vcan_daq with
#   tools/ota/flash.py, gets staged + verified, FlasherX's flash_move copies
#   it over A and resets, and the board has to come back as B.
#
#   nix develop -c renode/ota_e2e.sh [B_REF]     (default HEAD~1)
#
# Needs the vcan module loaded; runs in a user+network namespace, no root.
# Takes a few minutes: the emulator runs ~6x slower than real time.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ -z "${OTA_E2E_IN_NS:-}" ]]; then
  B_REF=${1:-HEAD~1}
  work=$(mktemp -d)
  trap 'git worktree remove --force "$work/b" 2>/dev/null; rm -rf "$work"' EXIT

  pio run -e teensy41
  cp .pio/build/teensy41/firmware.elf "$work/a.elf"

  git worktree add -q "$work/b" "$B_REF"
  (cd "$work/b" && pio run -e teensy41 &&
    python3 tools/ota/image.py build .pio/build/teensy41/firmware.hex -o "$work/b.ota")

  OTA_E2E_IN_NS=1 WORK="$work" unshare --user --map-root-user --net "$0"
  exit $?
fi

for bus in vcan_acc vcan_inv vcan_daq; do
  ip link add dev "$bus" type vcan
  ip link set "$bus" txqueuelen 1000
  ip link set up "$bus"
done

renode --console --disable-gui --plain \
  -e "\$elf=@$WORK/a.elf; include @renode/vcan.resc; logLevel 3; emulation RunFor \"00:15:00\"; quit" \
  > "$WORK/renode.log" 2>&1 &
renode_pid=$!
trap 'kill $renode_pid 2>/dev/null' EXIT

# Give it time to boot before talking to it
sleep 8
python3 tools/ota/flash.py "$WORK/b.ota" --channel vcan_daq --timeout-scale 10
