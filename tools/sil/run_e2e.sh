#!/usr/bin/env bash
# Build the SIL VCU and run the end-to-end OTA tests against it.
#
# Without root this re-runs itself in a private user + network namespace,
# where it's allowed to create vcan interfaces (the vcan kernel module has
# to be loaded already: `sudo modprobe vcan`, or boot.kernelModules on
# NixOS). Nothing touches the host's real network config.
#
# If the buses already exist (tools/sil/setup_vcan.sh, or CI) it just uses
# them; as root (e.g. a container with NET_ADMIN) it creates them in place.
set -euo pipefail
cd "$(dirname "$0")/../.."

have_buses() {
  for bus in vcan_acc vcan_inv vcan_daq; do
    ip link show "$bus" &>/dev/null || return 1
  done
}

# Build outside the namespace, platformio may need the real network
if [[ -z "${VCU_E2E_IN_NS:-}" ]]; then
  pio run -e sil

  if ! have_buses && [[ $EUID -ne 0 ]]; then
    exec env VCU_E2E_IN_NS=1 unshare --user --map-root-user --net "$0" "$@"
  fi
fi

if ! have_buses; then
  for bus in vcan_acc vcan_inv vcan_daq; do
    ip link add dev "$bus" type vcan
    ip link set "$bus" txqueuelen 1000
    ip link set up "$bus"
  done
fi

python3 -m pytest tools/sil -v -s "$@"
