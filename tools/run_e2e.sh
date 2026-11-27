#!/usr/bin/env bash
# Build what the end-to-end tests need, then run them with pytest on virtual
# CAN (vcan_acc/inv/daq):
#
#   tools/run_e2e.sh                  # all of them
#   tools/run_e2e.sh tools/sil        # VCU as a Linux program (seconds)
#   tools/run_e2e.sh renode           # real firmware in Renode (minutes)
#   tools/run_e2e.sh renode --flash-timing=max
#
# Arguments go to pytest. Without root this re-runs itself in a private user
# + network namespace, where it's allowed to create vcan interfaces (the vcan
# kernel module has to be loaded already: `sudo modprobe vcan`, or
# boot.kernelModules on NixOS). Nothing touches the host's real network.
#
# If the buses already exist (tools/sil/setup_vcan.sh) it just uses them; as
# root (e.g. a container with NET_ADMIN) it creates them in place.
set -euo pipefail
cd "$(dirname "$0")/.."

have_buses() {
  for bus in vcan_acc vcan_inv vcan_daq; do
    ip link show "$bus" &>/dev/null || return 1
  done
}

# Build outside the namespace, platformio and uv may need the real network. The
# Renode test builds a second firmware itself, reusing these libdeps.
if [[ -z "${VCU_E2E_IN_NS:-}" ]]; then
  pio run -e sil -e teensy41
  uv sync --frozen

  if ! have_buses && [[ $EUID -ne 0 ]]; then
    exec env VCU_E2E_IN_NS=1 unshare --user --map-root-user --net "$0" "$@"
  fi
fi

if [[ -n "${VCU_E2E_IN_NS:-}" ]]; then
  ip link set lo up # the Renode tests reach its monitor over TCP
fi

if ! have_buses; then
  for bus in vcan_acc vcan_inv vcan_daq; do
    ip link add dev "$bus" type vcan
    ip link set "$bus" txqueuelen 1000
    ip link set up "$bus"
  done
fi

(( $# )) || set -- tools/sil renode
# No network in the namespace: use the environment synced above
exec uv run --frozen --no-sync pytest -v -s "$@"
