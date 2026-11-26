#!/usr/bin/env bash
# Create the three virtual CAN buses the SIL VCU uses. Needs root.
#
#   sudo tools/sil/setup_vcan.sh          # create
#   sudo tools/sil/setup_vcan.sh down     # remove
#
# On NixOS you can make this permanent with boot.kernelModules = [ "vcan" ];
# plus a systemd-networkd netdev, but running this once per boot is fine.
set -euo pipefail

BUSES=(vcan_acc vcan_inv vcan_daq)

if [[ "${1:-up}" == "down" ]]; then
  for bus in "${BUSES[@]}"; do
    ip link del "$bus" 2>/dev/null || true
  done
  exit 0
fi

modprobe vcan
for bus in "${BUSES[@]}"; do
  if ! ip link show "$bus" &>/dev/null; then
    ip link add dev "$bus" type vcan
  fi
  # Deeper TX queue than the default 10, ISO-TP bursts overflow it
  ip link set "$bus" txqueuelen 1000
  ip link set up "$bus"
done
ip -brief link show type vcan
