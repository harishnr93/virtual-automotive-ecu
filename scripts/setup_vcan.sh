#!/usr/bin/env bash
# Creates the virtual CAN interface vcan0 (requires root and a kernel with CONFIG_CAN_VCAN).
#
#   sudo ./scripts/setup_vcan.sh            # create vcan0
#   sudo ./scripts/setup_vcan.sh vcan1      # other name
#
# On stock WSL2 kernels the vcan module does not exist; the ECUs then fall back to the
# UDP software CAN bus automatically (--bus auto), so this script is optional there.
set -euo pipefail

IFACE="${1:-vcan0}"

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run as root (sudo $0 $IFACE)" >&2
  exit 1
fi

if ! modprobe vcan 2>/dev/null; then
  echo "error: kernel module 'vcan' not available on $(uname -r)." >&2
  echo "       Use the UDP software CAN bus instead (default --bus auto), or build a" >&2
  echo "       WSL2 kernel with CONFIG_CAN_VCAN=m (see docs/phase1_virtual_can.md)." >&2
  exit 2
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  ip link add dev "$IFACE" type vcan
fi
ip link set up "$IFACE"
ip -details link show "$IFACE"
echo "$IFACE is up. Monitor with: candump -td $IFACE"
