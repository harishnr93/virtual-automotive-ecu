#!/usr/bin/env bash
# Runs the Phase 1 demo: Sensor ECU and Cruise ECU exchanging CAN frames.
#
#   ./scripts/run_demo.sh [seconds] [auto|udp|socketcan]
#
# Console output of each ECU and a candump-format trace are written to logs/.
set -euo pipefail

DURATION="${1:-5}"
BUS="${2:-auto}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$ROOT/build/bin"
LOGS="$ROOT/logs"

for exe in vecu_sensor_ecu vecu_cruise_ecu; do
  [ -x "$BIN/$exe" ] || { echo "error: $BIN/$exe not found - build first (see README)" >&2; exit 1; }
done
mkdir -p "$LOGS"

echo "Starting cruise ECU and sensor ECU for ${DURATION}s on bus '$BUS' ..."
"$BIN/vecu_cruise_ecu" --bus "$BUS" --duration "$DURATION" --log "$LOGS/cruise_ecu_can.log" \
  2> >(tee "$LOGS/cruise_ecu.txt" | sed 's/^/[CRUISE] /' >&2) &
CRUISE_PID=$!
"$BIN/vecu_sensor_ecu" --bus "$BUS" --duration "$DURATION" --log "$LOGS/sensor_ecu_can.log" \
  2> >(tee "$LOGS/sensor_ecu.txt" | sed 's/^/[SENSOR] /' >&2) &
SENSOR_PID=$!

trap 'kill $CRUISE_PID $SENSOR_PID 2>/dev/null || true' INT TERM
wait $CRUISE_PID $SENSOR_PID
sleep 0.2  # let the log pipes drain

echo
echo "Logs written to $LOGS:"
ls -1 "$LOGS"
echo
echo "First frames of the cruise ECU CAN trace (candump -l format):"
head -n 8 "$LOGS/cruise_ecu_can.log"
