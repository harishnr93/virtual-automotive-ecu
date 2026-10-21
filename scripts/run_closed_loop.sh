#!/usr/bin/env bash
# Closed-loop run: cruise control ECU (C++) + vehicle simulator (Python) on one CAN bus.
#
#   ./scripts/run_closed_loop.sh [scenario.json] [auto|udp|socketcan]
#
# Results in logs/closed_loop/<scenario>.{csv,svg,json} and the ECU console log.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-$ROOT/config/scenarios/cruise_full_demo.json}"
BUS="${2:-auto}"
ECU="$ROOT/build/bin/vecu_cruise_ecu"
OUT="$ROOT/logs/closed_loop"

[ -x "$ECU" ] || { echo "error: $ECU not found - build first (see README)" >&2; exit 1; }
[ -f "$SCENARIO" ] || { echo "error: scenario $SCENARIO not found" >&2; exit 1; }
mkdir -p "$OUT"

NAME="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["name"])' "$SCENARIO")"
DURATION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["duration_s"])' "$SCENARIO")"
ECU_DURATION="$(python3 -c "print($DURATION + 1.5)")"

echo "Scenario '$NAME' (${DURATION}s) on bus '$BUS'"
"$ECU" --bus "$BUS" --duration "$ECU_DURATION" --log "$OUT/$NAME.can.log" \
  --timing-report "$OUT/$NAME.timing.json" 2> "$OUT/$NAME.ecu.txt" &
ECU_PID=$!
trap 'kill $ECU_PID 2>/dev/null || true' INT TERM
sleep 0.5  # let the ECU start its tasks

PYTHONPATH="$ROOT/simulator" python3 -m vecu_sim.simulator --scenario "$SCENARIO" --bus "$BUS" \
  --csv "$OUT/$NAME.csv" --svg "$OUT/$NAME.svg" --report "$OUT/$NAME.json"
wait $ECU_PID

echo
echo "ECU state changes:"
grep -E "CRUISE_STATE|RX_TIMEOUT|RX_RECOVERED|DEADLINE_MISS" "$OUT/$NAME.ecu.txt" | sed 's/^/  /'
echo
echo "Results: $OUT/$NAME.{csv,svg,json}, ECU log $OUT/$NAME.ecu.txt"
