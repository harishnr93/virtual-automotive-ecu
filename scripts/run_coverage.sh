#!/usr/bin/env bash
# Code coverage of the whole test suite (GCC gcov, no extra tools).
#
#   ./scripts/run_coverage.sh                    # build-coverage/, all tests, docs/coverage_report.md
#   ./scripts/run_coverage.sh --report /tmp/c.md
#
# The ECU processes started by the system tests write their counters too (also on a watchdog
# emergency exit: coverage builds flush the counters before _Exit).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$ROOT/build-coverage"
JOBS="$(nproc)"

if [ ! -f "$BUILD/CMakeCache.txt" ]; then
  CXX=g++ cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Debug -DVECU_ENABLE_COVERAGE=ON -DVECU_WARNINGS_AS_ERRORS=ON
fi
cmake --build "$BUILD" -j"$JOBS"
find "$BUILD" -name '*.gcda' -delete            # counters from earlier runs
ctest --test-dir "$BUILD" -j"$JOBS" --output-on-failure || echo "warning: some tests failed; coverage still reported"
python3 "$ROOT/scripts/coverage_report.py" "$BUILD" "$@"
