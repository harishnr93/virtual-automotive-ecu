#!/usr/bin/env bash
# Full regression run: the same stages as the GitHub Actions CI, on the local machine.
#
#   ./scripts/run_regression.sh                 # GCC -Werror, ASan+UBSan, Clang TSan, static analysis
#   ./scripts/run_regression.sh --quick         # GCC -Werror build + all tests only
#   ./scripts/run_regression.sh --label firmware   # only tests with this CTest label
#
# Stages (each in its own build directory, configured on first use):
#   gcc     build/             GCC, RelWithDebInfo, warnings as errors
#   asan    build-asan/        GCC, Debug, AddressSanitizer + UndefinedBehaviorSanitizer
#   tsan    build-tsan-clang/  Clang, Debug, ThreadSanitizer (tests run with ASLR disabled)
#   static  cppcheck + clang-tidy on src/ and apps/ (uses build/compile_commands.json)
#
# Logs and JUnit reports: logs/regression/<stage>.log, logs/regression/<stage>-junit.xml
# Exit code 0 only if every stage passed.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/logs/regression"
JOBS="$(nproc)"
QUICK=0
LABEL=""

while [ $# -gt 0 ]; do
  case "$1" in
    --quick) QUICK=1 ;;
    --label) LABEL="$2"; shift ;;
    --jobs) JOBS="$2"; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (use --help)" >&2; exit 2 ;;
  esac
  shift
done

mkdir -p "$OUT"
declare -a SUMMARY=()
failed=0

# stage NAME BUILD_DIR CXX BUILD_TYPE CMAKE_OPTIONS...
run_stage() {
  local name="$1" dir="$ROOT/$2" cxx="$3" type="$4"
  shift 4
  local log="$OUT/$name.log" start result tests
  start=$(date +%s)
  echo "== [$name] configure + build ($dir)"
  {
    if [ ! -f "$dir/CMakeCache.txt" ]; then
      CXX="$cxx" cmake -S "$ROOT" -B "$dir" -DCMAKE_BUILD_TYPE="$type" -DVECU_WARNINGS_AS_ERRORS=ON "$@"
    fi &&
    cmake --build "$dir" -j"$JOBS"
  } > "$log" 2>&1
  if [ $? -ne 0 ]; then
    SUMMARY+=("$(printf '%-7s %-6s %s' "$name" FAIL "build failed, see $log")")
    failed=1
    return
  fi
  echo "== [$name] tests"
  local label_args=()
  [ -n "$LABEL" ] && label_args=(-L "$LABEL")
  ctest --test-dir "$dir" -j"$JOBS" --output-on-failure --output-junit "$OUT/$name-junit.xml" \
        "${label_args[@]}" >> "$log" 2>&1
  result=$?
  tests=$(grep -E "tests passed, .* tests failed out of" "$log" | tail -1)
  if [ $result -eq 0 ]; then
    SUMMARY+=("$(printf '%-7s %-6s %s (%ss)' "$name" PASS "$tests" $(( $(date +%s) - start )))")
  else
    SUMMARY+=("$(printf '%-7s %-6s %s, see %s' "$name" FAIL "${tests:-tests did not run}" "$log")")
    failed=1
  fi
}

run_stage gcc build g++ RelWithDebInfo
if [ $QUICK -eq 0 ]; then
  run_stage asan build-asan g++ Debug -DVECU_ENABLE_SANITIZERS=ON
  if command -v clang++ >/dev/null; then
    run_stage tsan build-tsan-clang clang++ Debug -DVECU_ENABLE_TSAN=ON
  else
    SUMMARY+=("$(printf '%-7s %-6s %s' tsan SKIP 'clang++ not installed')")
  fi
  echo "== [static] cppcheck + clang-tidy"
  if "$ROOT/scripts/run_static_analysis.sh" "$ROOT/build" > "$OUT/static.log" 2>&1; then
    SUMMARY+=("$(printf '%-7s %-6s %s' static PASS 'cppcheck and clang-tidy: no findings')")
  else
    SUMMARY+=("$(printf '%-7s %-6s %s' static FAIL "see $OUT/static.log")")
    failed=1
  fi
fi

echo
echo "Regression summary ($(date '+%Y-%m-%d %H:%M'), $(uname -r))"
printf '  %s\n' "${SUMMARY[@]}"
exit $failed
