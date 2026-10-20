#!/usr/bin/env bash
# Runs cppcheck and clang-tidy on the project sources (not on tests or third-party code).
#   ./scripts/run_static_analysis.sh [build-dir]
# The build directory must contain compile_commands.json (CMake generates it).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"
status=0

if [ ! -f "$BUILD/compile_commands.json" ]; then
  echo "error: $BUILD/compile_commands.json missing - configure with CMake first" >&2
  exit 1
fi

mapfile -t SOURCES < <(cd "$ROOT" && find src apps -name '*.cpp' | sort)

if command -v cppcheck >/dev/null; then
  echo "== cppcheck"
  (cd "$ROOT" && cppcheck --std=c++17 --language=c++ --enable=warning,style,performance,portability \
     --inline-suppr --suppress=missingIncludeSystem --suppress=unusedFunction \
     --suppress=passedByValue `# std::string_view is designed to be passed by value` \
     --error-exitcode=1 --quiet -I include -I src -I apps "${SOURCES[@]}") || status=1
else
  echo "cppcheck not installed - skipped"
fi

if command -v clang-tidy >/dev/null; then
  echo "== clang-tidy"
  (cd "$ROOT" && clang-tidy -p "$BUILD" --quiet "${SOURCES[@]}" 2>/dev/null) | tee "$BUILD/clang-tidy.txt"
  if grep -q "warning:" "$BUILD/clang-tidy.txt"; then
    echo "clang-tidy: $(grep -c 'warning:' "$BUILD/clang-tidy.txt") warning(s)"
    status=1
  else
    echo "clang-tidy: no warnings"
  fi
else
  echo "clang-tidy not installed - skipped"
fi

exit $status
