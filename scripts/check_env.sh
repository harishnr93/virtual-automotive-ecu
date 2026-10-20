#!/usr/bin/env bash
# Verifies the development environment for the Virtual Automotive ECU Platform.
# Read-only: installs nothing, needs no sudo.
set -u

ok()   { printf '  [ OK ] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; }
fail() { printf '  [FAIL] %s\n' "$*"; missing=1; }
missing=0

tool() {  # tool <command> <label> [version-args]
  if command -v "$1" >/dev/null 2>&1; then
    ok "$2: $("$1" ${3:---version} 2>&1 | head -n1)"
  else
    fail "$2 not found"
  fi
}

echo "== System"
ok "Kernel: $(uname -r)"
grep -qi microsoft /proc/version 2>/dev/null && ok "Running under WSL" || ok "Native Linux"

echo "== Toolchain"
tool g++ "g++"
tool cmake "CMake"
tool gdb "GDB"
tool git "Git"
tool python3 "Python"
python3 -c "import pytest" 2>/dev/null && ok "pytest: $(python3 -c 'import pytest;print(pytest.__version__)')" \
  || fail "pytest missing (sudo apt install python3-pytest)"
[ -f /usr/include/gtest/gtest.h ] && ok "GoogleTest headers found" || fail "GoogleTest missing (sudo apt install libgtest-dev)"

echo "== Static analysis (optional)"
command -v cppcheck   >/dev/null && ok "cppcheck: $(cppcheck --version)" || warn "cppcheck not installed"
command -v clang-tidy >/dev/null && ok "clang-tidy: $(clang-tidy --version | grep -m1 version)" || warn "clang-tidy not installed"

echo "== SocketCAN / vcan"
config=""
if [ -r /proc/config.gz ]; then config=$(zcat /proc/config.gz); fi
if [ -z "$config" ] && [ -r "/boot/config-$(uname -r)" ]; then config=$(cat "/boot/config-$(uname -r)"); fi
if [ -n "$config" ]; then
  echo "$config" | grep -Eq '^CONFIG_CAN=(y|m)'      && ok "CONFIG_CAN enabled"      || warn "CONFIG_CAN disabled"
  echo "$config" | grep -Eq '^CONFIG_CAN_RAW=(y|m)'  && ok "CONFIG_CAN_RAW enabled"  || warn "CONFIG_CAN_RAW disabled"
  echo "$config" | grep -Eq '^CONFIG_CAN_VCAN=(y|m)' && ok "CONFIG_CAN_VCAN enabled" \
    || warn "CONFIG_CAN_VCAN not set -> no vcan0; ECUs use the UDP software CAN bus"
else
  warn "kernel config not readable"
fi
python3 - <<'EOF' && ok "AF_CAN raw socket can be created" || warn "AF_CAN sockets not available"
import socket
socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW).close()
EOF
if [ -e /sys/class/net/vcan0 ]; then ok "vcan0 present ($(cat /sys/class/net/vcan0/operstate))"; else warn "vcan0 not present"; fi
command -v candump >/dev/null && ok "can-utils installed" || warn "can-utils not installed (sudo apt install can-utils)"

echo
if [ "$missing" -eq 0 ]; then echo "Environment OK."; else echo "Required tools missing (see [FAIL] above)."; fi
exit "$missing"
