#!/usr/bin/env bash
# Provision a clean, course-configured EEPROM for the pinned SITL binary.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${1:-$repo_root/.sitl/betaflight-source/obj/main/betaflight_SITL.elf}"
work_dir="${SITL_WORK_DIR:-$repo_root/.sitl/course-angle-mode}"
config="$repo_root/examples/08-betaflight-sitl/config/course_angle_mode.config"

if [[ ! -x "$binary" ]]; then
    echo "SITL binary not found: $binary. Run tools/setup_betaflight_sitl.sh first." >&2
    exit 1
fi

python3 - <<'PY'
"""Refuse to start config-only SITL beside an already-running simulator."""
import socket
import sys

ports = ((socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004), (socket.SOCK_STREAM, 5761))
for socket_type, port in ports:
    probe = socket.socket(socket.AF_INET, socket_type)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError:
        protocol = "UDP" if socket_type == socket.SOCK_DGRAM else "TCP"
        print(f"Cannot provision while {protocol} port {port} is in use. Stop the other SITL/bridge first.", file=sys.stderr)
        sys.exit(1)
    finally:
        probe.close()
PY

mkdir -p "$work_dir"
rm -f "$work_dir/eeprom.bin"
log_file="$work_dir/provision.log"
if ! (
    cd "$work_dir"
    "$binary" --config "$config"
) >"$log_file" 2>&1; then
    tail -40 "$log_file" >&2
    exit 1
fi
if [[ ! -f "$work_dir/eeprom.bin" ]]; then
    echo "SITL did not create $work_dir/eeprom.bin; see $log_file" >&2
    exit 1
fi
echo "Provisioned course SITL EEPROM: $work_dir/eeprom.bin"
echo "Provision log: $log_file"
