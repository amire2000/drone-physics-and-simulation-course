#!/usr/bin/env bash
# Provision the course configuration, then start the normal SITL process.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${1:-$repo_root/.sitl/betaflight-source/obj/main/betaflight_SITL.elf}"
work_dir="${SITL_WORK_DIR:-$repo_root/.sitl/course-angle-mode}"

"$repo_root/tools/provision_betaflight_sitl.sh" "$binary"
cd "$work_dir"
exec "$binary" --ip 127.0.0.1
