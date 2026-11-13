#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${EAE_BUILD_DIR:-${project_dir}/build}"

# Build first, then pass every runtime argument to the program unchanged.
"${project_dir}/build.sh" >&2
exec "${build_dir}/eae_firmware" "$@"
