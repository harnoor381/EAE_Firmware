#!/usr/bin/env bash
set -euo pipefail

# Find this folder even when the script is called from another directory.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${EAE_BUILD_DIR:-${project_dir}/build}"

cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE="${EAE_BUILD_TYPE:-RelWithDebInfo}"
cmake --build "$build_dir" --parallel "${EAE_BUILD_JOBS:-2}"
