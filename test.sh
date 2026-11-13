#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${EAE_BUILD_DIR:-${project_dir}/build}"

"${project_dir}/build.sh"
ctest --test-dir "$build_dir" --output-on-failure
