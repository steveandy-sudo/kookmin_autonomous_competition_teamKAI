#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_MODE=rule exec "$SCRIPT_DIR/run_complete_space_hybrid.sh" "${1:-}"
