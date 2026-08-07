#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# Reuse vehicle avoidance while keeping normal cone slalom disabled.
export XYCAR_CONE_AS_VEHICLE_OBSTACLE=true
export XYCAR_AVOIDANCE_IMMEDIATE_DEFAULT=false

exec "$SCRIPT_DIR/run_complete_avoidance_only.sh"
