#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

export XYCAR_CONE_AS_VEHICLE_OBSTACLE=true
export XYCAR_AVOIDANCE_IMMEDIATE_DEFAULT=false
export XYCAR_STEERING_ONLY=true

exec "$SCRIPT_DIR/run_complete_avoidance_only.sh"
