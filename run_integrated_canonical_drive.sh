#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$ROOT_DIR/run_integrated_best512_drive.sh" "$@"
