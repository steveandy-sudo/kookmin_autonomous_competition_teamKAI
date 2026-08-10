#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$ROOT_DIR"

export XYCAR_WS="$WORKSPACE"
export XYCAR_RULE_PERCEPTION_BACKEND=canonical

echo "============================================================"
echo "  my_rule 통합 주행: canonical RULE + CONE + AVOIDANCE"
echo "============================================================"
echo "인지 입력: /perception/canonical_road_image"
echo "주행 우선순위: CONE > YOLO+LiDAR AVOIDANCE > CANONICAL RULE"
echo "최종 모터 명령은 SPACE 게이트가 실행 상태일 때만 발행됩니다."
echo

exec "$WORKSPACE/src/my_drive/scripts/run_complete_rule_only.sh" "$@"
