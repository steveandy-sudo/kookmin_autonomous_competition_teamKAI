#!/usr/bin/env bash
set -eo pipefail

repo=/home/kai/kookmin_autonomous_competition_teamKAI
annotations=${repo}/analysis/canonical_white_line_annotations

export PYTHONPATH=${repo}/xycar_ws/src/shortcut_entry_review

exec python3 -m shortcut_entry_review.canonical_white_selection_review \
  "${annotations}" \
  --interval-ms 400
