#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TCAM_ENV="/home/daniel/tiscamera/build/env.sh"
if [[ -f "${TCAM_ENV}" ]]; then
  # shellcheck source=/home/daniel/tiscamera/build/env.sh
  set +u
  source "${TCAM_ENV}"
  set -u
fi

python3 "${SCRIPT_DIR}/scripts/software_sync_capture.py" \
  --config "${SCRIPT_DIR}/config/stereo_software_trigger.json" \
  "$@"
