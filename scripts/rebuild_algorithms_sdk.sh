#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTOCOL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROTOCOL_ROOT}/../.." && pwd)"
ALGORITHMS_ROOT="${MIGHTY_ALGORITHMS_ROOT:-${REPO_ROOT}/mighty-algorithms}"
ARCH="${1:-macos-arm64}"

if [[ ! -x "${ALGORITHMS_ROOT}/tools/package_sdk.sh" ]]; then
  echo "error: missing ${ALGORITHMS_ROOT}/tools/package_sdk.sh" >&2
  echo "Set MIGHTY_ALGORITHMS_ROOT=/path/to/mighty-algorithms if needed." >&2
  exit 2
fi

COPY_TO_MIGHTY_PROTOCOL=1 \
MIGHTY_PROTOCOL_ALGORITHMS_DIR="${PROTOCOL_ROOT}/lib/algorithms" \
  "${ALGORITHMS_ROOT}/tools/package_sdk.sh" "${ARCH}"
