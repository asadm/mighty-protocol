#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTOCOL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROTOCOL_ROOT}/../.." && pwd)"
ALGORITHMS_ROOT="${MIGHTY_ALGORITHMS_ROOT:-${REPO_ROOT}/mighty-algorithms}"

ARCH="${1:-macos-arm64-static}"

if [[ ! -x "${ALGORITHMS_ROOT}/tools/package_loopclosure_device.sh" ]]; then
  echo "error: missing ${ALGORITHMS_ROOT}/tools/package_loopclosure_device.sh" >&2
  echo "Set MIGHTY_ALGORITHMS_ROOT=/path/to/mighty-algorithms if needed." >&2
  exit 2
fi

if [[ "$(uname -s)" == "Darwin" && "${ARCH}" == linux-* ]]; then
  if [[ "${ALGORITHMS_ROOT}" != "${REPO_ROOT}/mighty-algorithms" ]]; then
    echo "error: Docker Linux packaging expects mighty-algorithms beside mighty-web under ${REPO_ROOT}" >&2
    exit 2
  fi
  if ! command -v docker >/dev/null 2>&1; then
    echo "error: Docker is required to package ${ARCH} from macOS" >&2
    exit 2
  fi

  PLATFORM="linux/amd64"
  if [[ "${ARCH}" == *"armv7"* ]]; then
    PLATFORM="linux/arm/v7"
  elif [[ "${ARCH}" == *"arm64"* || "${ARCH}" == *"aarch64"* ]]; then
    PLATFORM="linux/arm64"
  fi

  HOST_UID="$(id -u)"
  HOST_GID="$(id -g)"
  docker run --rm --platform "${PLATFORM}" \
    -v "${REPO_ROOT}:/work" \
    -w /work/mighty-algorithms \
    ubuntu:22.04 \
    bash -lc "
      set -euo pipefail
      export DEBIAN_FRONTEND=noninteractive
      apt-get update >/dev/null
      apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        build-essential \
        cmake \
        pkg-config \
        libeigen3-dev \
        binutils \
        patchelf >/dev/null
      STATIC_OPENCV=${STATIC_OPENCV:-1} \
      STRIP_BINARIES=${STRIP_BINARIES:-1} \
      COPY_TO_MIGHTY_PROTOCOL=1 \
      MIGHTY_PROTOCOL_LOOPCLOSURE_DIR=/work/mighty-web/mighty-protocol/lib/loopclosure \
      BUILD_JOBS=${BUILD_JOBS:-2} \
        tools/package_loopclosure_device.sh '${ARCH}'
      chown -R ${HOST_UID}:${HOST_GID} \
        dist \
        .deps \
        build-opencv-minimal-${ARCH} \
        mighty-loopclosure-device/build-${ARCH} \
        /work/mighty-web/mighty-protocol/lib/loopclosure/${ARCH} 2>/dev/null || true
    "
  exit 0
fi

STATIC_OPENCV="${STATIC_OPENCV:-1}" \
STRIP_BINARIES="${STRIP_BINARIES:-1}" \
COPY_TO_MIGHTY_PROTOCOL=1 \
MIGHTY_PROTOCOL_LOOPCLOSURE_DIR="${PROTOCOL_ROOT}/lib/loopclosure" \
"${ALGORITHMS_ROOT}/tools/package_loopclosure_device.sh" "${ARCH}"
