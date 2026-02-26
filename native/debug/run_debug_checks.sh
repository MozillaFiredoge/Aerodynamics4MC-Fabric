#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOD_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
CLASSES_DIR="${BUILD_DIR}/classes"
RESOURCES_DIR="${BUILD_DIR}/resources"

detect_os() {
  local uname_s
  uname_s="$(uname -s)"
  case "${uname_s}" in
    Linux*) echo "linux" ;;
    Darwin*) echo "macos" ;;
    MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
    *)
      echo "Unsupported OS for debug harness: ${uname_s}" >&2
      exit 2
      ;;
  esac
}

detect_arch() {
  local uname_m
  uname_m="$(uname -m)"
  case "${uname_m}" in
    x86_64|amd64) echo "x86_64" ;;
    arm64|aarch64) echo "arm64" ;;
    *)
      echo "Unsupported CPU arch for debug harness: ${uname_m}" >&2
      exit 2
      ;;
  esac
}

native_filename() {
  local os="$1"
  case "${os}" in
    linux) echo "libaero_lbm.so" ;;
    macos) echo "libaero_lbm.dylib" ;;
    windows) echo "aero_lbm.dll" ;;
    *) return 1 ;;
  esac
}

pick_native_library() {
  local os="$1"
  local arch="$2"
  local file="$3"

  local candidates=(
    "${MOD_ROOT}/native/dist/natives/${os}-${arch}/${file}"
    "${MOD_ROOT}/native/dist/${os}-${arch}/${file}"
    "${MOD_ROOT}/native/build-${os}-${arch}/${file}"
    "${MOD_ROOT}/native/build/${file}"
    "${MOD_ROOT}/native/build/Release/${file}"
  )

  local p
  for p in "${candidates[@]}"; do
    if [[ -f "${p}" ]]; then
      echo "${p}"
      return 0
    fi
  done

  echo "No native library found for ${os}-${arch}. Checked:" >&2
  for p in "${candidates[@]}"; do
    echo "  - ${p}" >&2
  done
  return 1
}

OS_NAME="$(detect_os)"
ARCH_NAME="$(detect_arch)"
LIB_FILE="$(native_filename "${OS_NAME}")"
LIB_PATH="$(pick_native_library "${OS_NAME}" "${ARCH_NAME}" "${LIB_FILE}")"
RESOURCE_NATIVE_DIR="${RESOURCES_DIR}/natives/${OS_NAME}-${ARCH_NAME}"

rm -rf "${BUILD_DIR}"
mkdir -p "${CLASSES_DIR}" "${RESOURCE_NATIVE_DIR}"
cp "${LIB_PATH}" "${RESOURCE_NATIVE_DIR}/${LIB_FILE}"

echo "[debug] using native: ${LIB_PATH}"

javac --release 21 -d "${CLASSES_DIR}" \
  "${MOD_ROOT}/src/client/java/com/aerodynamics4mc/client/NativeLibraryLoader.java" \
  "${MOD_ROOT}/src/client/java/com/aerodynamics4mc/client/NativeLbmBridge.java" \
  "${SCRIPT_DIR}/java/com/aerodynamics4mc/client/NativeLbmDebugHarness.java"

java -cp "${CLASSES_DIR}:${RESOURCES_DIR}" \
  com.aerodynamics4mc.client.NativeLbmDebugHarness "$@"
