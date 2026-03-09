#!/usr/bin/env bash
# scripts/install.sh
#
# Quick-install script for MyFOCLib.
# Builds the static library in Release mode and installs headers + library
# to the system (default: /usr/local) or to a custom prefix.
#
# Usage:
#   ./scripts/install.sh                  # install to /usr/local (may need sudo)
#   ./scripts/install.sh /opt/myfoclib   # install to custom prefix
#   PREFIX=/home/user/.local ./scripts/install.sh
#
# Requirements: cmake >= 3.15, make (or ninja), C compiler

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
PREFIX="${1:-${PREFIX:-/usr/local}}"

echo "=================================================="
echo " MyFOCLib install script"
echo " Source  : ${REPO_ROOT}"
echo " Build   : ${BUILD_DIR}"
echo " Prefix  : ${PREFIX}"
echo "=================================================="

# ---------------------------------------------------------------------------
# Configure
# ---------------------------------------------------------------------------
cmake -S "${REPO_ROOT}" \
      -B "${BUILD_DIR}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${PREFIX}"

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
cmake --build "${BUILD_DIR}" --parallel "$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)"

# ---------------------------------------------------------------------------
# Test (optional — skip with SKIP_TESTS=1)
# ---------------------------------------------------------------------------
if [[ "${SKIP_TESTS:-0}" != "1" ]]; then
    echo ""
    echo "Running unit tests ..."
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
fi

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
echo ""
echo "Installing to ${PREFIX} ..."
cmake --install "${BUILD_DIR}"

echo ""
echo "Installation complete."
echo "  Library : ${PREFIX}/lib/libfoc.a"
echo "  Headers : ${PREFIX}/include/myfoclib/"
echo ""
echo "To use in your CMake project, add:"
echo "  find_library(FOC_LIB foc HINTS ${PREFIX}/lib)"
echo "  target_include_directories(<target> PRIVATE ${PREFIX}/include/myfoclib)"
echo "  target_link_libraries(<target> \${FOC_LIB})"
