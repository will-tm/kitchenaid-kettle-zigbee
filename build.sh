#!/bin/bash
# Build script for KitchenAid Zigbee Kettle Controller
# Handles complete setup on first run - just clone and run ./build.sh
#
# Usage:
#   ./build.sh [options]
#
# Options (any order):
#   nrf54l15dk      - Target nRF54L15 DK (default)
#   custom          - Target custom board (same as DK for now)
#   clean/pristine  - Clean build
#   flash           - Flash after build (J-Link)
#
# Examples:
#   ./build.sh              # Build for nrf54l15dk
#   ./build.sh flash        # Build and flash
#   ./build.sh clean flash  # Clean build and flash

set -e

BOARD=""
PRISTINE=""
DO_FLASH=""

# Parse all arguments - detect board vs options
for arg in "$@"; do
    case "$arg" in
        pristine|clean)
            PRISTINE="--pristine"
            ;;
        flash)
            DO_FLASH="1"
            ;;
        nrf54l15dk|custom)
            BOARD="$arg"
            ;;
    esac
done

# Default board if not specified
BOARD="${BOARD:-nrf54l15dk}"

# Normalize board names to SDK HWMv2 format (board/soc)
SYSBUILD_CONF=""
case "$BOARD" in
    nrf54l15dk|custom)
        BOARD="nrf54l15dk/nrf54l15/cpuapp"
        # Use base sysbuild (no MCUboot) - MCUboot has SDK compatibility issues
        # To enable OTA later, use: SYSBUILD_CONF="firmware/sysbuild_mcuboot.conf"
        ;;
esac

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# === Setup: Python virtual environment ===
if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate venv
source .venv/bin/activate

# === Setup: Install Python dependencies ===
if ! command -v west &> /dev/null; then
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
fi

# === Setup: Initialize west workspace ===
if [ ! -f ".west/config" ]; then
    echo "Initializing west workspace..."
    mkdir -p .west
    cat > .west/config << 'EOF'
[manifest]
path = .
file = west.yml

[zephyr]
base = deps/zephyr
EOF
fi

# === Setup: Download SDK if needed ===
# Check for deps/zephyr/west.yml as indicator of complete SDK
if [ ! -f "deps/zephyr/west.yml" ]; then
    echo "Downloading nRF Connect SDK (this takes a while, ~4GB)..."
    west update
fi

# === Build ===
echo "========================================"
echo "Building KitchenAid Zigbee Kettle"
echo "Board: ${BOARD}"
echo "========================================"

# Set Zephyr SDK path if Nordic toolchain is installed
if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]; then
    NCS_TOOLCHAIN=$(ls -d /opt/nordic/ncs/toolchains/*/opt/zephyr-sdk 2>/dev/null | head -1)
    if [ -n "$NCS_TOOLCHAIN" ]; then
        export ZEPHYR_SDK_INSTALL_DIR="$NCS_TOOLCHAIN"
        echo "Using SDK: $ZEPHYR_SDK_INSTALL_DIR"
    fi
fi

# Set NRF module directory for MCUboot/OTA integration
export ZEPHYR_NRF_MODULE_DIR="${SCRIPT_DIR}/deps/nrf"

# Build the application using sysbuild (SDK v2.8+)
EXTRA_CMAKE_ARGS="-DZEPHYR_NRF_MODULE_DIR=${SCRIPT_DIR}/deps/nrf"
if [ -n "$SYSBUILD_CONF" ]; then
    EXTRA_CMAKE_ARGS="${EXTRA_CMAKE_ARGS} -DSB_CONF_FILE=${SCRIPT_DIR}/${SYSBUILD_CONF}"
fi
west build -b "${BOARD}" -d build firmware ${PRISTINE} \
    -- ${EXTRA_CMAKE_ARGS}

echo "========================================"
echo "Build complete!"
echo "========================================"

# === Flash ===
if [ -n "$DO_FLASH" ]; then
    echo ""
    echo "Flashing..."

    # DK and other boards use J-Link/nrfjprog
    west flash --build-dir build

    echo "========================================"
    echo "Flash complete!"
    echo "========================================"
else
    echo ""
    echo "Output files in ${SCRIPT_DIR}/build/:"
    echo "  firmware/zephyr/zephyr.hex - Flash via J-Link"
    echo "  merged.hex                  - Combined image"
    echo ""
    echo "To flash: ./build.sh flash"
fi
