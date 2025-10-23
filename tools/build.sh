#!/bin/bash
#
# Build script for Arduino-Forth project
# Replaces the TASM/dosemu/gforth workflow with open-source Python tools
#
# This script:
# 1. Assembles Ardufigo.ASM using the Python-based TASM-compatible assembler
# 2. Converts the Intel HEX output to a C header file
# 3. Places the header file in the Arduino project directory
#
# Author: Open Source Replacement
# License: Public Domain (matching FIG-FORTH release)

set -e  # Exit on error

# Define paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TOOLS_DIR="$PROJECT_ROOT/tools"
TASM_DIR="$PROJECT_ROOT/ArdufigoTASM"
ARDUINO_DIR="$PROJECT_ROOT/Ardufigo"
BUILD_DIR="$PROJECT_ROOT/build"

# Create build directory if it doesn't exist
mkdir -p "$BUILD_DIR"

echo "====================================================================="
echo "Arduino-Forth Build System"
echo "====================================================================="
echo ""

# Step 1: Assemble Ardufigo.ASM
echo "Step 1: Assembling Ardufigo.ASM..."
python3 "$TOOLS_DIR/tasm_compat.py" \
    "$TASM_DIR/Ardufigo.ASM" \
    "$BUILD_DIR/Ardufigo.hex"

if [ $? -eq 0 ]; then
    echo "✓ Assembly successful"
else
    echo "✗ Assembly failed"
    exit 1
fi

echo ""

# Step 2: Convert HEX to C header
echo "Step 2: Converting Intel HEX to C header..."
python3 "$TOOLS_DIR/hex2header.py" \
    "$BUILD_DIR/Ardufigo.hex" \
    "$ARDUINO_DIR/ArdufigoROM.h" \
    "body" \
    "0x4000"

if [ $? -eq 0 ]; then
    echo "✓ Conversion successful"
else
    echo "✗ Conversion failed"
    exit 1
fi

echo ""

# Show output file info
echo "====================================================================="
echo "Build Complete!"
echo "====================================================================="
echo "Output files:"
echo "  - Intel HEX: $BUILD_DIR/Ardufigo.hex"
echo "  - C Header:  $ARDUINO_DIR/ArdufigoROM.h"
echo ""
echo "You can now compile the Arduino project:"
echo "  cd $ARDUINO_DIR"
echo "  arduino-cli compile --fqbn arduino:avr:mega ."
echo ""
echo "Or open Ardufigo.ino in the Arduino IDE."
echo "====================================================================="
