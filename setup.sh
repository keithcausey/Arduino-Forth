#!/bin/bash
#
# Setup script for Arduino-Forth
# This script checks prerequisites and builds the project
#

set -e

echo "====================================================================="
echo "Arduino-Forth Setup"
echo "====================================================================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    if [ "$1" == "ok" ]; then
        echo -e "${GREEN}✓${NC} $2"
    elif [ "$1" == "error" ]; then
        echo -e "${RED}✗${NC} $2"
    elif [ "$1" == "warn" ]; then
        echo -e "${YELLOW}⚠${NC} $2"
    else
        echo "$2"
    fi
}

# Check for Python 3
echo "Checking prerequisites..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    print_status "ok" "Python 3 found (version $PYTHON_VERSION)"
else
    print_status "error" "Python 3 not found"
    echo ""
    echo "Please install Python 3.6 or later:"
    echo "  Ubuntu/Debian: sudo apt-get install python3"
    echo "  macOS: brew install python3"
    echo "  Windows: Download from python.org"
    exit 1
fi

# Check for Arduino IDE or arduino-cli
ARDUINO_FOUND=false
if command -v arduino-cli &> /dev/null; then
    ARDUINO_VERSION=$(arduino-cli version 2>&1 | head -1)
    print_status "ok" "arduino-cli found ($ARDUINO_VERSION)"
    ARDUINO_FOUND=true
elif [ -d "/usr/share/arduino" ] || [ -d "/Applications/Arduino.app" ] || [ -d "$HOME/Arduino" ]; then
    print_status "ok" "Arduino IDE installation detected"
    ARDUINO_FOUND=true
fi

if [ "$ARDUINO_FOUND" == "false" ]; then
    print_status "warn" "Arduino IDE or arduino-cli not detected"
    echo ""
    echo "You will need Arduino IDE or arduino-cli to compile the project."
    echo "  arduino-cli: https://arduino.github.io/arduino-cli/"
    echo "  Arduino IDE: https://www.arduino.cc/en/software"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Building Arduino-Forth..."
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Change to project root
cd "$PROJECT_ROOT"

# Run the build script
if [ -f "tools/build.sh" ]; then
    bash tools/build.sh
    
    if [ $? -eq 0 ]; then
        echo ""
        print_status "ok" "Build completed successfully!"
        echo ""
        echo "====================================================================="
        echo "Next Steps:"
        echo "====================================================================="
        echo ""
        echo "1. Connect your Arduino Mega to your computer"
        echo ""
        echo "2. Upload the code:"
        echo "   - Using Arduino IDE: Open Ardufigo/Ardufigo.ino"
        echo "   - Using arduino-cli: Run the following commands:"
        echo "       cd Ardufigo"
        echo "       arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 ."
        echo "       arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 ."
        echo ""
        echo "3. Open Serial Monitor at 115200 baud to interact with FORTH"
        echo ""
        echo "For more details, see QUICKSTART.md"
        echo "====================================================================="
    else
        print_status "error" "Build failed!"
        echo ""
        echo "Check the error messages above and ensure all prerequisites are met."
        exit 1
    fi
else
    print_status "error" "Build script not found at tools/build.sh"
    echo "Make sure you're running this from the project root directory."
    exit 1
fi
