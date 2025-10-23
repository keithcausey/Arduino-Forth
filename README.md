# Arduino-Forth
A version of Fig Forth for the Arduino Mega1280 and 2560.

The scope of this project is to create an operating system based on Fig-FORTH that uses the Arduino API. 
A brief explanation of FORTH is required here. There is a relatively small but extensible set of 'primitives' written in the native 'C' and 'C++' of the Arduino API, and the outer interpreter written as indirect threaded code references to the primitives. The outer interpreter is assembled from Ardufigo.ASM and converted to a C header file using open-source Python tools. This arrangement allows the compilation of new code using the Arduino Fig-FORTH (Ardufigo), the source of which may be stored on an SD card using a resident FORTH-based editor.

## Quick Start

**New here?** See [QUICKSTART.md](QUICKSTART.md) for step-by-step instructions.

**TL;DR:**
```bash
# 1. Clone and build
git clone https://github.com/keithcausey/Arduino-Forth.git
cd Arduino-Forth
./setup.sh

# 2. Upload to Arduino Mega using Arduino IDE or arduino-cli
# 3. Connect via Serial Monitor at 115200 baud
```

## Directory Structure

- **Ardufigo/** - Contains the Arduino IDE project with the C/C++ code for the Arduino API
- **ArdufigoTASM/** - Contains Ardufigo.ASM, the assembly source for the outer interpreter
- **ArdufigoForth/** - Contains historical documentation and notes
- **tools/** - Contains the build tools (Python-based assembler and converter)

## Building the Project

### Prerequisites

- Python 3.6 or later
- Arduino IDE or arduino-cli (for compiling the Arduino code)
- Arduino Mega 1280 or 2560 board

### Build Steps

1. **Assemble the outer interpreter:**
   ```bash
   cd tools
   ./build.sh
   ```

   This script will:
   - Assemble `ArdufigoTASM/Ardufigo.ASM` using the Python-based TASM-compatible assembler
   - Convert the output to a C header file (`Ardufigo/ArdufigoROM.h`)

2. **Compile and upload the Arduino code:**
   
   Using Arduino IDE:
   - Open `Ardufigo/Ardufigo.ino` in the Arduino IDE
   - Select your Arduino Mega board (Tools → Board → Arduino Mega)
   - Select your serial port (Tools → Port)
   - Click Upload

   Using arduino-cli:
   ```bash
   cd Ardufigo
   arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
   ```

## About the Tools

The original build process used proprietary TASM assembler which has been replaced with open-source Python-based tools:

- **tasm_compat.py** - A TASM-compatible assembler written in Python that supports the TASM directives used in Ardufigo.ASM
- **hex2header.py** - Converts Intel HEX format to C header files with PROGMEM byte arrays
- **build.sh** - Build script that orchestrates the assembly and conversion process

All tools are licensed as Public Domain to match the FIG-FORTH release. See [tools/README.md](tools/README.md) for detailed documentation.

## Additional Files

**INVERSEK.BLK** is the block file that Ardufigo uses for storing and editing FORTH source code. The name INVERSEK.BLK comes from the original project: inverse kinematic calculations for positioning Dynamixel servos. The *.BLK extension is a standard FORTH extension for FORTH source code.

## License

This project is based on FIG-FORTH, which is **PUBLIC DOMAIN**:

> ALL PUBLICATIONS OF THE FORTH INTEREST GROUP
> ARE PUBLIC DOMAIN. THEY MAY BE FURTHER
> DISTRIBUTED BY THE INCLUSION OF THIS CREDIT NOTICE:
> 
> This publication has been made available by the
>   Forth Interest Group
>   P.O.Box 1105
>   San Carlos, CA 94070
>   U.S.A.

All components of this project, including the build tools, maintain this public domain status.
