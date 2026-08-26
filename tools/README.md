# Build Tools Documentation

This document describes the open-source build tools that replace the proprietary TASM assembler.

## Overview

The original Arduino-Forth build process relied on:
- TASM (proprietary assembler) running in dosemu
- gforth scripts for file format conversion

The new build process uses:
- Python-based TASM-compatible assembler (tasm_compat.py)
- Python-based Intel HEX to C header converter (hex2header.py)
- Bash build script (build.sh)

All tools are public domain to match the FIG-FORTH license.

## Tool Details

### tasm_compat.py

A Python implementation of a TASM-compatible assembler that processes Ardufigo.ASM.

**Supported TASM Directives:**
- `.dw` / `DW` - Define word (16-bit little-endian)
- `.db` / `defb` - Define byte
- `.TEXT` / `defc` - Define text string
- `.ORG` - Set origin address
- `.equ` / `equ` - Define constant/symbol
- `.MSFIRST` - Set little-endian mode
- `.BYTE` - Define byte(s)
- `.page` - Page directive (ignored)
- `.END` - End of assembly
- `#define` - C-style macro definition

**Features:**
- Two-pass assembly for forward references
- Symbol and label resolution
- Hex number support (suffix 'h', prefix '0x')
- Arithmetic expression evaluation
- Intel HEX output format

**Usage:**
```bash
python3 tasm_compat.py <input.asm> <output.hex>
```

**Example:**
```bash
python3 tasm_compat.py ArdufigoTASM/Ardufigo.ASM build/Ardufigo.hex
```

### hex2header.py

Converts Intel HEX format to C/C++ header files with PROGMEM byte arrays for Arduino.

**Features:**
- Parses Intel HEX format
- Generates formatted C header with PROGMEM declaration
- Includes ASCII representation in comments
- Customizable array name and size

**Usage:**
```bash
python3 hex2header.py <input.hex> <output.h> [array_name] [array_size_hex]
```

**Example:**
```bash
python3 hex2header.py build/Ardufigo.hex Ardufigo/ArdufigoROM.h body 0x4000
```

**Output Format:**
```c
const PROGMEM byte body[0x4000] = {
0x3C, 0x00, 0x12, 0x15, ... // <... 0000
0x00, 0x1F, 0x00, 0x0F, ... // .... 0010
...
};
```

### build.sh

Master build script that orchestrates the entire build process.

**Features:**
- Automated assembly and conversion
- Error checking at each step
- Progress reporting
- Creates build directory automatically

**Usage:**
```bash
cd tools
./build.sh
```

The script will:
1. Create the build directory if needed
2. Run tasm_compat.py to assemble Ardufigo.ASM
3. Run hex2header.py to generate ArdufigoROM.h
4. Report success and provide next steps

## Build Process Flow

```
Ardufigo.ASM
     |
     v
[tasm_compat.py]  ← Two-pass assembly
     |                - Pass 1: Collect symbols/labels
     |                - Pass 2: Generate code
     v
Ardufigo.hex (Intel HEX format)
     |
     v
[hex2header.py]   ← Format conversion
     |
     v
ArdufigoROM.h (C header with PROGMEM array)
     |
     v
[Arduino IDE / arduino-cli]  ← Compilation
     |
     v
Ardufigo.ino.hex (Arduino firmware)
```

## Intel HEX Format

The assembler outputs standard Intel HEX format:
```
:LLAAAATT[DD...]CC
```
Where:
- `LL` = Record length (number of data bytes)
- `AAAA` = Address (16-bit)
- `TT` = Record type (00=data, 01=EOF)
- `DD` = Data bytes
- `CC` = Checksum (two's complement of sum)

## Differences from TASM

**Compatibility:**
- Supports all TASM directives used in Ardufigo.ASM
- Handles forward label references
- Processes #define macros
- Little-endian word ordering (.MSFIRST)

**Known Limitations:**
- Does not support CPU instruction mnemonics (not needed for this project)
- No macro expansion beyond #define
- Limited conditional assembly support

These limitations do not affect the Arduino-Forth project as Ardufigo.ASM only uses data directives (DW, defb, defc) for the outer interpreter definition.

## Troubleshooting

**Problem:** Assembly fails with symbol resolution errors
**Solution:** Check that all labels are defined before use, or ensure they are defined somewhere in the file for the two-pass assembler to find.

**Problem:** Generated header differs from original
**Solution:** This is expected. The Python assembler may organize data slightly differently but produces functionally equivalent output. Verify the Arduino code compiles successfully.

**Problem:** Build script fails on permission denied
**Solution:** Ensure scripts are executable: `chmod +x tools/*.py tools/*.sh`

**Problem:** Python not found
**Solution:** Install Python 3.6+: `sudo apt-get install python3`

## Development and Testing

To test the assembler on a simple file:
```bash
cat > test.asm << 'EOF'
.ORG 0
test:   DW 0x1234
        defb 0x56, 0x78
        defc "HELLO"
.END
EOF

python3 tasm_compat.py test.asm test.hex
python3 hex2header.py test.hex test.h
```

## License

All build tools are released into the PUBLIC DOMAIN to match the FIG-FORTH license:

> ALL PUBLICATIONS OF THE FORTH INTEREST GROUP
> ARE PUBLIC DOMAIN. THEY MAY BE FURTHER
> DISTRIBUTED BY THE INCLUSION OF THIS CREDIT NOTICE

This means you are free to:
- Use the tools for any purpose
- Modify the tools
- Distribute the tools
- Incorporate the tools into other projects

No warranty is provided. The tools are provided "as is".

## Contributing

Improvements to the build tools are welcome. Consider:
- Adding support for more TASM directives
- Improving error messages
- Adding unit tests
- Optimizing assembly performance
- Supporting additional output formats

## References

- FIG-FORTH Release 1.1 Documentation
- TASM Assembler Manual (historical reference)
- Intel HEX File Format Specification
- Arduino PROGMEM Documentation
