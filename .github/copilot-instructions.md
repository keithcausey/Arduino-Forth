# GitHub Copilot Instructions for Arduino-Forth

## Project Overview

This is a Fig-FORTH implementation for the Arduino Mega 1280/2560 microcontrollers. The project creates an operating system based on Fig-FORTH that uses the Arduino API with the following architecture:

- **Primitives**: Written in C/C++ using the Arduino API
- **Outer Interpreter**: Indirect threaded code referencing the primitives
- **Compilation**: TASM assembler output translated by gforth scripts for Arduino API
- **Storage**: Source code stored on SD card using a FORTH-based editor

## Hardware Constraints

Target platform: ATMega 256 (Arduino Mega 1280/2560)
- 256KB Flash memory (In-System Self-Programmable)
- 4KB EEPROM
- 8KB Internal SRAM

Always consider these memory constraints when modifying code.

## Project Structure

### Directories
- **Ardufigo/**: Contains Arduino API code
  - `Ardufigo.ino`: Main Arduino sketch
  - `ArdufigoROM.h`: ROM table containing the Forth program (stored in FLASH)
  - `ArdufigoRAM.h`: SRAM table definitions
  - `INVERSEK.*`: Forth source code blocks

- **ArdufigoForth/**: Gforth translation scripts
  - `fileReadWrite.fs`: Converts Intel HEX to Arduino-compatible format
  - `saveMe.sh`: Build automation script

- **ArdufigoTASM/**: TASM assembler source
  - `Ardufigo.ASM`: Fig-FORTH assembly source for outer interpreter

## Coding Standards

### Arduino C/C++ Code

1. **Memory Management**
   - Use `PROGMEM` directive for large data tables stored in Flash
   - Be mindful of SRAM usage (only 8KB available)
   - Use EEPROM for persistent storage when appropriate

2. **Include Directives**
   - Use standard Arduino libraries: `<avr/pgmspace.h>`, `<EEPROM.h>`, `<SD.h>`
   - Keep commented-out includes for reference (e.g., `<SPI.h>`)

3. **Code Style**
   - Use clear, descriptive variable names (e.g., `IP` for instruction pointer, `WA` for word address)
   - Include inline comments explaining virtual machine operations
   - Use `#define` for hardware pins (e.g., `#define ledPin 13`)

4. **Debugging**
   - Use conditional compilation with `#define DEVMODE 1` for debug code
   - Comment out debug code rather than deleting it

### Assembly Code (TASM)

1. Follow Fig-FORTH conventions for word definitions
2. Maintain link fields and name headers
3. Keep original FIG-FORTH credits and attribution comments

### Forth Code

1. Use traditional Forth naming conventions (uppercase, hyphenated words)
2. Include stack effect comments: `( before -- after )`
3. Document complex word definitions with inline comments using `\`

## Build Process

The build process involves three stages:
1. **TASM Assembly**: Assembles Forth definitions into Intel HEX format
2. **Gforth Translation**: Converts Intel HEX to Arduino-compatible C arrays
3. **Arduino IDE Compilation**: Compiles final sketch for ATMega 256

When modifying build scripts:
- Test each stage independently
- Preserve intermediate file formats for debugging
- Maintain compatibility with dosemu (for TASM) and gforth

## File Formats

### Intel HEX Format
Input format from TASM assembler:
```
:10010000214601360121470136007EFE09D2190140
```

### Arduino Array Format
Output format for ArdufigoROM.h:
```c
const byte body[16384] PROGMEM = {
0x20, 0x00, 0x0D, 0x97, ...
```

## Testing

- Test on actual Arduino Mega hardware when possible
- Verify memory usage doesn't exceed platform constraints
- Test serial port communication (used for Forth console)
- Validate SD card file operations

## Security Considerations

- Do not hardcode passwords or sensitive data
- Validate all external input from serial port
- Check array bounds when accessing ROM/RAM tables
- Handle SD card errors gracefully

## Documentation

- Update README.md for significant architectural changes
- Document new Forth words with stack effects and descriptions
- Include comments explaining memory layout changes
- Keep notes.txt updated with build process changes

## Common Pitfalls

- **Memory Overflow**: Always check that data fits within Flash/SRAM limits
- **Stack Effects**: Verify Forth word stack effects match documentation
- **Serial Port**: Remember half-duplex port requires direction switching (pin 42)
- **File Paths**: Build scripts use absolute paths specific to development environment
