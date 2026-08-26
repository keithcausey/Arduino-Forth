# Quick Start Guide - Arduino-Forth

This guide will help you get Arduino-Forth up and running quickly.

## What You Need

### Hardware
- Arduino Mega 1280 or 2560
- USB cable to connect Arduino to your computer
- (Optional) SD card module and SD card for storing FORTH programs

### Software
- Python 3.6 or later
- Arduino IDE 1.8+ or arduino-cli
- Git (to clone the repository)

## Step-by-Step Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/keithcausey/Arduino-Forth.git
cd Arduino-Forth
```

### 2. Build the Outer Interpreter

The outer interpreter needs to be assembled from the assembly source code:

```bash
cd tools
./build.sh
cd ..
```

This will:
- Assemble `ArdufigoTASM/Ardufigo.ASM` 
- Generate `Ardufigo/ArdufigoROM.h`

You should see output like:
```
=====================================================================
Arduino-Forth Build System
=====================================================================

Step 1: Assembling Ardufigo.ASM...
Pass 1: Collecting symbols...
  Found 497 symbols and 497 labels
Pass 2: Generating code...
✓ Assembly successful

Step 2: Converting Intel HEX to C header...
✓ Conversion successful

=====================================================================
Build Complete!
=====================================================================
```

### 3. Open the Arduino Project

**Using Arduino IDE:**
1. Launch Arduino IDE
2. File → Open
3. Navigate to `Ardufigo/Ardufigo.ino`
4. Click Open

**Using arduino-cli:**
```bash
cd Ardufigo
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
```

### 4. Select Your Board

**Arduino IDE:**
1. Tools → Board → Arduino AVR Boards → Arduino Mega or Mega 2560
2. Tools → Processor → ATmega2560 (or ATmega1280 if you have that variant)
3. Tools → Port → Select the port where your Arduino is connected (e.g., COM3 on Windows, /dev/ttyACM0 on Linux)

### 5. Upload to Arduino

**Arduino IDE:**
1. Click the Upload button (→) or Sketch → Upload
2. Wait for compilation and upload to complete

**arduino-cli:**
```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
```

### 6. Connect to the FORTH System

1. Open the Serial Monitor (Tools → Serial Monitor in Arduino IDE)
2. Set baud rate to 115200
3. You should see the Arduino-Forth prompt
4. Try some FORTH commands:
   ```forth
   1 2 + .    ( Should print "3" )
   : HELLO CR ." Hello, World!" ;
   HELLO
   ```

## Troubleshooting

### Build Script Fails

**Problem:** `bash: ./build.sh: Permission denied`
**Solution:** Make the script executable:
```bash
chmod +x tools/build.sh
```

**Problem:** `python3: command not found`
**Solution:** Install Python 3:
- Ubuntu/Debian: `sudo apt-get install python3`
- macOS: `brew install python3`
- Windows: Download from python.org

### Arduino Compilation Fails

**Problem:** "ArdufigoROM.h: No such file or directory"
**Solution:** Run the build script first (step 2)

**Problem:** SD library not found
**Solution:** Install the SD library:
- Arduino IDE: Sketch → Include Library → Manage Libraries → Search "SD" → Install
- arduino-cli: `arduino-cli lib install SD`

### Upload Fails

**Problem:** "Port is not available"
**Solution:** 
1. Check the Arduino is connected
2. Check the correct port is selected
3. Close any other programs using the serial port

**Problem:** "avrdude: stk500v2_ReceiveMessage(): timeout"
**Solution:**
1. Try pressing the reset button on Arduino before uploading
2. Try a different USB cable
3. Check the correct board/processor is selected

### Serial Monitor Shows Garbage

**Problem:** Random characters in serial monitor
**Solution:** Set the correct baud rate (115200)

## Next Steps

Once you have Arduino-Forth running:

1. **Learn FORTH:** The system uses FIG-FORTH syntax. See the FIG-FORTH documentation for language reference.

2. **Explore Built-in Words:** Type `VLIST` to see all available FORTH words.

3. **Use SD Card Storage:** Connect an SD card module to store and load FORTH programs from .BLK files.

4. **Check System Status:** Use `.S` to see the parameter stack, `.SL` to see the loop stack.

5. **Hardware Control:** Arduino-Forth includes words for GPIO control:
   - `INPUT` / `OUTPUT` - Set pin direction
   - `P@` - Read pin
   - `P!` - Write pin
   - `A@` - Read analog pin

## Example Programs

### Blink LED
```forth
: BLINK
  42 22 DO
    OUTPUT I <P>    ( Set pins 22-41 as outputs )
  LOOP
  BEGIN
    42 22 DO
      HIGH I P!     ( Set pin high )
    LOOP
    1000 MILLIS + BEGIN DUP MILLIS < UNTIL DROP
    42 22 DO
      LOW I P!      ( Set pin low )
    LOOP
    1000 MILLIS + BEGIN DUP MILLIS < UNTIL DROP
  AGAIN
;
```

### Simple Calculator
```forth
: CALC
  BEGIN
    KEY DUP EMIT    ( Get and echo character )
    CASE
      43 OF + ENDOF      ( + )
      45 OF - ENDOF      ( - )
      42 OF * ENDOF      ( * )
      47 OF / ENDOF      ( / )
      46 OF . ENDOF      ( . to print )
      SWAP OVER          ( Keep number on stack )
    ENDCASE
  AGAIN
;
```

## Resources

- **FIG-FORTH Documentation:** Original FIG-FORTH manuals explain the language
- **Tools Documentation:** See `tools/README.md` for details on the build system
- **Assembly Source:** `ArdufigoTASM/Ardufigo.ASM` contains the outer interpreter definitions
- **Arduino Source:** `Ardufigo/Ardufigo.ino` contains the inner interpreter and primitives

## Getting Help

If you encounter issues:

1. Check the troubleshooting section above
2. Verify the build completed successfully
3. Check the Arduino serial monitor for error messages
4. Review the tools/README.md for build system details

## License

Arduino-Forth is based on FIG-FORTH which is PUBLIC DOMAIN. All components of this project, including the build tools, are released into the public domain.

You are free to use, modify, and distribute this software for any purpose.
