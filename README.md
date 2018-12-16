# Arduino-Forth
A version of Fig Forth for the Arduino Mega1280 and 2560.
The scope of this project is to create an operating system based on Fig-FORTH that uses the Arduino API. 
A brief explanation of FORTH is required here. There is a relatively small but extensible set of 'primatives' written in the native 'C' and 'C++' of the Arduino API, and the outer interpreter written as indirect threaded code references to the primatives. This outer interpreter is fed to the Arduino API via a gforth translation script reading the output of the TASM assembler. This arrangement allows the compilation of new code using the Arduino Fig-FORTH (Ardufigo), the source of which may be stored on an SD card using a resident FORTH-based editor.

There are three directories here that are used to separate three conceptually different parts of the compiler for the system.
The 'Ardufigo' directory contains the code necessary to operate the Arduino API.
The 'ArdufigoFORTH' directory contains the scripts for the production of translated files for Ardufigo.
The ArdufigoTASM folder compiles the outer interpreter that will be translated by gforth scripts and provided to the Arduino API.
There is another file that is tentatively entitled INVERSEK.BLK which is what the Ardufigo system uses for storing and editing its source code.
