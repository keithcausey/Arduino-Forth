# Arduino-Forth
A version of Fig Forth for the Arduino Mega1280 and 2560
There are three directories here that are used to separate three conceptually different parts of the compiler for the Fig-FORTH Arduino or "Figuino".
The 'Ardufigo' directory contains the code necessary to operate the Arduino API.
The 'ArdufigoFORTH' directory contains the scripts for the production of translated files for Ardufigo.
The ArdufigoTASM folder compiles the outer interpreter that will be translated by gforth scripts and provided to the Arduino API.

