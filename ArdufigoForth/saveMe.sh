#!/bin/sh

dosemu /media/sdc1/Software/Arduino.Sketch.Book/ArdufigoForth/drive_c/Z80FIGFO.BAT
#echo Step 1
#read -p "Press any key..."

#dosemu z80figfo.bat
#echo Step 2
#read -p "Press any key..."

#cp /home/knoppix/.dosemu/drive_c/z80figfo.obj ~/.gforth/readFileTest.txt
#echo Step 3
#read -p "Press any key..."

#cd ~/.gforth
#echo Step 4
#read -p "Press any key..."

#gforth fileReadWrite.fs
#echo Step 5
#read -p "Press any key..."
#cp /home/knoppix/.gforth/writeFileTest.txt /media/sdc1/Software/Arduino.Sketch.Book/Ardufigo/Ardufigo.h

#the following line(s) is/are copied from the arduino ide
#these two compile the code:
#/home/knoppix/arduino-1.8.3/arduino-builder -dump-prefs -logger=machine -hardware /home/knoppix/arduino-1.8.3/hardware -tools /home/knoppix/arduino-1.8.3/tools-builder -tools /home/knoppix/arduino-1.8.3/hardware/tools/avr -built-in-libraries /home/knoppix/arduino-1.8.3/libraries -libraries /media/sdc1/Software/Arduino.Sketch.Book/libraries -fqbn=arduino:avr:mega:cpu=atmega2560 -vid-#pid=0X2341_0X0042 -ide-version=10803 -build-path /tmp/arduino_build_423278 -warnings=none -build-cache /tmp/arduino_cache_627979 -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avrdude.path=/home/knoppix/arduino-1.8.3/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=/home/knoppix/arduino-1.8.3/hardware/tools/avr -prefs=runtime.tools.avr-gcc.path=/home/knoppix/#arduino-1.8.3/hardware/tools/avr -verbose /media/sdc1/Software/Arduino.Sketch.Book/Ardufigo/Ardufigo.ino
#/home/knoppix/arduino-1.8.3/arduino-builder -compile -logger=machine -hardware /home/knoppix/arduino-1.8.3/hardware -tools /home/knoppix/arduino-1.8.3/tools-builder -tools /home/knoppix/arduino-1.8.3/hardware/tools/avr -built-in-libraries /home/knoppix/arduino-1.8.3/libraries -libraries /media/sdc1/Software/Arduino.Sketch.Book/libraries -fqbn=arduino:avr:mega:cpu=atmega2560 -vid-#pid=0X2341_0X0042 -ide-version=10803 -build-path /tmp/arduino_build_423278 -warnings=none -build-cache /tmp/arduino_cache_627979 -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avrdude.path=/home/knoppix/arduino-1.8.3/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=/home/knoppix/arduino-1.8.3/hardware/tools/avr -prefs=runtime.tools.avr-gcc.path=/home/knoppix/#arduino-1.8.3/hardware/tools/avr -verbose /media/sdc1/Software/Arduino.Sketch.Book/Ardufigo/Ardufigo.ino
#this one uploads the code to the board
#/home/knoppix/arduino-1.8.3/hardware/tools/avr/bin/avrdude -C/home/knoppix/arduino-1.8.3/hardware/tools/avr/etc/avrdude.conf -v -patmega2560 -cwiring -P/dev/ttyACM2 -b115200 -D -Uflash:w:/tmp/arduino_build_800381/halcyonZ80a.ino.hex:i 
#Beach Rats