\      __   __ _ _      ____                ___        __    _ _          __      __     
\      \ \ / _(_| | ___|  _ \ ___  __ _  __| \ \      / _ __(_| |_ ___   / _|___ / /     
\  _____\ | |_| | |/ _ | |_) / _ \/ _` |/ _` |\ \ /\ / | '__| | __/ _ \ | |_/ __/ /_____ 
\ |_____/ |  _| | |  __|  _ |  __| (_| | (_| | \ V  V /| |  | | ||  __/_|  _\__ \ |_____|
\      /_/|_| |_|_|\___|_| \_\___|\__,_|\__,_|  \_/\_/ |_|  |_|\__\___(_|_| |___/\_\     
\                                                                                        

\ program name: fileReadWrite.fs

\ This program uses two files. Initially one is an Intel HEX file and the other is blank. On completion the blank file is filled
\ with parsed hex bytes suitable for inclusion in an Arduino program for the construction of binaries stored in FLASH. The idea 
\ is that ultimately the FLASH image will be written into a shared SRAM that will be used by a host microprocessor.             
\ There is no error checking but from what I understand the file access directives all leave a number on the stack that is      
\ typically used by the word 'throw'. Instead of untangling all of that I just drop that byte and carry on.                     
\ below is illuminated generic INTEL HEX:                                                                                       
\ the first two characters, 0x10 in this case, are the number of bytes in the record (each line is called a record)             
\ The next four characters constitute the 64k location the record should occupy                                                 
\ the next two characters define the record type, in the case of the first four lines, data=0. In the case of the               
\ last record,1=EOF                                                                                                             
\ The next [number of bytes in the record]*2 are the actual data and the final two bytes are the checksum. The checksum         
\ is the 8-bit sum of each byte between the record length and the checksum itself which is then one's complement inverted.      
\                                                            
\     :10010000214601360121470136007EFE09D2190140            
\     :100110002146017E17C20001FF5F16002148011928            
\     :10012000194E79234623965778239EDA3F01B2CAA7            
\     :100130003F0156702B5E712B722B732146013421C7            
\     :00000001FF                                            
\                                                            
\ kjc 08aug2017                                              

variable sou        40 allot \ this is where the name and string length of the source file will be stored.
variable des        40 allot \ this is where the name and string length of the destination file will be stored.   
variable header		40 allot \ this is where "const byte body[16384]  PROGMEM = {" will be stored
variable footer 	10 allot \ this is where "};" will be stored
variable ptr \ and this is the pointer that will be used to write into 'source' and 'destination'.

: sou! ( addr len -) dup sou c! \ save the string length in the first byte of the array
	1 ptr ! \ initialize the pointer into the string array
	over + swap 
		do 
			i c@ 
			ptr @ sou + c!
			1 ptr +! 
		loop
;

: des! ( addr len -) dup des c! \ save the string length in the first byte of the array
	1 ptr ! \ initialize the pointer into the string array
	over + swap
		do
			i c@
			ptr @ des + c!
			1 ptr +!
		loop
;

: header! ( addr len -) dup header c! \ save the string length in the first byte of the array
	1 ptr ! \ initialize the pointer into the string array
	over + swap
		do
			i c@
			ptr @ header + c!
			1 ptr +!
		loop
;

: footer! ( addr len -) dup footer c! \ save the string length in the first byte of the array
	1 ptr ! \ initialize the pointer into the string array
	over + swap
		do
			i c@
			ptr @ footer + c!
			1 ptr +!
		loop
;

: sou@    ( - addr len) sou dup    1+ swap c@ ;
: des@    ( - addr len) des dup    1+ swap c@ ;
: header@ ( - addr len) header dup 1+ swap c@ ;
: footer@ ( - addr len) footer dup 1+ swap c@ ;

: sou. ( -) 
  sou @ 0= 0=
  if
  	sou@ over + swap do i c@ emit loop
  else
  	." empty string" 
  then 
;

: des. ( -) \ display the text file in the destination string variable
  des @ 0= 0= \ if it's not equal to '0'
  if
	des@ over + swap do i c@ emit loop
  else
  	." empty string"
  then
;

: header. ( -) \ display the text file in the destination string variable
  header @ 0= 0= \ if it's not equal to '0'
  if
	header@ over + swap do i c@ emit loop
  else
  	." empty string"
  then
;

\ clear the strings to all '0'
: 0sou    ( -) sou 40 0 fill ; 0sou
: 0des    ( -) des 40 0 fill ; 0des
: 0header ( -) header 40 0 fill ; 0header
: 0footer ( -) footer 10 0 fill ; 0footer

0 value input-file  \ the location of the input file handle
0 value output-file \ the location of the output file handle

variable buff 200 allot

: clr-buff ( -) buff 200 0 fill ; clr-buff

: ctrl? ( - f) dup bl < ; \ leaves a flag if the character was a control character ( less than ASCII space)

: char? ( c - n f) \ converts the character on the stack into a binary digit assuming a base of 16 
	48 - dup 0< 
		if 
			0 exit  
		then
	dup 9 >
		if 
			7 -
		then
;

\      __   ___                   _____ _ _           __        
\      \ \ / _ \ _ __   ___ _ __ |  ___(_| | ___ ___ / /        
\  _____\ | | | | '_ \ / _ | '_ \| |_  | | |/ _ / __/ /_____    
\ |_____/ | |_| | |_) |  __| | | |  _| | | |  __\__ \ |_____|   
\      /_/ \___/| .__/ \___|_| |_| |   |_|_|\___|___/\_\        
\               |_|              |_|                            


\ s" ~/.wine/drive_c/TASM/ARDUFIGO.OBJ"

: header-write ( - ) \ write the header to the file
  header @ 0= 0= \ if it's not equal to '0'
  if
	header@ over + swap do i c@ output-file emit-file drop loop
	13 output-file emit-file drop 
    10 output-file emit-file drop
  else
  	." empty string"
  then
;

: footer-write ( -) \ display the text file in the destination string variable
  footer @ 0= 0= \ if it's not equal to '0'
  if
	footer@ over + swap do i c@ output-file emit-file drop loop
	13 output-file emit-file drop 
    10 output-file emit-file drop
  else
  	." empty string"
  then
;

s" readFileTest.txt" sou!               \ put the source file name in a string variable preceeded by a byte count
sou@ r/o open-file drop to input-file   \ use the string var to open the file as read only and load the handle into 'input-file' value

s" writeFileTest.txt" des!              \ do the same for the destination file
des@ w/o open-file drop to output-file  \

s" const byte body[16384]  PROGMEM = {" header!

s" };" footer!

variable record-len
variable record-addr
variable record-type
variable record-buffer 256 allot

hex
: >var ( addr n -) \ put the next two characters in a variable
	0 swap 0 do 
		10 * input-file key-file char? + 
	loop
  !
;

: 4>var ( addr -) \ put a 4 digit number in the requested variable
	4 >var
;

: 2>var ( addr -) \ put a 2 digit number in the requested variable
	2 >var
;

: get-record-len ( -)
	0 
		2 0 do 
			10 * input-file key-file char? + 
		loop 
	record-len ! 
;

: get-record-addr ( -)
	0 
		4 0 do 
			10 * input-file key-file char? + 
		loop 
	record-addr ! 
;

: get-record-type ( -)
	0 
		2 0 do 
			10 * input-file key-file char? + 
		loop 
	record-type ! 
;

40 value ELF \ extra line feed for breaking up the file into 1k readable chunks
variable ELF-MATE

: do-elf
  ELF-MATE @ ELF mod 
  0= if 
  	cr 
   	0D output-file emit-file drop 
  	0A output-file emit-file drop
  then 
 ;

variable theByte \ two characters to be fused into a byte and then emitted at the end of a line

: byte-out ( -) \  writes a byte symbol of the form 0xNN to the file and echoes that to the terminal
	." 0x" 
	30 output-file emit-file drop \ 0x
	78 output-file emit-file drop 
	2 0 do 
	input-file key-file dup emit dup  theByte  I + C! 
	output-file emit-file drop 
	loop ." ," space 
	2C output-file emit-file drop \ ,
	20 output-file emit-file drop \ BL
	;

variable hexCharString 40 allot

: formatEmit
  dup dup 20 < swap 7E > or if drop 2E then
  output-file emit-file drop ;

: line->out ( -) 
	record-len @ 0 do 
		byte-out 
			0 
			2 0 do 
				10 * theByte I + C@ char? + 
			loop 
		I hexCharString + C!
	loop 
	2 0 do 2F output-file emit-file drop loop \ emit two slashes
	20 output-file emit-file drop \ a space
	record-len @ 0 do i hexCharString + C@ formatEmit loop
	0D emit 0A emit 
	0D output-file emit-file drop 
	0A output-file emit-file drop 
	1 ELF-MATE +!
	do-elf ;

: find-: ( -) 
	begin
		input-file key-file 3A ( : ) = 
	until
;

: get-all-record-data 
	get-record-len 
	get-record-addr 
	get-record-type 
;

: align-record 
	find-: 
	get-all-record-data 
;

: output-all
	header-write
	0 ELF-MATE !
	begin
		align-record record-type @ 0= 
	while
		line->out
	repeat
	footer-write
    output-file flush-file drop
;

output-all \ perform the top-level operation

bye \ and exit the program on completion

decimal