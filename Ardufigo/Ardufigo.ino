/*  _            _        __ _              _             
   / \   _ __ __| |_   _ / _(_) __ _  ___  (_)_ __   ___  
  / _ \ | '__/ _` | | | | |_| |/ _` |/ _ \ | | '_ \ / _ \ 
 / ___ \| | | (_| | |_| |  _| | (_| | (_) _| | | | | (_) |
/_/   \_|_|  \__,_|\__,_|_| |_|\__, |\___(_|_|_| |_|\___/ 
                               |___/                      

the ATMega 256 has the following system memory
–   256KBytes of In-System Self-Programmable Flash 
–   4Kbytes EEPROM
–   8Kbytes Internal SRAM

*/ 

#include <avr/pgmspace.h> // this will allow storing the virtual machine's program in FLASH
#include <EEPROM.h> // 4096 system variables (VALUES) or can be stored here in the form of screens if desired
#include <math.h> // used for the transcendental arithmetic functions 
//#include <SPI.h> // many devices, SPI included, use this library
#include <SD.h> // used for mass storage
#include "ArdufigoROM.h" // the huge table containing the forth program
#include "ArdufigoRAM.h" // the huge SRAM table

// This is where the 'hard coded' part of the VM is stored. The user part of the memory space is in SRAM 
// in the 'pad' space and is specified by addresses who's msbs are set ~ 0x8000 and above in the unsigned world
// (i.e. <0)

extern const PROGMEM byte body[0x4000]; // The ArdufigoROM.h file must be examined to appreciate this. It's the entire ROM for the OS.

extern byte pad[5*1024]; // this is where all of the SRAM is located

//int padPointer = 0; // the pointer into that pad

// uncomment this line for debugging code 
//#define DEVMODE 1

#define txRxSwitch 42 // this will change the direction of the half-duplex port.

// The heartbeat led business
#define ledPin 13
int ledPinState = 0;
long ledMil = millis();
long ledMilDur = 250; // set the duration for 1/4 of a second

int sPortSel = 2; // this selects the serial port used by test routines

// VM variables
int IP = 0; // the instruction pointer

int WA = 0; // the word address register

int CA = 0; // the code address register
int CAh = 0;
int CAl = 0;

int PA = 0;
int PAh = 0;
int PAl = 0;

#define PSTACKLEN 32
#define PSTACKZED 31
int stack[PSTACKLEN]; // this is a parameter stack
int stackPointer = PSTACKLEN - 1; // the pointer into that stack

#define RSTACKLEN 16
#define RSTACKZED 15
int stackR[RSTACKLEN]; // this is the return stack
int stackPointerR = RSTACKLEN - 1; // this is the return stack pointer

#define LSTACKLEN 16
#define LSTACKZED 15
int stackL[LSTACKLEN]; // this is the loop stack
int stackPointerL = LSTACKLEN - 1;

#define FSTACKLEN 16
#define FSTACKZED 15
float stackF[FSTACKLEN]; // this is the floating point stack
int stackPointerF = FSTACKLEN - 1;

#define UPINIT 0x93C0 // the user variables base address
#define EPINIT 0xCF00 // the EEPROM based user variables base address

// the serial port mapppings
#define PORTPGM       0x0000
#define DYNAMIXEL     0x0001
#define PORTDEBUG     0x0002
#define PORTMAIN      0x0003

#define SDCS              53 // The CS_ pin for the SD card

// assuming the most relevant parts of the stack are the top three entries I am using the following mnemonics:
// TOS is the Top Of Stack
// NOS is the Next On Stack
// BOS is the Bottom Of Stack
#define TOS          stack[stackPointer]
#define NOS          stack[stackPointer+1]
#define BOS          stack[stackPointer+2]

File  theFile; // this is the 'hardwired' file that will have to suffice pending implementation of string variables
char fileName[13];

//String fStr((char*)fileName);
File root;

// 16-bit
int  accuA  = 0; // an accumulator
int  accuB  = 0; // another accumulator
int  accuC  = 0; // yet another accumulator
int  accuD  = 0; // still yet another accumulator
   
// 32-bit   
long accuAx = 0; // for double number manipulations
long accuBx = 0; //  '    '       '        '

// 32-bit
float accuAf = 0.0; // floating point accumulators etc...
float accuBf = 0.0; //
float accuCf = 0.0; //
float accuDf = 0.0; //
static char foutstr[15]; // for displaying floats

// for type conversion
union { // for accessing the integers(16) within a long(32)
   unsigned long uarpro;
   long larpro;
   int iarpro[2];
   byte barpro[4];
} parpro;

union { // the cross-type resource translation depot
   float fivot;
   unsigned long uivot;
   long  livot;
   int   iivot[2];
   byte  bivot[4];
} pivot; 

 //////////////////////////////////////////////////////////////////////// 
//////////////////////////////////////////////////////////////////////////
/////               ____       _                ___                  /////
////               / ___|  ___| |_ _   _ _ __  / \ \                  ////
////               \___ \ / _ | __| | | | '_ \| | | |                 ////
////                ___) |  __| |_| |_| | |_) | | | |                 ////
////               |____/ \___|\__|\__,_| .__/| | | |                 ////
////                                    |_|    \_/_/                  ////
/////                                                                /////
//////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////// 

void setup() {
   pinMode(SDCS, OUTPUT);
   digitalWrite(SDCS, HIGH);
	pinMode(txRxSwitch, OUTPUT);
	pinMode(ledPin, OUTPUT);
	digitalWrite(txRxSwitch, HIGH); // set the half-duplex line to output

	Serial.begin(115200); // onboard/programming
   Serial1.begin(1000000); // DYNAMIXEL Servos
   Serial2.begin(115200); // debug
   Serial3.begin(115200); // main
#if defined(DEVMODE)
   Serial2.print("\033[48;5;16m"); // B black
   Serial2.print("\033[38;5;48m"); // T aqua 
   Serial2.println("                                Debugging services available.");
   Serial2.print("\033[38;5;231m"); // T white 
   Serial2.print("\033[38;5;0m"); // T black            |
   Serial2.write(0x2E); // print an identifying "."     | Print an invisible dot
   Serial2.print("\033[38;5;231m"); // T white          | 
   innerTest(); // check the status of the inner interpreter pointers
#endif

   // eventually the optional argument, CSpin, 
   // will be included so the SPI port can be used for other applications sumultaneously

   loadIP(0); // load the instruction pointer from the first word

   // erase the SRAM
	//int i;
	//for (i=0; i<(1024*5); i++) {
	//	pad[i] = 0;
	//}

	//IP = 0; // set the instruction pointer to the beginning of the prog
}
 //////////////////////////////////////////////////////////////////////// 
//////////////////////////////////////////////////////////////////////////
/////                 _                       ___                    /////
////                 | |    ___   ___  _ __  / \ \                    ////
////                 | |   / _ \ / _ \| '_ \| | | |                   ////
////                 | |__| (_) | (_) | |_) | | | |                   ////
////                 |_____\___/ \___/| .__/| | | |                   ////
////                                  |_|    \_/_/                    ////
/////                                                                /////
//////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////// 

void loop() { // very simple when it works...
	next();
}
 ////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////        _____                 _   _                  ___        /////
////        |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ / \ \        ////
////        | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| | | |       ////
////        |  _|| |_| | | | | (__| |_| | (_) | | | \__ | | | |       ////
////        |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___| | | |       ////
////                                                     \_/_/        ////
/////                                                                /////
//////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////
// load the instruction pointer from addr
void loadIP(int addr) {
   pivot.bivot[1] = pgm_read_byte(&(body[addr])); // hi byte
   addr++;
   pivot.bivot[0] = pgm_read_byte(&(body[addr])); // lo byte
   IP = pivot.iivot[0];
}

// Parameter Stack Ops ~ post decrement push / pre increment pop
void pushp(int val) {
	stack[stackPointer] = val;
	stackPointer--;
	stackPointer &= 63;
}

int popp() {
	int val;
	stackPointer++;
	stackPointer &= 63;
	val = stack[stackPointer];
	return val;
}

void pushA() { pushp(accuA); }// push accumulator 'A' to the parameter stack
void popA() { accuA = popp(); } // pop the stack top into accumulator 'A'

void pushB() { pushp(accuB); } // push accumulator 'A' to the parameter stack
void popB() { accuB = popp(); } // pop the stack top into accumulator 'A'
	
void pushC() { pushp(accuC); } // push accumulator 'A' to the parameter stack
void popC() { accuC = popp(); } // pop the stack top into accumulator 'A'U*
	
void pushD() { pushp(accuD); } // push accumulator 'A' to the parameter stack
void popD() { accuD = popp(); } // pop the stack top into accumulator 'A'

void pushpx(long val) {
	int valo; int valh;	
	valo = ((int)(val & 0x0000FFFF));
	val = val >> 16;
	valh = (int)val;
	stack[stackPointer] = valo;
	stackPointer--;
	stackPointer &= 63;
	stack[stackPointer] = valh;
	stackPointer--;
	stackPointer &= 63;
}

long poppx() {
	long val;
	int valo; int valh;
	stackPointer++;
	stackPointer &= 63;
	valh = stack[stackPointer];
	stackPointer++;
	stackPointer &= 63;
	valo = stack[stackPointer];
	val = (long)valh;
	val = val << 16;
	val |= (long)valo;
	return val;
}

void pushAx() { pushpx(accuAx); } // push accumulator 'A' to the parameter stack
void popAx() { accuAx = poppx(); } // pop the stack top into accumulator 'A'

void pushBx() { pushpx(accuBx); } // push accumulator 'A' to the parameter stack
void popBx() { accuBx = poppx(); } // pop the stack top into accumulator 'A'

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////  _____ _             _   _             ____       _       _    /////
////  |  ___| | ___   __ _| |_(_)_ __   __ _|  _ \ ___ (_)_ __ | |_   ////
////  | |_  | |/ _ \ / _` | __| | '_ \ / _` | |_) / _ \| | '_ \| __|  ////
////  |  _| | | (_) | (_| | |_| | | | | (_| |  __| (_) | | | | | |_   ////
////  |_|   |_|\___/ \__,_|\__|_|_| |_|\__, |_|   \___/|_|_| |_|\__|  ////
/////                                  |___/                         /////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// floating point stack ops:  ~ post decrement push / pre increment pop
void pushf(float val) {
   stackF[stackPointerF] = val;
   stackPointerF--;
   stackPointerF &= FSTACKZED;
}

float popf() {
   float val;
   stackPointerF++;
   stackPointerF &= FSTACKZED;
   val = stackF[stackPointerF];
   return val;
}

void pushAf() { pushf(accuAf); }
void popAf() { accuAf = popf(); }

void pushBf() { pushf(accuBf); }
void popBf() { accuBf = popf(); }

void pushCf() { pushf(accuCf); }
void popCf() { accuCf = popf(); }

void pushDf() { pushf(accuDf); }
void popDf() { accuDf = popf(); }

// fstack ops
void pupi() {
   accuAf = 3.14159265;
   pushAf();
}

void dtof() { // (p d - ) ; (f - f )
   popAx();
   accuAf = (float)accuAx;
   pushAf();
}

void ftod() {
   popAf();
   accuAx = (long)accuAf;
   pushAx();
}

void stof() { // (p d - ) ; (f - f )
   popA();
   accuAf = (float)accuA;
   pushAf();
}

void ftos() { // F>S
   popAf();
   accuA = (int)accuAf;
   pushA();
}

void fswap() {
   popAf();  popBf();
   pushAf(); pushBf();
}

void fover() {
   popAf();
   popBf();
   pushBf();
   pushAf();
   pushBf();
}

void fdrop() {
   stackPointerF++;
   stackPointerF &= FSTACKZED;
}

void fdup() {
   popAf();
   pushAf(); pushAf();
}

void frot() { // (f f1, f2, f3 - f2, f3, f1 )
   popAf();
   popBf();
   popCf();
   pushBf();
   pushAf();
   pushCf();
}

void fslas() {
   popDf(); // divisor
   popCf(); // dividend
   accuAf = accuCf / accuDf;
   pushAf();
}

void fstar() {
   popDf(); // multiplier
   popCf(); // multiplicand
   accuAf = accuCf * accuDf;
   pushAf();
}

void fplus() {
   popDf(); // addend
   popCf(); // addend
   accuAf = accuCf + accuDf;
   pushAf();
}

void fsubb() {
   popDf(); // minuend
   popCf(); // subtrahend
   accuAf = accuCf - accuDf;
   pushAf();
}

void fsin() {
   popAf();
   accuBf = sin(accuAf);
   pushBf();
}

void fcos() {
   popAf();
   accuBf = cos(accuAf);
   pushBf();
}

void ftan() {
   popAf();
   accuBf = tan(accuAf);
   pushBf();
}

void fasin() {
   popAf();
   accuBf = asin(accuAf);
   pushBf();
}

void facos() {
   popAf();
   accuBf = acos(accuAf);
   pushBf();
}

void fatan() {
   popAf();
   accuBf = atan(accuAf);
   pushBf();
}

void fatan2() {
   popAf(); // y val
   popBf(); // x val
   accuBf = atan2(accuAf, accuBf);
   pushBf();
}

void fsqrt() {
   popAf();
   accuBf = sqrt(accuAf);
   pushBf();
}

void fpow() { // (f base, exp - f )
   popAf(); //  exp
   popBf(); //  base
   accuCf = pow(accuBf, accuAf);
   pushCf();
}

void fexp() {
   popAf();
   accuBf = exp(accuAf);
   pushBf();
}

void fstore() { // (f f - ) / (p a - ) ~ fp number is on the float stack; address is on the parameter stack
   popA(); // get the address
   popAf(); // get the floating point #
   pivot.fivot = accuAf;
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) {
      case 0xC000: // eeprom
         EEPROM.update(accuA,   pivot.bivot[3]); // EEPROM.update(addr, val);
         EEPROM.update(accuA+1, pivot.bivot[2]); // EEPROM.update(addr, val);
         EEPROM.update(accuA+2, pivot.bivot[1]); // EEPROM.update(addr, val);
         EEPROM.update(accuA+3, pivot.bivot[0]); // EEPROM.update(addr, val);
      break;
      case 0x8000:
         pad[accuA] =   pivot.bivot[3]; // and save the lo-byte
         pad[accuA+1] = pivot.bivot[2]; // and save the lo-byte
         pad[accuA+2] = pivot.bivot[1]; // and save the lo-byte
         pad[accuA+3] = pivot.bivot[0]; // and save the lo-byte
      break;
      default:
         Serial.println("F! address range error.");
         //goto lfsr;
      break;
   }
//lfsr:
   noop();
}
/*

*/
void fat() { // (f - f ) / (p a - ) ~ initially there is an address on the parameter stack. This is consumed and a fp val remains on the fp stack
   popA(); // get the address
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) {
      case 0xC000:
         pivot.bivot[3] = EEPROM.read(accuA); // EEPROM
         pivot.bivot[2] = EEPROM.read(accuA+1); // EEPROM
         pivot.bivot[1] = EEPROM.read(accuA+2); // EEPROM
         pivot.bivot[0] = EEPROM.read(accuA+3); // EEPROM
      break;
      case 0x8000:
         pivot.bivot[3] = pad[accuA]; // SRAM
         pivot.bivot[2] = pad[accuA+1]; // SRAM
         pivot.bivot[1] = pad[accuA+2]; // SRAM
         pivot.bivot[0] = pad[accuA+3]; // SRAM
      break;
      default:
         pivot.bivot[3] = pgm_read_byte(&(body[accuA])); // FLASH
         pivot.bivot[2] = pgm_read_byte(&(body[accuA+1])); // FLASH
         pivot.bivot[1] = pgm_read_byte(&(body[accuA+2])); // FLASH
         pivot.bivot[0] = pgm_read_byte(&(body[accuA+3])); // FLASH
      break;
   } 
   accuAf = pivot.fivot;
   pushAf(); 
}

void fpstor() { // F+! (p addr - ) (f fincrement - )
   dup();
   fat();
   fplus();
   fstore();
}

// destructive pictured output from the top of the floating point stack 
void fdot() {
   popAf();
   dtostrf(accuAf, 10, 7, foutstr);
   PAh = UPINIT + 0x03B; // the location of the user variable OUT (0x93C0 + 0x01A)
   PAh &= 0x3FFF; // place within the actual addressable range
   pivot.bivot[0] = pad[PAh]; // get the number stored in COM#
   pivot.bivot[0] &= 3; // range reduce to 1-of-4 selection (may change later to include more ports)
   switch(pivot.bivot[0]) {
      case 0:
         Serial.print(foutstr);
      break;
      case 1:
         Serial1.print(foutstr);
      break;
      case 2:
         Serial2.print(foutstr);
      break;
      case 3:
         Serial3.print(foutstr);
      break;
   }
}

void fcells() {
   stack[stackPointer+1] = stack[stackPointer+1] << 2;
}

void flt() { // F<
   fsubb();    // F-
   ftos();     // F>S
   zlt();      // 0<
}

//   __  __       _        _      ____             _   _                
//  |  \/  | __ _| |_ _ __(___  _|  _ \ ___  _   _| |_(_)_ __   ___ ___ 
//  | |\/| |/ _` | __| '__| \ \/ | |_) / _ \| | | | __| | '_ \ / _ / __|
//  | |  | | (_| | |_| |  | |>  <|  _ | (_) | |_| | |_| | | | |  __\__ \
//  |_|  |_|\__,_|\__|_|  |_/_/\_|_| \_\___/ \__,_|\__|_|_| |_|\___|___/
//

void fourmod() {
   stack[stackPointer+1] &= 0x03;
}

void sxtmod() {
   stack[stackPointer+1] &= 0x0F;
}

void fourslas() {
   stack[stackPointer+1] = stack[stackPointer+1] >> 2;
}

void sxtslas() {
   stack[stackPointer+1] = stack[stackPointer+1] >> 4;
}

////////////////////////////////////////////////////////////////////////
// Return Stack Ops
void tor() { // >R  ~ post decrement push / pre increment pop
	popA();
	stackR[stackPointerR] = accuA;
	stackPointerR--;
}

void rfrom() { // R>
	stackPointerR++;
	accuA = stackR[stackPointerR];
	pushA();
}

void rfetch() { // R or R@ in some models
	accuA = stackR[stackPointerR + 1];
	pushA();
}

// Loop Stack Ops
void tol() { // >L  ~ post decrement push / pre increment pop
   popA();
   stackL[stackPointerL] = accuA;
   stackPointerL--; stackPointerL &= LSTACKZED;
}

void lfrom() { // L>
   stackPointerL++; stackPointerL &= LSTACKZED;
   accuA = stackL[stackPointerL];
   pushA();
}

void lfetch() {
   accuA = stackL[stackPointerL + 1];
   pushA();
}

void voodoop() { // (DO) ;the virtual machine 'DO' push two words to the loop stack ~ post decrement push / pre increment pop
	popB(); // get the index
	popA(); // and the limit
	stackL[stackPointerL] = accuA;
	stackPointerL--;
	stackL[stackPointerL] = accuB;
	stackPointerL--;
}

void leave() { // LEAVE ( makes the limit the same as the index )
   stackL[stackPointerL + 1] = stackL[stackPointerL + 2];  
}

void paloop() { // (LOOP)
   accuD = 1;
   comloop();
}

void paploop() { // (+LOOP)
   popD(); // get the increment
   comloop();
}

void comloop() { // code common to both (LOOP) and (+LOOP)
   stackL[stackPointerL + 1] += accuD; // add the increment to the index
   accuA = stackL[stackPointerL + 1]; // index
   accuB = stackL[stackPointerL + 2]; // limit
   if (accuD < 0) { // check
      accuA = accuB - accuA;
   }
   else {
      accuA = accuA - accuB;
   } 
   if (accuA < 0) { // make the loop
      if (IP < 0) { // take loop address from SRAM 
         IP &= 0x7FFF;
         pivot.bivot[1] = pad[IP];
         IP++;
         pivot.bivot[0] = pad[IP];         
      }
      else { // take loop address from FLASH 
         pivot.bivot[1] = pgm_read_byte(&(body[IP]));
         IP++;
         pivot.bivot[0] = pgm_read_byte(&(body[IP]));
      }
      IP = pivot.iivot[0];
   }
   else { // exit the loop
      stackPointerL += 2; // remove the index and limit
      IP += 2; // and resume execution past the branch
   }
}
      


void eye() { // I (the outer loop counter)
	accuA = stackL[stackPointerL + 1];
   pushA();
}
void eyep() {
   accuA = stackL[stackPointerL + 2];
   pushA();
}
void jay() { // J (next inner loop counter)
	accuA = stackL[stackPointerL + 3];
   pushA();
}
void jayp() {
   accuA = stackL[stackPointerL + 4];
   pushA();
}
void kay() { // K (next inner loop counter)
	accuA = stackL[stackPointerL + 5];
   pushA();
}
void kayp() {
   accuA = stackL[stackPointerL + 6];
   pushA();
}

void dup() { // DUP
	accuA = stack[stackPointer+1];
   stack[stackPointer] = accuA;
   stackPointer--;
}


void swap() { // SWAP
   accuB = stack[stackPointer+2];
   accuA = stack[stackPointer+1];
   stack[stackPointer+2] = accuA;
   stack[stackPointer+1] = accuB;
}

void over() { // OVER
	accuB = stack[stackPointer+2];
	stack[stackPointer] = accuB;
   stackPointer--;
}

void tdup() {
	popAx();
	pushAx(); pushAx();
}

void tswap() { // ( da, db, - db, da )
	popAx();  popBx();
	pushAx(); pushBx();
}

void tover() { // ( da, db, - da, db, da )
	popAx(); popBx();
	pushBx(); pushAx(); pushBx();
}

void add() { // +
	popA();
	stack[stackPointer+1] += accuA;
}

void sub() { // -
	popA();
	stack[stackPointer+1] -= accuA;
}

void dadd() { // D+ 
	popA(); popB();
   popC(); popD();
   parpro.iarpro[1] = accuA;
   parpro.iarpro[0] = accuB;
   pivot.iivot[1] = accuC;
   pivot.iivot[0] = accuD; 
	pivot.uivot += parpro.uarpro;
   accuC = pivot.iivot[1];
   accuD = pivot.iivot[0];
 	pushD();
   pushC();
}
/*
void dadd() // D+ {
   popAx();
   popBx();
   accuAx += accuBx;
   pushAx();
}

*/
// A map of one 16-bit x 16-bit = 32-bit operation in terms of four 8-bit x 8-bit = 16-bit operations
//                 +-------+-------+
//                 |   A   |   B   | ~ 16-bit multiplier
//                 +-------+-------+
//                 +-------+-------+
//             *   |   C   |   D   | ~ 16-bit multiplicand
//                 +-------+-------+
// +-------+-------+-------+-------+ 
// |-------|-------|     D * B     | [1]
// +-------+-------+-------+-------+
// |-------|     D * A     |-------| [2]
// +-------+-------+-------+-------+    ~ partial products
// |-------|     C * B     |-------| [3] 
// +-------+-------+-------+-------+
// |     C * A     |-------|-------| [4]
// +-------+-------+-------+-------+
// +-------+-------+-------+-------+
// |      AB       *       CD      | ~ 32-bit product
// +-------+-------+-------+-------+
// The next two routines are rather heavy-handed as they use a lot of bit and byte banging.
// The U* routine consists of four 8-bit partial products that are shifted and summed to 
// form the result. The U/ routine uses the most brutal of the two methods. It is a shift
// and subtract algorithm that is so old you can see the dust between its bits.
// The sad truth is that I could not find any GCC routines that would perform these simple
// operations properly. 
/*
void nul() { // alternative to mul (U*)
   popA(); // multiplier (CD)
   popB(); // multiplicand (AB)
   accuAx = 0;
   pivot.iivot[0] = accuA; // split everything into bytes
   accuAb = pivot.bivot[1];
   accuBb = pivot.bivot[0];
   pivot.iivot[0] = accuB;
   accuCb = pivot.bivot[1];
   accuDb = pivot.bivot[0];
   parpro.larpro = 0; // partial product #1
   pivot.iivot[0] = accuDb * accuBb;
   parpro.iarpro[0] = pivot.iivot[0];
   accuAx += parpro.larpro;
   parpro.larpro = 0; // partial product # 2
   pivot.iivot[0] = accuDb * accuAb;
   parpro.barpro[1] = pivot.bivot[0];
   parpro.barpro[2] = pivot.bivot[1];
   accuAx += parpro.larpro;
   parpro.larpro = 0; // partial product # 3
   pivot.iivot[0] = accuCb * accuBb;
   parpro.barpro[1] = pivot.bivot[0];
   parpro.barpro[2] = pivot.bivot[1];
   accuAx += parpro.larpro;
   parpro.larpro = 0; // partial product # 4
   pivot.iivot[0] = accuCb * accuAb;
   parpro.iarpro[1] = pivot.iivot[0];
   accuAx += parpro.larpro;
   parpro.larpro = accuAx; // place the final product on the stack
   accuA = parpro.iarpro[0];
   accuB = parpro.iarpro[1];
   pushA();
   pushB();
} */
void mul() { // U* 
	long temp = 0;
	long semp = 0;
	popA(); // multiplier (CD)
	popB(); // multiplicand (AB)
   accuD = accuA & 0x0FF; // D ~ work registers
   accuC = accuA >> 8;    // C
   accuA = accuB >> 8;    // A
   accuB = accuB & 0x0FF; // B
   PAl = accuD * accuB;   // DB ~ partial products (PP)
   PAh = accuD * accuA;   // DA
   CAl = accuC * accuB;   // CB
   CAh = accuC * accuA;   // CA
	temp = (long)PAl;     // cast PPs as longs and sum
	semp = (long)PAh;
   semp = semp << 8;
   semp += temp;
   temp = (long)CAl;
   temp = temp << 8;
   semp += temp;
   temp = (long)CAh;
   temp = temp << 16;
   temp += semp; // all partial products are summed
   semp = temp & 0x0FFFF;
   accuB = (int)semp; // low
   temp = temp >> 16;
	accuA = (int)temp; // high
	pushB();
	pushA();
}

void div() { // U/
	popA(); // (bc) divisor
   popB(); // (hl) dividend hi / remainder
   popC(); // (de) dividend lo / quotient
   if ((accuB) >= (accuA)) { // divisor is greater than dividend exit with -1
      accuB = -1; accuC = -1;
   }
   else { // division is possible: go for it
      int i;
      for (i = 0; i < 16; i++) {
         accuD = accuC & 0x8000; // shift the dividend; accuD=CY
         accuC = accuC << 1;
         accuB = accuB << 1;
         if (accuD != 0) {
            accuB++; // add in the carry
         }
         accuC++; // assume the subtraction was possible
         if (accuB < accuA) { // if subtraction is impossible
            accuC--; // clear the lsb
         }
         else {
            accuB -= accuA;
         }
      }
   }
   pushB(); // remainder
   pushC(); // quotient   
}

void rot() { // ROT
   accuA = stack[stackPointer+1];
   accuB = stack[stackPointer+2];
   accuC = stack[stackPointer+3];
   stack[stackPointer+1] = accuC;
   stack[stackPointer+2] = accuA;
   stack[stackPointer+3] = accuB;
}

void por() {
	popA();
	stack[stackPointer+1] |= accuA; 
}

void pand() {
	popA();
	stack[stackPointer+1] &= accuA; 
}

void pxor() {
	popA();
	popB();
	accuA ^= accuB;
	pushA();
}

int genPop(int pointer, int dest) { // use any integer as a stack pointer and any integer as a destination
	int temp;
	if (pointer < 0) { // the action is in SRAM
		temp = pad[pointer];
		pointer++;
		dest = pad[pointer];
	}
	else { // the action is in FLASH
		temp = pgm_read_byte(&(body[pointer]));
		pointer++;
		dest = pgm_read_byte(&(body[pointer]));
	}
	pointer++;
	temp = temp << 8; dest |= temp;
	return dest;
}

void fetch() { // @ ~ uses accuC and accuB and accuA
	accuA = stack[stackPointer+1];
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) {
      case 0xC000:
         pivot.bivot[1] = EEPROM.read(accuA); // hi
         pivot.bivot[0] = EEPROM.read(accuA + 1); // lo
      break;
      case 0x8000:
         pivot.bivot[1] = pad[accuA]; // hi
         pivot.bivot[0] = pad[accuA + 1]; // lo
      break;
      default:
         pivot.bivot[1] = pgm_read_byte(&(body[accuA])); // hi
         pivot.bivot[0] = pgm_read_byte(&(body[accuA + 1])); // lo
      break;
   }
   //accuD = pivot.iivot[0];
   stack[stackPointer+1] = pivot.iivot[0];
}



void store() { // ! ~ works for SRAM & EEPROM spaces
	popA(); // get the address
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   popD(); // lo
   accuC = accuD >> 8; // hi
   switch(accuB) {
      case 0xC000:
         EEPROM.update(accuA, accuC);
         EEPROM.update((accuA + 1), accuD);
      break;
      case 0x8000:
         pad[accuA] = accuC; // save hi
         pad[accuA + 1] = accuD; // and save lo
      break;
      default:
         Serial.println("! address range error.");
      break;
   }
}

void dfetch() { // @ ~ uses accuC and accuB and accuA
	popA(); // get the address
   accuD = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuD) {
      case 0xC000: // EEPROM
         for (accuC = 3; accuC >= 0; accuC--) {
            parpro.barpro[3-accuC] = EEPROM.read(accuC);
         }
      break;
      case 0x8000: // SRAM
         for (accuC = 3; accuC >= 0; accuC--) {
            parpro.barpro[3-accuC] = pad[accuC];
         }
      break;
      default: // everything else
         parpro.barpro[3-accuC] = pgm_read_byte(&(body[accuC]));
      break;
   }
   accuA = parpro.iarpro[1];
   accuB = parpro.iarpro[0];
   pushB();
   pushA();
}

void dstore() { // 2! 
   	popA(); // get the address
   	popB(); // get the hi-word
   	popC(); // the the lo-word
      parpro.iarpro[1] = accuB;
      parpro.iarpro[0] = accuC;
      accuD = accuA & 0xC000; // the switch variable
      accuA &= 0x3FFF; // 
      switch(accuD) {
         case 0xC000: // EEPROM
            for (accuC = 0; accuC < 4; accuC++) {
               EEPROM.update((accuA+accuC), parpro.barpro[3-accuC]);
            }
         break;
         case 0x8000: // SRAM
            for (accuC = 0; accuC < 4; accuC++) {
               pad[accuA+accuC] = parpro.barpro[3-accuC];
            }
         break;
         default: // everything else (FLASH)
            Serial.println("2! address range error.");
         break;
      }
}

void cfetch() { // C@ ~ handles all of the data spaces
	accuA = stack[stackPointer+1];
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) {
      case 0xC000: // EEPROM
         accuD = EEPROM.read(accuA); // EEPROM
      break;
      case 0x8000: // SRAM
         accuD = pad[accuA]; // SRAM
      break;
      default: // every address lower than 8000.
         accuD = pgm_read_byte(&(body[accuA])); // FLASH
      break;
   }
	stack[stackPointer+1] = accuD;
}

void cstore() { // C! ~ handles all of the data spaces
   accuA = stack[stackPointer+1]; // get the address
   accuD = stack[stackPointer+2]; // get the data
   stackPointer+=2;
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) {
      case 0xC000:
         EEPROM.update(accuA, accuD); // EEPROM.update(addr, val);
      break;
      case 0x8000:
         pad[accuA] = accuD; // and save the lo-byte
      break;
      default:
         Serial.println("C! can't write to FLASH.");
      break;
   }
}
   

void plusstore() { // +! ~ ( inc, addr - )
   popA(); // the address
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   popD(); // the increment
   switch(accuB) {
      case 0xC000: // EEPROM
         pivot.bivot[1] = EEPROM.read(accuA);
         pivot.bivot[0] = EEPROM.read(accuA + 1);
         pivot.iivot[0] += accuD;
         EEPROM.update(accuA, pivot.bivot[1]);
         EEPROM.update((accuA + 1), pivot.bivot[0]);
      break;
      case 0x8000: // SRAM
         pivot.bivot[1] = pad[accuA];
         pivot.bivot[0] = pad[accuA + 1];
         pivot.iivot[0] += accuD;
         pad[accuA] = pivot.bivot[1];
         pad[accuA + 1] = pivot.bivot[0];
      break;
      default: // everything else
         Serial3.println("+! can't write to FLASH.");
         pushB();
         pushA();
      break;
   }
}

void zeq() { // 0= ;zero equals
	accuA = 0;
	popD();
	if (accuD == 0) {
		accuA--;
	}
	pushA();
}

void zero() { // 0
	accuA = 0;
	pushA();
}

void one() { // 1
	accuA = 1;
	pushA();
}

void two() { // 2
	accuA = 2;
	pushA();
}

void mone() { // -1
	accuA = 0xFFFF;
	pushA();
}

void mtwo() { // -2
	accuA = 0xFFFE;
	pushA();
}

void equals() { // =
	accuB = 0;
	popA();
	popD();
	accuD -= accuA;
	if (accuD == 0) {
		accuB--;
	}
	pushB();
}

void drop() {
	stackPointer++;
}

void twodrop() {
   stackPointer += 2;
}

void setBaud0() {
   Serial.end(); // shut down to initial state
   Serial.begin(fileName); // use the string previously loaded into fileName
}

void setBaud1() {
   Serial1.end(); // shut down to initial state
   Serial1.begin(fileName); // use the string previously loaded into fileName
}

void setBaud2() {
   Serial2.end(); // shut down to initial state
   Serial2.begin(fileName); // use the string previously loaded into fileName
}

void setBaud3() {
   Serial3.end(); // shut down to initial state
   Serial3.begin(fileName); // use the string previously loaded into fileName
}

void qterm3() { // ?TERMINAL ( - f)
   accuA = -1; // assume there are characters available
   accuB = Serial3.available();
   if (accuB == 0) {
      accuA++;
   }
   pushA();
}

void key3() { // (KEY3)
	while (Serial3.available() == 0) {
		tasks(); // any routine that may possibly have to wait for a condition must do tasks()!
	}
	accuA = Serial3.read();
	pushA();
}

void key2() { // (KEY2)
   accuB = Serial2.available();
   if (Serial2.available() != 0) {
      accuA = Serial2.read();
      pushA();
   }
   pushB();
}

void key1() { // (KEY1) the DYNAMIXEL port
   accuB = Serial1.available();
   if (Serial1.available() != 0) {
      accuA = Serial1.read();
      pushA();
   }
   pushB();
}

void key0() { // (KEY0)
   accuB = Serial.available();
   if (Serial.available() != 0) {
      accuA = Serial.read();
      pushA();
   }
   pushB();
}

void emit3() { // (emit3)
	popA();
	Serial3.write(accuA);
}

void emit2() { // (emit2)
   popA();
   Serial2.write(accuA);
}

void emit1() { // (emit1)
   popA();
   Serial1.write(accuA);
}

void emit0() { // (emit0)
   popA();
   Serial.write(accuA);
}

void noop() {
}

void bran() { // BRANCH
	if (IP < 0) { // the data is in SRAM; mask a proxy pointer
		parpro.iarpro[0] = 0x7FFF & IP;
		pivot.bivot[1] = pad[parpro.iarpro[0]];
		parpro.iarpro[0]++; 
		pivot.bivot[0] = pad[parpro.iarpro[0]];
	}
	else { // the data is in FLASH; use the IP as pointer
		pivot.bivot[1] = pgm_read_byte(&(body[IP]));
		IP++;
		pivot.bivot[0] = pgm_read_byte(&(body[IP]));
	}
	IP = pivot.iivot[0];	
}

void zbran() { // 0BRANCH
	accuD = stack[stackPointer+1];
   stackPointer++;
	if (accuD == 0) { // take the branch if TOS == 0
		bran();
	}
	else { // skip over the branch word
		IP += 2;
	}
}

// : OF     4 ?PAIRS  COMPILE OVER   COMPILE =   COMPILE 0BRANCH
//          HERE 0 ,   COMPILE DROP  5  ;   IMMEDIATE
//
// figure out how to incorporate the literal to be compared to 
// into the (OF) word.

void pof() { // used to speed up the Eaker case statement
   accuD = 0;
   accuB = stack[stackPointer+2];   // over
   popA();
   accuA -= accuB;                  // =
   if (accuA == 0) {
      accuD--;
   }
   pushD();
   zbran();                         // 0branch
   if (accuD != 0) {
      stackPointer++;               // drop
   }
}
// 0 !=0 --     
void clit() {
   accuB = IP & 0xC000; // generate a case vector
   accuA = IP & 0x3FFF; // generate the pointer
   switch(accuB) {
      case 0xC000:
         parpro.barpro[0] = EEPROM.read(accuA); // EEPROM
      break;
      case 0x8000:
         parpro.barpro[0] = pad[accuA]; // SRAM
      break;
      default:
         parpro.barpro[0] = pgm_read_byte(&(body[accuA])); // FLASH
      break; 
   }
   parpro.barpro[1] = 0;
   parpro.barpro[2] = parpro.barpro[0] & 0x80; // make a test flag here 
   if (parpro.barpro[2] != 0) {
      parpro.barpro[1]--;
   }
   stack[stackPointer] = parpro.iarpro[0];
   stackPointer--;
   IP += 1;
}

void lit() {
   accuB = IP & 0xC000; // generate a case vector
   accuA = IP & 0x3FFF; // generate the pointer
   switch(accuB) {
      case 0xC000:
         parpro.barpro[1] = EEPROM.read(accuA); // EEPROM
         accuA++;
         parpro.barpro[0] = EEPROM.read(accuA); // EEPROM
      break;
      case 0x8000:
         parpro.barpro[1] = pad[accuA]; // SRAM
         accuA++;
         parpro.barpro[0] = pad[accuA]; // SRAM
      break;
      default:
         parpro.barpro[1] = pgm_read_byte(&(body[accuA])); // FLASH
         accuA++;
         parpro.barpro[0] = pgm_read_byte(&(body[accuA])); // FLASH
      break; 
   }
   stack[stackPointer] = parpro.iarpro[0];
   stackPointer--;
   IP += 2;
}

void dlit() {
   accuB = IP & 0xC000; // generate a case vector
   accuA = IP & 0x3FFF; // generate the pointer
   switch(accuB) {
      case 0xC000:
         parpro.barpro[3] = EEPROM.read(accuA); // EEPROM
         accuA++;
         parpro.barpro[2] = EEPROM.read(accuA); // EEPROM
         accuA++;
         parpro.barpro[1] = EEPROM.read(accuA); // EEPROM
         accuA++;
         parpro.barpro[0] = EEPROM.read(accuA); // EEPROM
      break;
      case 0x8000:
         parpro.barpro[3] = pad[accuA]; // SRAM
         accuA++;
         parpro.barpro[2] = pad[accuA]; // SRAM
         accuA++;
         parpro.barpro[1] = pad[accuA]; // SRAM
         accuA++;
         parpro.barpro[0] = pad[accuA]; // SRAM
      break;
      default:
         parpro.barpro[3] = pgm_read_byte(&(body[accuA])); // FLASH
         accuA++;
         parpro.barpro[2] = pgm_read_byte(&(body[accuA])); // FLASH
         accuA++;
         parpro.barpro[1] = pgm_read_byte(&(body[accuA])); // FLASH
         accuA++;
         parpro.barpro[0] = pgm_read_byte(&(body[accuA])); // FLASH
      break; 
   }
   accuA = parpro.iarpro[0];
   accuB = parpro.iarpro[1];
   pushA();
   pushB();
   IP += 4;
}
/*
void lit() { // read the next word in the instruction stream and push it to the stack; continue execution
	if (IP < 0) { // the data is in SRAM
		IPl = 0x7FFF & IP;
		accuA = pad[IPl];
		IPl++; IP++;
		accuB = pad[IPl];
	}
	else { // the data is in FLASH
		accuA = pgm_read_byte(&(body[IP]));
		IP++;
		accuB = pgm_read_byte(&(body[IP]));
	}
	accuA = accuA << 8;
	accuA |= accuB;
	pushA(); // push the word to the stack
	IP++; // and update the instruction pointer
}
*/
void pindir() { // (pin#, dir - ) ~ emulates data order of Arduino's pinMode(pin#, direction)
   popA(); // this is the pin#. Off limits pins will be included later
	popB(); // this is the direction; 0=IN, <>0=OUT
	if (accuB == 0) {
		pinMode(accuA, INPUT);
	}
	else {
		pinMode(accuA, OUTPUT);
	}
}

void pinsto() { // ( data, addr - ) ~ emulates data order of Arduino's digitalWrite(pin#, value)
	accuA = stack[stackPointer+1]; // the pin address
   accuB = stack[stackPointer+2]; // the pin data (0 or 1)
   stackPointer+=2;
	if (accuB == 0) {
		digitalWrite(accuA, LOW);
	}
	else {
		digitalWrite(accuA, HIGH);
	}
}

void pinFetch() { // ( addr - data )
	popA();
	accuB = digitalRead(accuA);
	pushB();
}

void analogFetch() { // ( analogPin# - analogValue )
   popA();
   accuD = analogRead(accuA);
   pushD();
}

void analogStore() {
   popA(); // this is the address of the PWM port
   popD(); // the number to output
   analogWrite(accuA, accuD);
}

void execute() { // clipped from working inner interpreter
   popA(); WA = accuA;
#if defined(DEVMODE)
   Serial2.write(0x45);
   innerTest();
#endif
   int tWA; //////////////////////////////////////CA<--@WP; WP++///////////////////////////////////////////
   if ( WA < 0 ) { // the WA is in SRAM (the upper part of memory)
      tWA = WA & 0x3FFF;
      pivot.bivot[1] = pad[tWA]; // 
      tWA++; WA++; // increment pointer and pseudo-pointer synchronously
      pivot.bivot[0] = pad[tWA];
      //tWA++; WA++; // increment pointer and pseudo-pointer synchronously
   }
   else { // the WA is in FLASH (the lower part of memory)
      pivot.bivot[1] = pgm_read_byte(&(body[WA])); // msB to accuA via WA
      WA++; // increment WA a second time only when you need the pointer for DOES> or VARIABLE etc...
      pivot.bivot[0] = pgm_read_byte(&(body[WA])); // lsb to CA via WA
   }
   CA = pivot.iivot[0];
#if defined(DEVMODE)
   Serial2.write(0x45);
#endif
   WA++; // <---this not being here is the reason execute would not work with colon definitions 20170906
#if defined(DEVMODE)
   innerTest();
#endif
   int tCA; //////////////////////////////////////PA<--@CA; CA++///////////////////////////////////////////
   if ( CA < 0 ) { // the CA is in SRAM (the upper part of memory)
      tCA = CA & 0x3FFF;
      pivot.bivot[1] = pad[tCA]; // 
      tCA++; CA++; // increment pointer and pseudo-pointer synchronously
      pivot.bivot[0] = pad[tCA];
   }
   else { // the CA is in FLASH (the lower part of memory)
      pivot.bivot[1] = pgm_read_byte(&(body[CA])); // msB to accuA via WA
      CA++;
      pivot.bivot[0] = pgm_read_byte(&(body[CA])); // lsb to CA via WA
   }
   PA = pivot.iivot[0];
#if defined(DEVMODE)
   Serial2.write(0x45);
   innerTest();
   Serial2.write(10);
   Serial2.write(13);
#endif
   primative();
}

void oneplus() {
	stack[stackPointer+1]++;
}
void twoplus() {
	stack[stackPointer+1] += 2;
 }
void oneminus() {
	stack[stackPointer+1]--;
}
void twominus() {
	stack[stackPointer+1] -= 2;
}
void twotimes() {
	stack[stackPointer+1] = stack[stackPointer+1] << 1;
}
void twodivby() {
	stack[stackPointer+1] = stack[stackPointer+1] >> 1;
}

void minus() {
	popA();
	accuA = ~accuA;
	accuA++;
	pushA();
}

void dminus() { // DMINUS
	popA(); // hi
   popB(); // lo
   accuA ^= -1;
   accuB ^= -1;
   accuB++;
   if (accuB == 0) {
      accuA++;
   }
   pushB();
	pushA();
}

void stod() { // S->D single-to-double
	accuB = 0; // assume the high word is all '0'
	popA(); // get the word to sign extend
	if ((accuA & 0x8000) != 0) { // check the most-significant-bit (msb)
		accuB--; // if it's non-zero then set all the high word bits
	}
	pushA(); // push the low word
	pushB(); // push the high word
}

void ecstor() { // E! stores a byte in the EEPROM ( data, address -) 
	popA(); // addr
	popB(); // data
	EEPROM.update(accuA, accuB); // EEPROM.update(addr, val);
}

void ecfetch() { // E@ retrieves a byte from the EEPROM 
	popA(); // get the address
	accuB = EEPROM.read(accuA);
	pushB(); // push the data
}

void zlt() { // 0< 
   accuB = 0;
   popA();
   accuA &= 0x8000;
   if (accuA != 0) {
      accuB--;
   }
   pushB();
}
// mixed memory read access

// EEPROM.update(accuA, accuD); // EEPROM.update(addr, val);

//   15   14   13   12   11   10   09   08      07   06   05   04   03   02   01   00
//                                                                destination  source
// +----+----+----+----+----+----+----+----+  +----+----+----+----+----+----+----+----+
// |    |    |    |    |    |    |    |    |  |    |    |    |    |DDDD|DDDD|SSSS|SSSS|
// |    |    |    |    |    |    |    |    |  |    |    |    |    |DDDD|DDDD|SSSS|SSSS|
// +----+----+----+----+----+----+----+----+  +----+----+----+----+----+----+----+----+

// 00 = flash
// 01 = flash
// 10 = sram
// 11 = eeprom

// sram <-- flash    1000
// sram <-- flash    1001
// sram <-- sram     1010
// sram <-- eeprom   1011

// eeprom <-- flash  1100
// eeprom <-- flash  1101
// eeprom <-- sram   1110
// eeprom <-- eeprom 1111

void bmove() {
// count   destin  source
   popC(); popB(); popA(); 
   pivot.iivot[0] = accuA; pivot.iivot[0] = pivot.iivot[0] >> 14; pivot.iivot[0] &= 0x03; //      source seminybble
   pivot.iivot[2] = accuB; pivot.iivot[2] = pivot.iivot[2] >> 12; pivot.iivot[2] &= 0x0C; // destination seminybble
   accuD = pivot.iivot[0] | pivot.iivot[2]; // the decision variable is in accuD
   accuB &= 0x3FF; accuA &= 0x3FF;
   if ((accuD < 8)) {
      crlf(); Serial3.println("CMOVE can't write to FLASH."); 
   }
   else {
      switch (accuD) { 
         case 0x08: // sram <- flash     20
            for (CAh=0; CAh<accuC; CAh++) {
               pad[accuB+CAh] = pgm_read_byte(&(body[accuA+CAh]));
            }
         break;
         case 0x09: // sram <- flash     21
            for (CAh=0; CAh<accuC; CAh++) {
               pad[accuB+CAh] = pgm_read_byte(&(body[accuA+CAh]));
            }
         break;
         case 0x0A: // sram <- sram      22
            for (CAh=0; CAh<accuC; CAh++) {
               pad[accuB+CAh] = pad[accuA+CAh];
            }
         break;
         case 0x0B: // sram <- eeprom    23
            for (CAh=0; CAh<accuC; CAh++) {
               pad[accuB+CAh] = EEPROM.read(accuA+CAh);
            }
         break;
         case 0x0C: // eeprom <- flash   30
            for (CAh=0; CAh<accuC; CAh++) {
               EEPROM.update(accuB+CAh, pgm_read_byte(&(body[accuA+CAh])));
            }
         case 0x0D: // eeprom <- flash   31
            for (CAh=0; CAh<accuC; CAh++) {
               EEPROM.update(accuB+CAh, pgm_read_byte(&(body[accuA+CAh])));
            }
         break;
         case 0x0E: // eeprom <- sram    32
            for (CAh=0; CAh<accuC; CAh++) {
               EEPROM.update(accuB+CAh, pad[accuA+CAh]);
            }
         break;
         case 0x0F: // eeprom <- eeprom  33
            for (CAh=0; CAh<accuC; CAh++) {
               EEPROM.update(accuB+CAh, EEPROM.read(accuA+CAh));
            }
         break;
      }
   }
   pushD(); // make a copy of the decision variable
}

void cmove() { // (source, destination, count - )
   popC(); popB(); popA(); 
   accuD = accuB & 0x8000; // form a state flag
   accuD = accuD >> 1;
   accuD &= 0x7FFF;
   PAh = accuA & 0x8000;
   accuD |= PAh;
      switch (accuD) {
      case 0x0000:  // absurd ~ flash to flash
         crlf(); Serial3.println("CMOVE address range violation.");          
      break;
      case 0x4000: // good ~ flash to sram
         accuB &= 0x7FFF; accuA &= 0x7FFF;
         for (CAh = 0; CAh < accuC; CAh++) {
            CAl = pgm_read_byte(&(body[accuA]));
            pad[accuB] = CAl;
            accuA++; accuB++;
         }            
      break;
      case 0x8000: // nonsense ~ sram to flash
         crlf(); Serial3.println("CMOVE address range violation.");            
      break;
      case 0xC000: // altight ~ sram to sram
         accuB &= 0x7FFF; accuA &= 0x7FFF;
         for (CAh = 0; CAh < accuC; CAh++) {
            CAl = pad[accuA];
            pad[accuB] = CAl;
            accuA++; accuB++;
         }           
      break;
      default:
         sPortSel = 0;
         crlf();
         hexInt(PAl);
         Serial3.println(" is an illegal CMOVE op.");
      break;
   }
}

void fill() { // works on SRAM and EEPROM
   popB(); // char
   popC(); // count
   popA(); // addr
   accuD = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuD) {
      case 0xC000: // EEPROM
         for (pivot.iivot[0] = 0; pivot.iivot[0] < accuC; pivot.iivot[0]++) {
            EEPROM.update(accuA + pivot.iivot[0], accuB);
         }
      break;
      case 0x8000: // SRAM
         for (pivot.iivot[0] = 0; pivot.iivot[0] < accuC; pivot.iivot[0]++) {
            pad[accuA + pivot.iivot[0]] = accuB;
         }
      break;
      default:
         Serial3.println(" FLASH is an illegal FILL destination.");
      break;
   }
}


// outputs the contents of accuD to the port selected by the lower three bits in the COM# user variable
void soutSel() { // selectable serial output
   PAh = UPINIT + 0x03B; // the location of the user variable OUT (0x93C0 + 0x01A)
   PAh &= 0x3FFF; // place within the actual addressable range
   pivot.bivot[0] = pad[PAh]; // get the number stored in COM#
   pivot.bivot[0] &= 3; // range reduce to 1-of-4 selection (may change later to include more ports)
   switch(pivot.bivot[0]) {
      case 0:
         Serial.write(accuD);
      break;
      case 1:
         Serial1.write(accuD);
      break;
      case 2:
         Serial2.write(accuD);
      break;
      case 3:
         Serial3.write(accuD);
      break;
   }
}

void outplu() { // uses accuB as an address ~ expects increment in accuC
   popC();
   pushC();
   accuB = UPINIT + 0x01A; // the location of the user variable OUT (0x93C0 + 0x01A)
   accuB &= 0x3FFF; // place within the actual addressable range
   pivot.bivot[1] = pad[accuB]; // get the number stored in OUT
   pivot.bivot[0] = pad[accuB+1];
   pivot.iivot[0] += accuC; // add in the string length to OUT
   pad[accuB+1] = pivot.bivot[0]; // replace the incremented OUT
   pad[accuB] = pivot.bivot[1];
}

void spaces() { // increment OUT as well
   popA(); // get the number of spaces to output 
   accuB = UPINIT + 0x01A; // the location of the user variable OUT (0x93C0 + 0x01A)
   accuB &= 0x3FFF; // place within the actual addressable range
   pivot.bivot[1] = pad[accuB]; // get the number stored in OUT
   pivot.bivot[0] = pad[accuB+1];
   pivot.iivot[0] += accuA; // add in the string length to OUT
   pad[accuB+1] = pivot.bivot[0]; // replace the incremented OUT
   pad[accuB] = pivot.bivot[1];
   for (accuC = 0; accuC < accuA; accuC++) {
      accuD = 0x20;
      soutSel();
   }
}

void type() { // increments OUT as well
   popC(); // get the count
   popA(); // and the address
   if (accuC != 0) {
      accuB = UPINIT + 0x01A; // the location of the user variable OUT (0x93C0 + 0x01A)
      accuB &= 0x3FFF; // place within the actual addressable range
      pivot.bivot[1] = pad[accuB]; // get the number stored in OUT
      pivot.bivot[0] = pad[accuB+1];
      pivot.iivot[0] += accuC; // add in the string length to OUT
      pad[accuB+1] = pivot.bivot[0]; // replace the incremented OUT
      pad[accuB] = pivot.bivot[1];
      accuB = accuA & 0xC000; // create a 'switch' flag
      accuA &= 0x3FFF; // range reduce the address 
      for (PAl = 0; PAl < accuC; PAl++) {
         switch(accuB) {
            case 0xC000: // the string is in EEPROM
               accuD = EEPROM.read(accuA);
            break;
            case 0x8000: // the string is in SRAM
               accuD = pad[accuA];
            break;
            default: // the string has to be in FLASH
               accuD = pgm_read_byte(&(body[accuA]));
            break;
         }
      soutSel();
      accuA++;
      }
   }
}

/*
void pdotq() { // primative to handle strings embedded in the instruction stream (.")
   accuA = stackR[stackPointerR + 1]; // the top of the return stack
   accuD = accuA & 0xC000; // make a switch decision variable 
   accuA &= 0x3FFF; // mask off the bank bits
   switch(accuD) { // Get the count byte
      case 0xC000: // EEPROM
         accuC = EEPROM.read(accuA);
      break;
      case 0x8000: // SRAM
         accuC = pad[accuA];
      break;
      default: // FLASH
         accuC = pgm_read_byte(&(body[accuA]));
      break;
   }
   pushC();
   outplu(); // update the contents of OUT
   popC();
   stackR[stackPointerR + 1] += accuC; // update the return stack contents
   stackR[stackPointerR + 1]++;
   for (PAl = 0; PAl < accuC; PAl++){
      accuA++; // point at the next character
      switch(accuD) {
         case 0xC000: // EEPROM
            accuB = EEPROM.read(accuA);
         break;
         case 0x8000: // SRAM
            accuB = pad[accuA];
         break;
         default: // FLASH
            accuB = pgm_read_byte(&(body[accuA]));
         break; 
      }
      Serial3.write(accuB);
   }
}
*/
// this word could possibly be simplified to access only SRAM
// as this is the only reasonable place to locate variables
// since FLASH can't be written. Also the offsets are all 
// single bytes. 
void user() { // place SRAM address of the named variable on the stack
   accuC = WA;
   if (accuC < 0) { // takes place in SRAM
      accuB = pad[accuC]; // get the offset-byte of the word (this is a lo-endian VM)
   }
   else { // takes place in FLASH
      // pgm_read_byte(&(xAxis[i]))
      accuB = pgm_read_byte(&(body[accuC])); // get the hi-byte of the word (this is a lo-endian VM)
   }
   accuB += UPINIT; // add in the user base page
   pushB();
}

// This is a user version that uses 16-bit offsets
void userX() { // place SRAM address of the named variable on the stack
   accuC = WA;
   if (accuC < 0) { // takes place in SRAM
      accuB = pad[accuC]; // get the hi offset-byte of the word (this is a lo-endian VM)
      accuB = accuB << 8; // shift it into position
      accuC++; // point to the lo-byte
      accuA = pad[accuC]; // fetch it
   }
   else { // takes place in FLASH
      // pgm_read_byte(&(xAxis[i]))
      accuB = pgm_read_byte(&(body[accuC])); // get the hi-byte of the word (this is a lo-endian VM)
      accuB = accuB << 8;
      accuC++;
      accuA = pgm_read_byte(&(body[accuC]));
   }
   accuB |= accuA;
   accuB += UPINIT; // add in the user base page
   pushB();
}
// This routine has its data in the upper part of the EEPROM bank
// The offsets are all 
// single bytes. 
void userEE() { // place EEPROM address of the named variable on the stack
   accuC = WA;
   if (accuC < 0) { // takes place in SRAM
      accuB = pad[accuC]; // get the offset-byte of the word (this is a lo-endian VM)
   }
   else { // takes place in FLASH
      // pgm_read_byte(&(xAxis[i]))
      accuB = pgm_read_byte(&(body[accuC])); // get the hi-byte of the word (this is a lo-endian VM)
   }
   accuB += EPINIT; // add in the user base page
   pushB();
   fetch();
}

///////////////////////////////////////////////////////////
//                       Fig-Model                       //
//   : EXPECT                                            //
//     OVER + OVER                                       //
//     DO                                                //
//        KEY DUP 0E +ORIGIN @ =                         // is it a backspace?
//           IF                                          //
//              DROP 08 OVER I = DUP L> 2 - + >L -       // replace the latest character with an 0x08 and backup the counter
//           ELSE                                        //
//              DUP 0D = IF                              // 
//                          LEAVE DROP BL 0              //
//                       ELSE                            //
//                          DUP                          //
//                       THEN                            //
//              I C! 0 I 1 + !                           //
//           THEN                                        //
//           EMIT                                        //
//     LOOP DROP                                         //
//    ;                                                  //
///////////////////////////////////////////////////////////

// UPINIT + 0x01A ~ address of OUT
// There is a problem with the bs function. It can never be brought to the beginning of the line
// and each time it is used the 'unvisitable' place at the beginning is advanced one character
// further away 
void expect() { // EXPECT ( addr, count - )
   popC(); // get the count
   popB(); // and the address
   int i = 0; // INDEX
   int j = accuB & 0x3FFF; // put in writable media range
   int k = accuB & 0xC000; // save media type
   int l = (accuB + accuC) & 0x3FFF; // form LIMIT
   //int m = (UPINIT + 0x01E) & 0x3FFF; // form pointer to OUT
   int m = 0x13DA; // point to the lower of the two OUT variable bytes
   accuB &= 0x3FFF; // appropriately range-reduce the address 
   i = accuB; // set the INDEX equal to the first address location
   while ( i < l ) {
      key3(); popA(); // character is in 'accuA'
#if defined(DEVMODE)
      Serial2.print("\033[38;5;200m"); // T hot pink
      Serial2.write(accuA);  Serial2.print(" "); 
      Serial2.print(i, HEX); Serial2.print(" "); 
      Serial2.print(l, HEX); Serial2.print(" "); 
      Serial2.println(accuB, HEX);
      Serial2.print("\033[38;5;231m"); // T white 
#endif
      if (accuA == 0x07F) { // a backspace character
         accuA = 0x08; // the backspace command to be output to the terminal
         i -= 2; // decrement the pointer as is appropriate
         if (i < accuB) { // keep it within limits should it go below the lower boundary
            i = accuB; // do not go below limit
            accuA = 0x07;
         }
      }
      else {
         if (accuA == 0x0D) { // a carriage return
            l = 0;
            accuA = 0x20;
         }
         else {
            pad[i] = accuA;
            pad[i+1] = 0;
            pad[i+2] = 0;
         }
      }
      pushA(); // place the character on the stack and
      emit3();  // output the character to the terminal for visual display
      pad[m+1] += 1; // finally increment the OUT variable
      if (pad[m+1] == 0) {
         pad[m]++;
      }
      i++; // and increment the address/counter
   }
   //accuA = 0x0D; // insert a line feed character as well - the linux terminal needs one
   //pushA(); // place the character on the stack and
   //emit3();  // output the character to the terminal for visual display
   pad[m+1] += 1; // finally increment the OUT variable
   if (pad[m+1] == 0) {
      pad[m]++;
   }
   pad[i] = 0;
   pad[i+1] = 0;
}
//              ---->8088 Model<----

//    POP   AX          ;S1 - TERMINATOR CHAR
//    POP   BX          ;S2 - TEXT ADDR
//    PUSH  BX          ;ADDR - BACK TO STACK ( IT RHYMES )
//    MOV   AH,0        ;ZERO
//    MOV   DX,-1       ;CHAR OFFSET COUNTER
//    DEC   BX          ;ADDR -1
// ;
// ;   SCAN TO FIRST NON-TERMINATOR CHARACTER
// ENCL1: <----------------------------------------+  
//    INC   BX          ;ADDR+1                    |
//    INC   DX          ;COUNT+1                   |
//    CMP   AL,[BX]                                |
//    JZ ENCL1          ;WAIT FOR NON-TERMINATOR---+
//    PUSH  DX          ;OFFSET TO 1ST TEXT CHAR
//    CMP   AH,[BX]     ;NULL CHAR?
//    JNZ   ENCL2       ;NO------------------------+
// ;                                               |
// ;  FOUND NULL BEFORE 1ST NON-TERM CHAR          |
//    MOV   AX,DX       ;COPY COUNTER              |
//    INC   DX ; +1                                |
//    JMP   DPUSH                                  |
// ;                                               |
// ;   FOUND FIRST TEXT CHAR - COUNT THE CHARS     |
// ENCL2: <----------------------------------o-----+                                      
//    INC   BX          ; ADDR+1             |
//    INC   DX          ;COUNT+1             |
//    CMP   AL,[BX]     ;TERMINATOR CHAR?    |
//    JZ ENCL4          ;YES-----------------+-----+
//    CMP   AH,[BX]     ;NULL CHAR?          |     |
//    JNZ   ENCL2       ;NO, LOOP AGAIN------+     |
// ;                                               |
// ;   FOUND NULL AT END OF TEXT                   |
// ENCL3:                                          |
//    MOV   AX,DX       ;COUNTERS ARE EQUAL        |
//    JMP   DPUSH                                  |
// ;                                               |
// ;   FOUND TERMINATOR CHARACTER                  |
// ENCL4: <----------------------------------------+   
//    MOV   AX,DX 
//    INC   AX          ;COUNT+1
//    JMP   DPUSH

void enclose() { // ( beg_addr, delim  -  beg_addr, ofs, ofs, ofs)
   popA(); // delimiter
   pivot.iivot[0] = accuA; // the delimiter (to eliminate the possibility of data type incompatibility)
   popB(); // address
   pushB(); // save a copy
   accuB &= 0x3FFF; // and adjust so SRAM is accessible
   accuD = -1; // an offset counter
   accuB--; // addr - 1
// scan to first non-delimiter
encl1: // <----------------------------------------+
   accuB++; // increment addr                      |
   accuD++; // and offset                          |
   if (pivot.bivot[0] == pad[accuB]) { //          |
      goto encl1; //-------------------------------+
   }
   pushD(); // offset to first text character
   if (pad[accuB] != 0) {
      goto encl2; //-------------------------------+
   } // found null before 1st non-delimiter char   |
   accuA = accuD; // copy counter                  |
   accuD++; //                                     |
   goto dpush; //                                  |
// found first text char - count the chars         |
encl2: // <---------------------o------------------+
   accuB++; //                  |
   accuD++; //                  |
   if (pivot.bivot[0] == pad[accuB]) { //|delimiter char?
      goto encl4; //------------+------------------+
   } //                         |                  |
   if (pad[accuB] != 0) { //    |                  |
      goto encl2; //------------+                  |
   } //                                            |
// found null at end of text                       |
encl3: //                                          |
   accuA = accuD; //                               |
   goto dpush; //                                  |
// found terminator char                           |
encl4: // <----------------------------------------+
   accuA = accuD;
   accuA++;
dpush:
   pushD();
   pushA();
}

//              ---->8088 Model<----
//
//   MOV   AX,DS
//   MOV   ES,AX           ;ES = DS
//   POP   BX              ;NFA <----------------This is the target string
//   POP   CX              ;STRING ADDR <--------This is the linked list
//;
//;  SEARCH LOOP
//PFIN1: <----------------------------------------------------+
//   MOV   DI,CX           ;GET LINKED LIST ADDR              |
//   MOV   AL,[BX]         ;GET TARGET WORD LENGTH            |
//   MOV   DL,AL           ;SAVE WORD LENGTH                  |
//   XOR   AL,[DI]                                            |
//   AND   AL,3FH          ;CHECK LENGTHS                     |
//   JNZ   PFIN5           ;LENGTHS DIFFER----------------+   |
//                                                        |   |
//;                                                       |   |
//;   LENGTHS MATCH - CHECK EACH CHARACTER IN NAME        |   |
//PFIN2: <--------------------------------------------+   |   |
//   INC   BX                                         |   |   |
//   INC   DI              ; NEXT CHAR OF NAME        |   |   |
//   MOV   AL,[BX]                                    |   |   | 
//   XOR   AL,[DI]         ;COMPARE NAMES             |   |   |
//   ADD   AL,AL           ;THIS WILL BE TEST BIT 8   |   |   |
//   JNZ   PFIN5           ;NO MATCH------------------+---o   |
//   JNB   PFIN2           ;MATCH SO FAR - LOOP-------+   |   |
//;                                                       |   |  
//;   FOUND END OF NAME (BIT 8 SET) - A MATCH             |   |
//   ADD   BX,5            ; BX = PFA                     |   |
//   PUSH  BX              ; (S3) <- PFA                  |   |
//   MOV   AX,1            ;TRUE VALUE                    |   |
//   SUB   DH,DH                                          |   |
//   JMP   DPUSH                                          |   |
//;                                                       |   |
//;   NO NAME MATCH - TRY ANOTHER                         |   |
//;                                                       |   |
//; GET NEXT LINK FIELD ADDR (LFA)                        |   |
//; ( ZERO = FIRST WORD OF DICTIONARY )                   |   |
//;                                                       |   |
//PFIN5: <------------------------------------------------+   |
//   INC   BX              ;NEXT ADDR                     ^   |   
//   JB PFIN6              ;END OF NAME                   |   | JB = jump if below
//   MOV   AL,[BX]         ;GET NEXT CHAR                 |   |
//   ADD   AL,AL           ;SET/RESET CARRY               |   |
//   JMP   PFIN5           ;LOOP UNTIL FOUND--------------+   |
//;                                                           |
//PFIN6:                                                      |
//   MOV   BX,[BX]         ; GET LINK FIELD ADDR              |
//   OR BX,BX              ; START OF DICT ( 0 )              |
//   JNZ   PFIN1           ; NO , LOOK MORE-------------------+
//   MOV   AX,0            ; FALSE FLAG
//   JMP   APUSH           ; DONE ( NO MATCH FOUND )
//;

void pfind() { // (FIND)   ( linked_list, target_string - )
   uint8_t ds; // destination string char
   uint8_t ts; // target string char
   uint8_t lt; // the length byte of the target word
   uint8_t ll; // the length byte of the linked list word
   uint8_t lm; // masked linked list length byte
   uint8_t targ[0x1F]; // the target $ with the maximal number of characters
   int i; // internal index
   popC(); // linked list (NFA)
   popB(); // target string (string address)
   lt = pFinList(accuB); // get length byte of target string
   targ[0] = lt; // set the count byte in the target array
   for (int i = 1; i <= lt; i++) { // internalize the target string
      targ[i] = pFinList(accuB + i); // to reduce string retrieval times
   } //
pfin1: // EXAMINE THE LENGTH BYTE IN BOTH STRINGS <------------------------------------------------+
   ll = pFinList(accuC); // this will be used later to jump over chars if "no match"               |
   lm = ll & 0x1F; //                                                                              |
   accuD = accuC; // get addr ~ D & C = linked list; c is persistent; d is malteable               |
   if (lt != (lm)) { // length bytes (with compiler bits removed) aren't equal                     |
      goto pfin5; // step to next address in list>-----------------------------------------------+ |
   } //                                                                                          | |
pfin2: // LENGTHS MATCH; CHECK EACH CHARACTER IN NAME(S)                                         | |
   for (i = 1; i <= lt; i++) { // differs from model; look for loop len, not hi-bit set <------+ | |
      accuD++; // increment linked list address to point to the next byte                      | | |
      ds = pFinList(accuD); // get the character there                                         | | |
      ts = targ[i]; // and the corresponding character in the target $                         | | |
      if (ts != ds) { // if they are not equal get out and find the next address               | | |
         goto pfin5; // no match>--------------------------------------------------------------|-o |
      } //-------------------------------------------------------------------------------------+ | |
   } // everything matched - exit true                                                           | |
   accuC += (int)lm; accuC += 5; pushC(); // push the NFA                                        | |
   accuA = -1; // and a true flag                                                                | |
   accuD = (int)ll; // the length                                                                | |
   goto dpush; //                                                                                | |
pfin5: //   NO NAME MATCH - TRY ANOTHER <--------------------------------------------------------+ |
   accuA = (int)lm; // get the masked length byte of this word                                     |
   accuC += accuA; //                                                                              |
   accuC++; // now accuC is pointing at the LFA                                                    |
   PAh = pFinList(accuC); //                                                                       |
   PAh = PAh << 8; //                                                                              |
   accuC++; //                                                                                     |
   PAl = pFinList(accuC); //                                                                       |
   accuC = PAh | PAl; // next LFA is now in accuC                                                  |
   if (accuC == 0) { //                                                                            |
      accuA = 0; //                                                                                |
      goto apush; //                                                                               |
   } //                                                                                            |
   goto pfin1; //>---------------------------------------------------------------------------------+
dpush: //
   pushD(); //
apush: //
   pushA(); //
}
// the following is factored out code to search the list which can be in either FLASH, SRAM, or EEPROM
uint8_t pFinList(int addr) { 
   uint8_t dat;
   //addr &= 0x3FFF; // mask off all but the address bits
   int adsel = addr & 0xC000;
   addr &= 0x3FFF;
   switch(adsel) {
      case 0xC000:
         dat = EEPROM.read(addr);
      break;
      case 0x8000:
         dat = pad[addr];
      break;
      default:
         dat = pgm_read_byte(&(body[addr]));
      break;
   }
   return dat;
}
// The above routine can only reasonably be expected to have as its target string an 
// entity residing in SRAM. The list is probably in either FLASH or SRAM but could 
// ultimately be anywhere in the uController's storage areas.

void digit() { // DIGIT
	accuD = -1; // a true flag
	popB(); // get base
	popC(); // get char
	accuC -= 0x30; // subtract the numerical ascii bias
	if (accuC < 0) {
		goto exitfalse;
	}
	if (accuC <= 9) {
		goto basecheck;
	}
	accuC -= 7;
	if (accuC < 0) {
		goto exitfalse;
	}
basecheck:
	if (accuC > accuB) {
		goto exitfalse;
	}
	pushC(); 
	goto pushd;
exitfalse:
	accuD++;
pushd:
	pushD();
}

void toggle() { // (p addr, mask - ) TOGGLE
   parpro.larpro = 0;
	popD(); // get the mask
   pivot.iivot[0] = accuD;
	popA(); // get the address
   accuB = accuA & 0xC000;
   accuA &= 0x3FFF;
   switch(accuB) { // EEPROM
      case 0xC000:
         parpro.barpro[0] = EEPROM.read(accuA);
         parpro.barpro[0] ^= pivot.bivot[0];
         EEPROM.update(accuA, parpro.barpro[0]);
      break;
      case 0x8000: // SRAM
         parpro.barpro[0] = pad[accuA];
         parpro.barpro[0] ^= pivot.bivot[0];
         pad[accuA] = parpro.barpro[0];
      break;
      default: // everything unwritable
         Serial3.println("TOGGLE referenced FLASH illegally");
      break;
   }
}

void less() { // <
	popB();
	popA();
	accuD = 0;
	accuC = accuA - accuB;
	if (accuC < 0) {
		accuD--;
	}
	pushD();
}

void great() { // >
	popB();
	popA();
	pushB();
	pushA();
	less();
}

void dplus() { // D+
	popC(); popD();
   popA(); popB();
	pivot.iivot[0] = accuB; pivot.iivot[1] = accuA;
   parpro.iarpro[0] = accuD; parpro.iarpro[1] = accuC;
   pivot.livot += parpro.larpro;
   accuA = pivot.iivot[1];
   accuB = pivot.iivot[0];
	pushB(); pushA();
}

void twotest() {
	popAx();
	pushAx();
}

void dovar() { // this version will not work in FLASH, think of something else
   accuA = WA;
   pushA();
}

void docon() {
   if (WA < 0) { // word is defined in SRAM
      WA &= 0x3FFF;
      accuA = pad[WA];
      accuA = accuA << 8;
      WA++;
      accuA |= pad[WA];
   }
   else {
      accuA = pgm_read_byte(&(body[WA]));
      accuA = accuA << 8;
      WA++;
      accuA |= pgm_read_byte(&(body[WA]));
   }
   pushA();
}

void cswap() { // ( 1234h - 3412h )
   popA();
   accuC = accuA >> 8;
   accuA = accuA << 8;
   accuA |= accuC;
   pushA();
}

void spat() { // SP@
   accuA = stackPointer;
   pushA();
}

void spsto() { // SP! ~ initialize the stack pointer
   stackPointer = PSTACKLEN - 1;
}

void rpat() { // SP@
   accuA = stackPointerR;
   pushA();
}

void rpsto() { // SP! ~ initialize the stack pointer
   stackPointerR = RSTACKLEN - 1;
}

void lpat() { // LP@
   accuA = stackPointerL;
   pushA();
}

void lpsto() { // LP! ~ initialize the stack pointer
   stackPointerL = LSTACKLEN - 1;
}

void fpat() { // FP@
   accuA = stackPointerF;
   pushA();
}

void fpsto() { // FP! ~ initialize the stack pointer
   stackPointerF = FSTACKLEN - 1;
}


void pick() { // PICK
   popA();
   accuA += stackPointer;
   accuA++;
   accuA &= PSTACKZED;
   accuB = stack[accuA];
   pushB();
}

void millisIs() {
   parpro.larpro = millis();
   accuB = parpro.iarpro[0]; // lo
   accuA = parpro.iarpro[1]; // hi
   pushB();
   pushA();
}

void messages() {
   popA();
   Serial3.print("\033[38;5;200m"); // T hot pink
   switch(accuA) {
      case 0x00:
         Serial3.println("is not defined.");
      break;
      case 0x01:
         Serial3.println("Empty stack.");
      break;
      case 0x02:
         Serial3.println("Dictionary full.");
      break;
      case 0x04:
         Serial3.println("isn't unique.");
      break;
      case 0x06:
         Serial3.println("Disk range?");
      break;
      case 0x07:
         Serial3.println("Full stack.");
      break;
      case 0x08:
         Serial3.println("disk error.");
      break;
      case 0x11:
         Serial3.println("compilation only.");
      break;
      case 0x12:
         Serial3.println("execution only.");
      break;
      case 0x13:
         Serial3.println("conditionals not paired.");
      break;
      case 0x14:
         Serial3.println("definition not finished.");
      break;
      case 0x15:
         Serial3.println("protected dictionary.");
      break;
      case 0x16:
         Serial3.println("use only when loading.");
      break;
      case 0x17:
         Serial3.println("off current editing screen .");
      break;
      case 0x18:
         Serial3.println("declare vocabulary.");
      break;
   }
   Serial3.print("\033[38;5;231m"); // T white
}
//   ____  ____    _                        
//  / ___||  _ \  / \   ___ ___ ___ ___ ___ 
//  \___ \| | | |/ _ \ / __/ __/ _ / __/ __|
//   ___) | |_| / ___ | (_| (_|  __\__ \__ \
//  |____/|____/_/   \_\___\___\___|___|___/

void fNameSto() { // store a single character in the file name array and convert that to a string
   popA(); // get the address
   popD(); // get the data
   pivot.iivot[0] = accuD;
   fileName[accuA] = pivot.bivot[0];
}
// uint8_t my_str[6];

// Serial.println(sizeof(str))
void fNameDot() {
   //String fStr((char*)fileName);
   Serial3.println(fileName);
}

void sdInit() { // this should have an argument of either RD or WR
   Serial3.print("\033[38;5;200m"); // T hot pink
   Serial3.write(0x0D);
   Serial3.write(0x0A);
   Serial3.println("Initializing SD card...");

   if (!SD.begin(SDCS)) { // CS options can be found above
      Serial3.println("initialization failed!");
      return;
   }
   Serial3.println("initialization done.");
   Serial3.print("\033[38;5;231m"); // T white
}

void fsize() { // return the 32-bit size (in bytes) of the currently open file
   pivot.livot = theFile.size();
   accuB = pivot.iivot[0];
   accuA = pivot.iivot[1];
   pushB();
   pushA();
}

// the fopen and fclose routine will be changing states repeatedly within the context
// of an edit session. I have created another user variable called VERBOSE. When this
// variable is true the text information associated with the two operations will be 
// asserted. When it's false the text will be suppressed. 
// void verboTog() { // accept a stack variable to set the VERBOSE user variable
//   popD(); // the data to be placed in VERBOSE
//   accuB = UPINIT + 0x032; // the location of the user variable VERBOSE (0x93C0 + 0x032)
//   accuB &= 0x3FFF; // place within the actual addressable range
//   pivot.bivot[1] = pad[accuB]; // get the number stored in OUT
//   pivot.bivot[0] = pad[accuB+1];
//   pivot.iivot[0] += accuC; // add in the string length to OUT
//   pad[accuB+1] = pivot.bivot[0]; // replace the incremented OUT
//   pad[accuB] = pivot.bivot[1];
//}

void fopen() { // preceed with either READ or WRITE to specify the operations intended
   popA(); // 0 for read / non-0 for write
   if (accuA == 0) {
      theFile = SD.open(fileName, FILE_READ);
   }
   else {
      theFile = SD.open(fileName, FILE_WRITE);
   }
   //Serial3.print("\033[38;5;106m"); // T muted green
   //if (theFile) {
      //Serial3.print(fileName);
      //Serial3.println(" is open.");
   //}
   //else {
      //Serial3.print(fileName);
      //Serial3.println(" did not open!");
   //}
   //Serial3.print("\033[38;5;231m"); // T white
}

void fclose() { // close the file
   theFile.close();
   //Serial3.print("\033[38;5;200m"); // T hot pink
   //Serial3.print(fileName);
   //Serial3.println(" is now closed.");
   //Serial3.print("\033[38;5;231m"); // T white
}

void femit() { // write the value on the stack to the current byte location in the file 
   popD();
   pivot.iivot[0] = accuD;
   theFile.write(pivot.bivot[0]);
}

void fkey() { // get the next available byte from the file automatically advancing the file pointer
   pivot.bivot[0] = theFile.read();
   pivot.bivot[1] = 0;
   accuD = pivot.iivot[0];
   pushD();
}

void fseek() { // set the 32-bit file pointer with the double-number on the stack
   popA(); // get the hi word
   popB(); // get the lo word
   pivot.iivot[1] = accuA;
   pivot.iivot[0] = accuB;
   theFile.seek(pivot.uivot); 
}

void fpos() { // return the 32- position of the file pointer
   pivot.livot = theFile.position();
   accuA = pivot.iivot[1];
   accuB = pivot.iivot[0];
   pushB();
   pushA();
}

void printDirectory(File dir, int numTabs) {
   while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       // return to the first file in the directory
       dir.rewindDirectory();
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
   }
}

void ls() {
   root = SD.open("/");
   printDirectory(root, 0);
}
  
//  if (theFile)
//  {
//    Serial.print("Writing to the text file...");
//    myfile.println("Congratulations! You have successfully wrote on the text file.");
//   
//    myfile.close(); // close the file:
//    Serial.println("done closing.");
//  } else
//  {
//    // if the file didn't open, report an error:
//    Serial.println("error opening the text file!");
//  }

void primative() {
#if defined(DEVMODE)
   seemative();
#endif
	switch (PA) {
    	case 0x00:
    		key3();           // KEY
    	break;
    	case 0x01:
    		emit3();          // (EMIT3)
    	break;
    	case 0x02:
    		noop();           // NOOP
    	break;
    	case 0x03:
    		equals();         // =
    	break;
    	case 0x04:
    		mtwo();           // -2
    	break;
    	case 0x05:
    		mone();           // -1
    	break;
    	case  0x06:
    		two();            // 2
   	break;
   	case 0x07:
  	 	 	one();            // 1 
   	break;
    	case 0x08:
    		zero();           // 0
    	break;
    	case 0x09:
    		zeq();            // 0=
    	break;
    	case 0x0A:
    		cstore();         // C!
    	break;     
    	case 0x0B:
    		cfetch();         // C@
    	break;  
    	case 0x0C:
    		store();          // ! <---------------------store
    	break;  
    	case 0x0D:
    		fetch();          // @ <---------------------fetch
    	break;  
    	case 0x0E:
    		rot();            // ROT
    	break; 
    	case 0x0F: 
    		div();            // U/
    	break;
    	case 0x10:
    		mul();            // U*
    	break;
    	case 0x11:
    		sub();            // -
    	break;   
    	case 0x12: 	 
    		add();            // +
    	break;
    	case 0x13:
    		over();           // OVER
    	break;
    	case 0x14:
    		swap();           // SWAP
    	break;
    	case 0x15:
    		dup();            // DUP
    	break;
    	case 0x16:
    		voodoop();        // (DO)
    	break;
    	case 0x17:
    		rfrom();          // R>
    	break;
    	case 0x18:
    		tor();            // >R
    	break;
    	case 0x19:
    		bran();           // BRANCH
    	break;
    	case 0x1A:
    		zbran();          // 0BRANCH
    	break;
    	case 0x1B:
    		lit();            // LIT
    	break;
    	case 0x1C:
    		docol();          // (DOCOL) runtime routine for ':'
    	break;
    	case 0x1D:
    		semi();           // (SEMIS) runtime routine for ';S'
    	break;
    	case 0x1E:
    		eye();            // I a loop outer index
    	break;
    	case 0x1F:
    		jay();            // J an inner loop index
    	break;
    	case 0x20:
    		kay();            // K another inner loop index
    	break; 
    	case 0x21:
    		rfetch();         // R@
    	break;  
    	case 0x22:
    		drop();           // DROP
    	break;	
    	case 0x23:
    		pindir();         // <P>
    	break;	
    	case 0x24:
    		pinsto();         // P!
    	break;	
    	case 0x25:
    		pinFetch();       // P@
    	break;	
    	case 0x26:
    		hexIntFig();      // HEX.
    	break;	
    	case 0x27:
    		execute();        // EXECUTE
    	break;	
    	case 0x28:
    		oneplus();        // 1+
    	break;	
    	case 0x29:
    		oneminus();       // 1-
    	break;	
    	case 0x2A:
    		twoplus();        // 2+
    	break;	
    	case 0x2B:
    		twominus();       // 2-
    	break;	
    	case 0x2C:
    		twodrop();        // 2DROP
    	break;	
    	case 0x2D:
    		twotimes();       // 2*
    	break;	
    	case 0x2E:
    		twodivby();       // 2/
    	break;	  
    	case 0x2F:
    		minus();          // MINUS
    	break;
    	case 0x30:
    		dminus();         // DMINUS
    	break;  
    	case 0x31:
    		por();            // OR
    	break; 
    	case 0x32:
    		pand();           // AND
    	break; 
    	case 0x33:
    		pxor();           // XOR
    	break; 	 
    	case 0x34:
    		stod();           // S->D
    	break; 	   	 
    	case 0x35:
    		millisIs();       // MILLIS
    	break; 
    	case 0x36:
    		messages();       // messages
    	break;  
      case 0x37:
        	qterm3();          // ?TERMINAL
      break;    	
      case 0x38:
         zlt();            // 0<
      break;       
      case 0x39:
         paloop();         // (LOOP)
      break;  	
      case 0x3A:
         paploop();        // (+LOOP)
      break;  
      case 0x3B:
       	userX();          // userX ~ the DOUSE routine for 16-bit offsets
      break;  
      case 0x3C:
         leave();          // LEAVE
      break; 
      case 0x3D:
         cmove();          // CMOVE
      break; 
      case 0x3E:
         mdots();          // .S
      break; 
      case 0x3F:
         dotsL();          // .SL
      break; 
      case 0x40:
         SRAMdump();       // DUMP
      break; 
      case 0x41:
         user();           // douse for 8-bit offsets
      break; 
      case 0x42:
         fill();           // FILL
      break; 
      case 0x43:
         tol();            // >L loop stack operation, pop loop stack to parameter stack
      break; 
      case 0x44:
         lfrom();          // L> push parameter stack to loop stack
      break; 
      case 0x45:
         expect();         // EXPECT
      break; 
      case 0x46:
         enclose();        // ENCLOSE
      break; 
      case 0x47:
         pfind();          // (FIND)
      break; 
      case 0x48:
         lfetch();         // L@
      break; 
      case 0x49:
         dstore();         // 2!
      break; 
      case 0x4A:
         digit();          // DIGIT
      break; 
      case 0x4B:
         toggle();         // TOGGLE
      break; 
      case 0x4C:
         less();           // less
      break; 
      case 0x4D:
         great();          // great
      break; 
      case 0x4E:
         dplus();          // D+
      break; 
      case 0x4F:
         fNameSto();       // FNAME! ~ stores a name in the filename array of the form XXXXXXXX.YYY0 <--- note that it is zero terminated
      break; 
      case 0x50:
         plusstore();      // +!
      break;
      case 0x51:
         dovar();          // dovar
      break;
      case 0x52:
         docon();          // docon
      break;
      case 0x53:
         cswap();          // CSWAP
      break;
      case 0x54:
         spat();           // SP@
      break;
      case 0x55:
         spsto();          // SP!
      break;
      case 0x56:
         rpat();           // RP@
      break;
      case 0x57:
         rpsto();          // RP!
      break;
      case 0x58:
         pick();           // PICK
      break;
      case 0x59:
         analogFetch();    // A@
      break;
      case 0x5A:
         pupi();           // PUPI
      break;
      case 0x5B:
         fdot();           // F.
      break;
      case 0x5C:
         dtof();           // D>F
      break;
      case 0x5D:
         fslas();          // F/
      break;
      case 0x5E:
         ftod();           // F>D
      break;
      case 0x5F:
         stof();           // S>F
      break;
      case 0x60:
         ftos();           // F>S
      break;
      case 0x61:
         fswap();          // FSWAP
      break;
      case 0x62:
         fdrop();          // FDROP
      break;
      case 0x63:
         fdup();           // FDUP
      break;
      case 0x64:
         frot();           // FROT
      break;
      case 0x65:
         fstar();          // F*
      break;
      case 0x66:
         fplus();          // F+
      break;
      case 0x67:
         fsubb();          // F-
      break;
      case 0x68:
         fsin();           // FSIN
      break;
      case 0x69:
         fcos();           // FCOS
      break;
      case 0x6A:
         ftan();           // FTAN
      break;
      case 0x6B:
         dotsf();          // F.S
      break;
      case 0x6C:
         fasin();          // FASIN
      break;
      case 0x6D:
         facos();          // FACOS
      break;
      case 0x6E:
         fatan();          // FATAN
      break;
      case 0x6F:
         fatan2();         // FATAN2
      break;
      case 0x70:
         fsqrt();          // FSQRT
      break;
      case 0x71:
         fover();          // FOVER
      break;
      case 0x72:
         fpow();           // FPOW
      break;
      case 0x73:
         fexp();           // FEXP
      break;
      case 0x74:
         fat();            // F@
      break;
      case 0x75:
         fstore();         // F!
      break;
      case 0x76:
         dlit();           // DLIT
      break;
      case 0x77:
         spaces();         // SPACES
      break;
      case 0x78:
         type();           // TYPE
      break;
      case 0x79:
         type();           // (.") <*****************************************************************************************************
      break;
      case 0x7A:
         analogStore();    // A!
      break;
      case 0x7B:
         dodoes();         // DOES>
      break;
      case 0x7C:
         fNameDot();       // FNAME.
      break;
      case 0x7D:
         sdInit();         // SDINIT
      break;
      case 0x7E:
         fsize();          // FSIZE
      break;
      case 0x7F:
         fopen();          // FOPEN
      break;
      case 0x80:
         fclose();         // FCLOSE
      break;
      case 0x81:
         femit();          // FEMIT
      break;
      case 0x82:
         fkey();           // FKEY
      break;
      case 0x83:
         fseek();          // FSEEK
      break;
      case 0x84:
         fpos();           // FPOS
      break;
      case 0x85:
         userEE();         // USEREE ~ EEPROM based user bank
      break;
      case 0x86:
         emit2();          // (EMIT2)
      break;
      case 0x87:
         emit1();          // (EMIT1)
      break;
      case 0x88:
         emit0();          // (EMIT0)
      break;
      case 0x89:
         pof();            // (OF)
      break;
      case 0x8A:
         fcells();         // FCELLS
      break;
      case 0x8B:
         eyep();           // I'
      break;
      case 0x8C:
         jayp();           // J'
      break;
      case 0x8D:
         kayp();           // K'
      break;
      case 0x8E:
         fpstor();         // F+!
      break;
      case 0x8F:
         fourmod();        // 4MOD
      break;
      case 0x90:
         sxtmod();         // 16MOD
      break;
      case 0x91:
         fourslas();       // 4/
      break;
      case 0x92:
         sxtslas();        // 16/
      break;
      case 0x93:
         flt();            // F<
      break;
      case 0x94:
         ls();             // LS
      break;
      case 0x95:
         clit();           // CLIT
      break;
      case 0x96:
         bmove();          // BMOVE (prospective CMOVE replacement)
      break;
      case 0x97:
         key3();           // (KEY3)
      break;
      case 0x98:
         key2();           // (KEY2)
      break;
      case 0x99:
         key1();           // (KEY1)
      break;
      case 0x9A:
         key0();           // (KEY0)
      break;
      case 0x9B:
         setBaud0();       // (SETBAUD0)
      break;
      case 0x9C:
         setBaud1();       // (SETBAUD1)
      break;
      case 0x9D:
         setBaud2();       // (SETBAUD2)
      break;
      case 0x9E:
         setBaud3();       // (SETBAUD3)
      break;
      case 0x9F:
         lpat();           // LP@
      break;
      case 0xA0:
         lpsto();          // LP!
      break;
      case 0xA1:
         fpat();           // FP@
      break;
      case 0xA2:
         fpsto();          // FP!
      break;
    	default:
            sPortSel = PORTMAIN;
            crlf();
            hexInt(PA);
    		Serial3.println(" is an illegal instruction.");
    	break;
    }
}

#if defined(DEVMODE)
void seemative() { // "SEE" your primatives in action here!
   Serial2.print("\033[38;5;160m"); // T reddish
   switch (PA) {
      case 0x00:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println("key3");           
      break;
      case 0x01:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println("(emit3)"); 
      break;
      case 0x02:
         Serial2.println("noop"); 
      break;
      case 0x03:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("="); 
      break;
      case 0x04:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("-2"); 
      break;
      case 0x05:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("-1"); 
      break;
      case  0x06:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("2"); 
      break;
      case 0x07:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("1");  
      break;
      case 0x08:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("0"); 
      break;
      case 0x09:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("0="); 
      break;
      case 0x0A:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("c!"); 
      break;     
      case 0x0B:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("c@"); 
      break;  
      case 0x0C:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("!"); 
      break;  
      case 0x0D:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("@"); 
      break;  
      case 0x0E:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("rot"); 
      break; 
      case 0x0F: 
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("u/"); 
      break;
      case 0x10:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("u*"); 
      break;
      case 0x11:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("-"); 
      break;   
      case 0x12:   
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("+"); 
      break;
      case 0x13:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("over"); 
      break;
      case 0x14:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("swap"); 
      break;
      case 0x15:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("dup"); 
      break;
      case 0x16:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("(do)"); 
      break;
      case 0x17:
         Serial2.println("r>"); 
      break;
      case 0x18:
         Serial2.println(">r"); 
      break;
      case 0x19:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("branch"); 
      break;
      case 0x1A:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("0branch"); 
      break;
      case 0x1B:
         Serial2.print("\033[38;5;46m"); // T med green 
         Serial2.println("lit"); 
      break;
      case 0x1C:
         Serial2.println("DOCOL"); 
      break;
      case 0x1D:
         Serial2.println("SEMIS"); 
      break;
      case 0x1E:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("i"); 
      break;
      case 0x1F:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("j"); 
      break;
      case 0x20:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("k"); 
      break; 
      case 0x21:
         Serial2.println("r@"); 
      break;  
      case 0x22:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("drop"); 
      break;   
      case 0x23:
         Serial2.println("<p>"); 
      break;   
      case 0x24:
         Serial2.println("p!"); 
      break;   
      case 0x25:
         Serial2.println("p@"); 
      break;   
      case 0x26:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("hex."); 
      break;   
      case 0x27:
         Serial2.println("execute"); 
      break;   
      case 0x28:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("1+"); 
      break;   
      case 0x29:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("1-"); 
      break;   
      case 0x2A:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("2+"); 
      break;   
      case 0x2B:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("2-"); 
      break;   
      case 0x2C:
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("2drop"); 
      break;   
      case 0x2D:
         Serial2.print("\033[38;5;51m"); // T lite blue       
         dotsf();          // F.S
      break;
         Serial2.println("2*"); 
      break;   
      case 0x2E:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("2/"); 
      break;     
      case 0x2F:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("minus"); 
      break;
      case 0x30:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("dminus"); 
      break;  
      case 0x31:
         Serial2.print("\033[38;5;195m"); // T lte blue
         Serial2.println("or"); 
      break; 
      case 0x32:
         Serial2.print("\033[38;5;195m"); // T lte blue
         Serial2.println("and"); 
      break; 
      case 0x33:
         Serial2.print("\033[38;5;195m"); // T lte blue
         Serial2.println("xor"); 
      break;    
      case 0x34:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("s->d"); 
      break;               
      case 0x35:
         Serial2.println("millis"); 
      break; 
      case 0x36:
         Serial2.println("messages"); 
      break;  
      case 0x37:
         Serial2.println("?terminal"); 
      break;      
      case 0x38:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("0<"); 
      break;       
      case 0x39:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("(loop)"); 
      break;   
      case 0x3A:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("(+loop)"); 
      break;  
      case 0x3B:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("userx"); 
      break;  
      case 0x3C:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("leave"); 
      break; 
      case 0x3D:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("cmove"); 
      break; 
      case 0x3E:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println(".s"); 
      break; 
      case 0x3F:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println(".sl"); 
      break; 
      case 0x40:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println("dump"); 
      break; 
      case 0x41:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("user"); 
      break; 
      case 0x42:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("fill"); 
      break; 
      case 0x43:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println(">l"); 
      break; 
      case 0x44:
         Serial2.print("\033[38;5;129m"); // T purple 
         Serial2.println("l>"); 
      break;       
      case 0x45:
         Serial2.print("\033[38;5;231m"); // T white
         Serial2.println("expect"); 
      break; 
      case 0x46:
         Serial2.println("enclose"); 
      break; 
      case 0x47:
         Serial2.println("(find)"); 
      break; 
      case 0x48:
         Serial2.println("l@"); 
      break; 
      case 0x49:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("2!"); 
      break; 
      case 0x4A:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("digit"); 
      break; 
      case 0x4B:
         Serial2.println("toggle"); 
      break; 
      case 0x4C:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("<"); 
      break; 
      case 0x4D:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println(">"); 
      break; 
      case 0x4E:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("d+"); 
      break; 
      case 0x4F:
         Serial2.println("fname!"); 
      break; 
      case 0x50:
         Serial2.print("\033[38;5;51m"); // T lite blue 
         Serial2.println("+!"); 
      break;
      case 0x51:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("dovar"); 
      break;
      case 0x52:
         Serial2.print("\033[38;5;167m"); // T orange
         Serial2.println("docon"); 
      break;
      case 0x53:
         Serial2.println("cswap"); 
      break;
      case 0x54:
         Serial2.println("sp@"); 
      break;
      case 0x55:
         Serial2.println("sp!"); 
      break;
      case 0x56:
         Serial2.println("rp@"); 
      break;
      case 0x57:
         Serial2.println("rp!"); 
      break;
      case 0x58:      
         Serial2.print("\033[38;5;226m"); // T yellow
         Serial2.println("pick"); 
      break;
      case 0x59:
         Serial2.println("a@"); 
      break;
      case 0x5A:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("pupi"); 
      break;
      case 0x5B:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f."); 
      break;
      case 0x5C:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("D>f."); 
      break;
      case 0x5D:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f/"); 
      break;
      case 0x5E:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f>d"); 
      break;
      case 0x5F:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("s>f"); 
      break;
      case 0x60:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f>s"); 
      break;
      case 0x61:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fswap"); 
      break;
      case 0x62:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fdrop"); 
      break;
      case 0x63:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fdup"); 
      break;
      case 0x64:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("frot"); 
      break;      
      case 0x65:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f*"); 
      break;
      case 0x66:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f+"); 
      break;
      case 0x67:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f-"); 
      break;   
      case 0x68:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fsin"); 
      break;  
      case 0x69:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fcos"); 
      break;
      case 0x6A:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("ftan"); 
      break; 
      case 0x6B:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f.s"); 
      break; 
      case 0x6C:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fasin"); 
      break; 
      case 0x6D:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("facos"); 
      break; 
      case 0x6E:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fatan"); 
      break; 
      case 0x6F:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fatan2"); 
      break; 
      case 0x70:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fsqrt"); 
      break; 
      case 0x71:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fover"); 
      break; 
      case 0x72:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fpow"); 
      break; 
      case 0x73:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("fexp"); 
      break; 
      case 0x74:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f@"); 
      break;
      case 0x75:
         Serial2.print("\033[38;5;154m"); // T yellow green
         Serial2.println("f!"); 
      break; 
      case 0x76:
         Serial2.print("\033[38;5;46m"); // T med green 
         Serial2.println("dlit"); 
      break;
      case 0x77:
         Serial2.println("spaces"); 
      break;
      case 0x78:
         Serial2.println("type"); 
      break;
      case 0x79: // (.")
         Serial2.write(0x28); 
         Serial2.write(0x2E); 
         Serial2.write(0x22); 
         Serial2.write(0x29); 
         Serial2.write(10);
         Serial2.write(13);
      break;
      case 0x7A:
         Serial2.println("a!"); 
      break;
      case 0x7B:
         Serial2.println("dodoes>"); 
      break;
      case 0x7C:
         Serial2.println("fname."); 
      break;
      case 0x7D:
         Serial2.println("sdinit"); 
      break;
      case 0x7E:
         Serial2.println("fsize"); 
      break;
      case 0x7F:
         Serial2.println("fopen"); 
      break;
      case 0x80:
         Serial2.println("fclose"); 
      break;
      default:
         Serial2.println("\/\/\/\/\/\/\/\/"); // scream ERROR! outloud
            sPortSel = PORTMAIN;
            crlf();
            hexInt(PA);
         Serial3.println(" is an illegal instruction.");
      break;
   }
   Serial2.print("\033[38;5;231m"); // T white
   Serial2.write(10);
   Serial2.write(13);
}
#endif
void tasks() { // this is where activities that are happening 'autonomously' are executed
		if(millis() >= ledMil+ledMilDur) { // handle the heartbeat LED
		digitalWrite(ledPin, ledPinState);
		ledPinState = ~ledPinState;
		ledMil = millis();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////     ____     ___      _               _ __  __            _     _             __      /////
////      \ \ \   / (_)_ __| |_ _   _  __ _| |  \/  | __ _  ___| |__ (_)_ __   ___ / /       ////
////  _____\ \ \ / /| | '__| __| | | |/ _` | | |\/| |/ _` |/ __| '_ \| | '_ \ / _ / /_____   ////
//// |_____  /\ V / | | |  | |_| |_| | (_| | | |  | | (_| | (__| | | | | | | |  __\  _____|  ////
////      /_/  \_/  |_|_|   \__|\__,_|\__,_|_|_|  |_|\__,_|\___|_| |_|_|_| |_|\___|\_\       ////
/////                                                                                       /////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

// dodoe:
//    ld   hl,(rpp)  ; get the return stack pointer
//    dec   hl       ; make room on it
//    dec   hl       ;
//    ldw   (hl),bc  ; put the instruction pointer there        
//    ld   (rpp),hl  ; replace the return stack pointer
//-------------------------------------------------------
//    inc   de       ; increment WA
//    ex   de,hl     ; 
//    ldw   bc,(hl)  ; load the IP with the WA         
//    inc   hl       ; increment WA
//    inc   hl       ;
//    jhpush         ; and push that to the stack

void dodoes() {
   stackR[stackPointerR] = IP; // push IP to the return stack
   stackPointerR--;
      int tWA; // the Pseudo Word Address Register // IP <- @WA
   if ( WA < 0 ) { // the WA is in SRAM (the upper part of memory)
      tWA = WA & 0x3FFF;
      pivot.bivot[1] = pad[tWA]; // 
      tWA++; WA++; // increment pointer and pseudo-pointer synchronously
      pivot.bivot[0] = pad[tWA];
   }
   else { // the WA is in FLASH (the lower part of memory)
      pivot.bivot[1] = pgm_read_byte(&(body[WA])); // msB to accuA via WA
      WA++; // increment WA a second time only when you need the pointer for DOES> or VARIABLE etc...
      pivot.bivot[0] = pgm_read_byte(&(body[WA])); // lsb to CA via WA
   }
   WA++; // WA is now ready for use by DOCOL and other words (see above)
   IP = pivot.iivot[0];
   accuA = WA;
   pushA();
   next();   
}


// nest involves replacing the instruction pointer with the WA after pushing IP to @RS
void docol() { // @RP<--IP ; IP<--WA (which points to the next instruction in the thread)
#if defined(DEVMODE)
   Serial2.print("\033[38;5;160m"); // T reddish
   Serial2.write(0x44); // print an identifying 'D'
   Serial2.print("\033[38;5;231m"); // T white
   innerTest();
#endif
	stackR[stackPointerR] = IP;
   stackPointerR--;
	IP = WA;
	next();
}

// de-nest involves poping the return stack top into the instruction pointer
void semi() { // IP<--@RP ; RP++
#if defined(DEVMODE)
   Serial2.print("\033[38;5;160m"); // T reddish
   Serial2.write(0x53); // print an identifying 'S'
   Serial2.print("\033[38;5;231m"); // T white
   innerTest();
#endif
   stackPointerR++;
	IP = stackR[stackPointerR]; 
	next();
}

void next() { // WA<--@IP ; IP++
#if defined(DEVMODE)
      Serial2.print("\033[38;5;0m"); // T black
      Serial2.write(0x2E); // print an identifying "."
      Serial2.print("\033[38;5;231m"); // T white
      innerTest();
#endif
//////////////////////////////////////WA<--@IP; IP++///////////////////////////////////////////
	int tIP; // the psuedo Instruction Pointer
	if ( IP < 0 ) { // the IP is in SRAM (the upper part of memory)
      tIP = IP & 0x3FFF;
      pivot.bivot[1] = pad[tIP]; // the hi-byte
      tIP++; IP++; // synchrnonusly increment both the IP and its proxy
      pivot.bivot[0] = pad[tIP]; // the lo-byte
	}
	else {
      pivot.bivot[1] = pgm_read_byte(&(body[IP])); // msB to accA via IP
      IP++;
      pivot.bivot[0] = pgm_read_byte(&(body[IP])); // lsB to accA via IP
	}
	IP++; // IP is now pointing to the next instruction
   WA = pivot.iivot[0];
#if defined(DEVMODE)
   Serial2.print("\033[38;5;0m"); // T black
   Serial2.write(0x2E); // print an identifying "."
   Serial2.print("\033[38;5;231m"); // T white
   innerTest();
#endif
//////////////////////////////////////CA<--@WP; WP++///////////////////////////////////////////
	int tWA; // the Pseudo Word Address Register
	if ( WA < 0 ) { // the WA is in SRAM (the upper part of memory)
      tWA = WA & 0x3FFF;
      pivot.bivot[1] = pad[tWA]; // 
      tWA++; WA++; // increment pointer and pseudo-pointer synchronously
      pivot.bivot[0] = pad[tWA];
	}
	else { // the WA is in FLASH (the lower part of memory)
      pivot.bivot[1] = pgm_read_byte(&(body[WA])); // msB to accuA via WA
      WA++; // increment WA a second time only when you need the pointer for DOES> or VARIABLE etc...
      pivot.bivot[0] = pgm_read_byte(&(body[WA])); // lsb to CA via WA
	}
	WA++; // WA is now ready for use by DOCOL and other words (see above)
   CA = pivot.iivot[0];
#if defined(DEVMODE)
   Serial2.print("\033[38;5;0m"); // T black
   Serial2.write(0x2E); // print an identifying "."
   Serial2.print("\033[38;5;231m"); // T white
   innerTest();
#endif
//////////////////////////////////////PA<--@CA; CA++///////////////////////////////////////////
	int tCA; // the Pseudo Code Address Register
	if ( CA < 0 ) { // the CA is in SRAM (the upper part of memory)
      tCA = CA & 0x3FFF;
      pivot.bivot[1] = pad[tCA]; // 
      tCA++; CA++; // increment pointer and pseudo-pointer synchronously
      pivot.bivot[0] = pad[tCA];
	}
	else { // the CA is in FLASH (the lower part of memory)
      pivot.bivot[1] = pgm_read_byte(&(body[CA])); // msB to accuA via WA
      CA++;
      pivot.bivot[0] = pgm_read_byte(&(body[CA])); // lsb to CA via WA
	}
   PA = pivot.iivot[0];
#if defined(DEVMODE)
   Serial2.print("\033[38;5;0m"); // T black
   Serial2.write(0x2E); // print an identifying "."
   Serial2.print("\033[38;5;231m"); // T white
	innerTest();
#endif
	primative();

}
// This prints out all relevant pointers in the inner interpreter
// Eventually this will be output on another monitor through 
// another serial port



void hexNybble(int hnib) { // output a single hex digit
	hnib &= 0x0F;
   switch (sPortSel) {
   case 0x00:
      Serial.print(hnib, HEX); // onboard
   break;
   case 0x01:
      Serial1.print(hnib, HEX); // DYNAMIXEL
   break;
   case 0x02:
      Serial2.print(hnib, HEX); // debug
   break;
   case 0x03:
      Serial3.print(hnib, HEX); // main
   break;
	}
}

void hexByte(int hbyte) { // output a hex byte
   int i;
   for (i=0; i<2; i++) {
      accuA = hbyte;
      accuA = accuA >> ((1-i)*4);
      hexNybble(accuA);
   }
}

void hexInt(int hword) { // output a complete 4-digit int
	int i;
	for (i=0; i<4; i++) {
		accuA = hword;
		accuA = accuA >> ((3-i)*4);
		hexNybble(accuA);
	}

}

void hexIntFig() { // use the routine above with TOS (Top Of Stack)
   sPortSel = PORTMAIN;
	popA();
	hexInt(accuA);
}

int innerTestCounter = 0;
int scrap = 0x30;

void innerTest() { // output each of the relevant pointers in the Inner Interpreter
   sPortSel = PORTDEBUG; // select Serial2
   //Serial2.write(0x0D);
   hexInt(innerTestCounter);
   Serial2.write(0x20);
   Serial2.write(0x20);
   if (scrap == 0x30) {
	   Serial2.print("IP = ");
	   hexInt(IP);
	   Serial2.write(0x20);
      Serial2.write(0x20);
	   Serial2.print("WA = ");
	   hexInt(WA);
	   Serial2.write(0x20);
      Serial2.write(0x20);
	   Serial2.print("CA = ");
	   hexInt(CA);
	   Serial2.write(0x20);
      Serial2.write(0x20);
      Serial2.print("\033[38;5;213m"); // T pinkish
	   Serial2.print("PA = ");
	   hexInt(PA);
      Serial2.print("\033[38;5;231m"); // T white
      Serial2.write(0x20);
      Serial2.write(0x20);
      Serial2.print("PS = ");
      hexInt(stackPointer);
      Serial2.write(0x20);
      Serial2.write(0x20);
	   Serial2.print("RS = ");
      hexInt(stackPointerR);
      Serial2.write(0x20);
      Serial2.write(0x20);
      Serial2.print("LS = ");
      hexInt(stackPointerL);                           
      Serial2.write(0x20);
      Serial2.write(0x20);
   }
   Serial2.print("\033[38;5;118m"); // T lite green
   Serial2.print("TOS> ");
   Serial2.print("\033[38;5;228m"); // T lite yellow
   dots();
   Serial2.print("\033[38;5;118m"); // T lite green
   Serial2.print("<NOS");
   Serial2.print("\033[38;5;231m"); // T white
   Serial2.println(" ");
   innerTestCounter++;
   if (Serial2.available() != 0) {
      scrap = Serial2.read();
   }
}

void crlf() {
   Serial3.write(10);
   Serial3.write(13);
}

void dumpDot(int prascii) { // prep-ascii only prints ascii in the range of 0x20 (space) 
   if ((prascii < 0x020) | (prascii > 0x07E)) { // to 0x7E ( ~ ). The rest print ( . )
      prascii = 0x02E; // ASCII period or dot
   }
   Serial3.write(prascii);
}

// a dump to observe the contents of the memory
// FLASH  = 0000h ~ 3FFFh   00111111-11111111   The upper two bits are the bank identifiers
// SRAM   = 8000h ~ 93FFh   10010011-11111111
// EEPROM = F000h ~ FFFFh   11111111-11111111 
void SRAMdump() { // DUMP
   //popC(); // the count
   popA(); // the address
   sPortSel = PORTMAIN; // select the main serial port
   int i = 0;
   int j = 0;
   int k = 0;
   int l = 0;
   int m = 0; // the bank mask indicator will be used to select FLASH, SRAM, or EEPROM
   int n = accuA & 0xFFF0;
   m = accuA & 0xC000; // save the mask bit
   l = accuA & 0x3FF0; // and form the address used for accessing the memory banks
   accuA &= 0xFFF0;
   crlf();
   Serial3.print("\033[38;5;118m"); // T lite green
   Serial3.print("  Addr 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F - 0123456789ABCDEF");
   switch (m) {
      case 0x0000:
         Serial3.print("\033[38;5;184m"); // T yellowish
         Serial3.println(" FLASH");
         Serial3.print("\033[38;5;231m"); // T white
      break;
      case 0x4000:
         Serial3.println(" FOOL'S GOLD");
         Serial3.print("\033[38;5;231m"); // T white
      break;
      case 0x8000:
         Serial3.print("\033[38;5;48m"); // T aqua 
         Serial3.println(" SRAM");
         Serial3.print("\033[38;5;231m"); // T white
      break;
      case 0xC000:
         Serial3.print("\033[38;5;200m"); // T hot pink
         Serial3.println(" EEPROM");
         Serial3.print("\033[38;5;231m"); // T white
      break;
   }
   Serial3.print("\033[38;5;106m"); // T muted green
   Serial3.println("   |    |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   ||||||||||||||||");
   Serial3.print("\033[38;5;231m"); // T white
   for(i = 0; i < 16; i++) {
      Serial3.print("\033[38;5;118m"); // T lite green
      hexInt(i*16+n); // print the address at the beginning of the row
      tasks(); // check for the tasks that need refreshing as this can be a long routine
      Serial3.write(0x20); // space
      Serial3.print("\033[38;5;106m"); // T muted green
      Serial3.print("- "); // and hyphen
      Serial3.print("\033[38;5;231m"); // T white
      for(j = 0; j < 16; j++) { // then the data on that line
         switch (m) {
            case 0x0000:
               k = pgm_read_byte(&(body[j+i*16+l])); // FLASH
            break;
            case 0x4000:
               k = 0x0AA;                            // void
            break;
            case 0x8000:
               k = pad[j+i*16+l];                    // SRAM
            break;
            case 0xC000:
               k = EEPROM.read(j+i*16+l);            // EEPROM
            break;
         }
         
         hexByte(k); // print the byte 
         Serial3.write(0x20); // and the space preceeding the next piece of data
      }
      Serial3.print("\033[38;5;106m"); // T muted green
      Serial3.print("- "); // prepare to print the character equivalent of the line
      Serial3.print("\033[38;5;231m"); // T white
      for(j = 0; j < 16; j++) {
         switch (m) {
            case 0x0000:
               k = pgm_read_byte(&(body[j+i*16+l])); // FLASH
            break;
            case 0x4000:
               k = 0x0AA;                            // void
            break;
            case 0x8000:
               k = pad[j+i*16+l];                    // SRAM
            break;
            case 0xC000:
               k = EEPROM.read(j+i*16+l);            // EEPROM
            break;
         }
         dumpDot(k);
      }
      crlf();
   }
}


// this is a simple hex output version of .S for debugging
void dots() { // .S pictured numeric output of the stack contents
   sPortSel = PORTDEBUG; // select the main serial port
   int i;
   if (PSTACKZED - stackPointer != 0) {
         for ( i = stackPointer; i < PSTACKZED; i++) {
            //crlf();
            //hexByte(i+1);
            //Serial.write(0x020);
            accuD = stack[i+1];
            hexInt(accuD);
            Serial2.write(0x20);
      }
   }
}

// this is a simple hex output version of .S
void mdots() { // .S pictured numeric output of the stack contents
   sPortSel = PORTMAIN; // select the main serial port
   Serial3.print("\033[38;5;51m"); // T lite blue 
   Serial3.print("TOS> ");
   Serial3.print("\033[38;5;228m"); // T lite yellow 
   int i;
   if (PSTACKZED - stackPointer != 0) {
         for ( i = stackPointer; i < PSTACKZED; i++) {
            //crlf();
            //hexByte(i+1);
            //Serial.write(0x020);
            accuD = stack[i+1];
            hexInt(accuD);
            Serial3.write(0x20);
      }
   }
   Serial3.print("\033[38;5;51m"); // T lite blue 
   Serial3.print("<NOS ");
   Serial3.print("\033[38;5;231m"); // T white
}

// this is a simple floating point version of .S
void dotsf() { // .S pictured numeric output of the FP stack contents
   sPortSel = PORTDEBUG; // select the main serial port
   Serial3.print("\033[38;5;118m"); // T lite green
   Serial3.print("TOS> ");
   Serial3.print("\033[38;5;228m"); // T lite yellow
   int i;
   if (FSTACKZED - stackPointerF != 0) {
         for ( i = stackPointerF; i < FSTACKZED; i++) {
            //crlf();
            //hexByte(i+1);
            //Serial.write(0x020);
            Serial3.write(0x20);
            accuDf = stackF[i+1];
            dtostrf(accuDf, 10, 7, foutstr);
            Serial3.print(foutstr);
            Serial2.write(0x20);
      }
   }
   Serial3.print("\033[38;5;118m"); // T lite green
   Serial3.print(" <NOS");
   Serial3.print("\033[38;5;231m"); // T white
   Serial3.println(" ");
}

void dotsL() { // .S pictured numeric output of the stack contents
   sPortSel = PORTMAIN; // select the main serial port
   int i;
   if (LSTACKZED - stackPointerL != 0) {
         for ( i = stackPointerL; i < LSTACKZED; i++) {
            //crlf();
            //hexByte(i+1);
            //Serial.write(0x020);
            accuD = stackF[i+1];
            hexInt(accuD);
      }
      crlf();
   }
}

#if defined(DEVMODE)
void accuVu() { // look at the contents of all of the accumulators
   sPortSel = PORTDEBUG; // select Serial2
   pushA(); pushB(); pushC(); pushD(); // stack all of the accumulators
   Serial2.write(0x0D);
   Serial2.print("accuA = ");
   hexInt(accuA);
   Serial2.write(0x09);
   Serial2.print("accuB = ");
   hexInt(accuB);
   Serial2.write(0x09);
   Serial2.print("accuC = ");
   hexInt(accuC);
   Serial2.write(0x09);
   Serial2.print("accuD = ");
   hexInt(accuD);
   Serial2.println(" ");
   popD(); popC(); popB(); popA(); // return the state of all of the accumulators
}
#endif
/*        
   This is the map of a typical header.

         |------------+
n--------+  3 + 0x80  |<----+
         |------------+     |
n +  1---+      L     |     |
         |------------+     |
n +  2---+      I     |     |
         |------------+     |
n +  3---+      T     |     |
         |------------+     |      
n +  4---+     LFA    |     |      is '0' since this is the first word in the list
         |------------+     |  
n +  5---+     CFA    |---+ |      points to the next word in the list in the case of a primative
         |------------+   | |
n +  6---+Instruction |<--+ |      PFA
         |------------+     |
n +  7---+    7+80h   |<----+-+
         |------------+     | |
n +  8---+      E     |     | |
         |------------+     | |
n +  9---+      X     |     | |
         |------------+     | |
n + 10---+      E     |     | |
         |------------+     | |
n + 11---+      C     |     | |
         |------------+     | |
n + 12---+      U     |     | |
         |------------+     | |
n + 13---+      T     |     | |
         |------------+     | |
n + 14---+      E     |     | |
         |------------+     | |
n + 15---+     LFA    |-----+ |
         |------------+       | 
n + 16---+     CFA    |---+   |
         |------------+   |   | 
n + 17---+    6+80h   |<--+   |    PFA
         |------------+       |
n + 18---+      B     |       |
         |------------+       |
n + 19---+      R     |       |
         |------------+       |
n + 20---+      A     |       |
         |------------+       |
n + 21---+      N     |       |
         |------------+       |
n + 22---+      C     |       |
         |------------+       |
n + 23---+      H     |       |        
         |------------+       |       
n + 24---+     LFA    |<------+
         |------------+
n + 25---+     CFA    | <---------------In the case of a hi-level definition this address points to docol, the nexting code
         |------------+
n + 26---+            |           PFA etc...
         |------------+
n + 27---+            |
         |------------+
n + 28---+            |
         |------------+
n + 29---+            |
         |------------+
n + 30---+

I am at a point where I have to decide what I am going to do about the problem of there being
two memory spaces where programs are stored. The base program will be stored in PROGMEM with 
its generous 256kB storage space. Since it's not writable the user's program will have to be 
stored in SRAM. How do I distinguish between these two places? Solution used thus far: to set
the msb of the address to '1' to indicate that it does not indicate FLASH. That leaves the 
method of incorporating the EEPROM space to resolve. Tentatively I want to place it either 
at the end of the SRAM or at the very top of the addressable 64k address space.

   I am leaning toward letting the MSB of the address word being the one that makes the dis-
tinction. To illustrate: imagine that C@ has the following data 0x8000 C@. The HIGH msb
means that the 0xAA will be fetched from the first location in SRAM. This proposed format will
allow the maintainence of two 32kB memory spaces. That is more than I actually have access to
in the ATMega256.

   In doing this I have to be mindful of the fact that the set msb is not actually used to 
access the memory. It must be masked out when the actual table addresses are used. 
kjc 12aug2017 

   NOTE: the compiler sign extends the msb when right shifted. In other words, in an integer,
10000000-00000000 shifted right one bit becomes
11000000-00000000
kjc 21aug2017

   I got the SD card, the one on the ethernet shield, working with the MEGA2560. I have to do
some research on string functions in C, C++, to pass arguments to the SD functions as variables.
For the time being I am going to "hardwire" the open file and ancillary functions by coding the
name of a single file as being the only one I am using. 
kjc 22aug2017

   I think that I have written all of the words that will be required to implement a file system
and editor based on an SD card. I am keeping all of the variables in the user variable area.
So far this will be ROW and COL.
kjc 23aug2017

   In this model the return stack and the loop stack are two different stacks. As a consequence
the fig-Forth words that try to get the loop index off of the return stack will have to be re-
written. So far the word that I have to rewrite is 'EXPECT'.

   Put an LED on pin 52 (the SD chip select) to distinguish it from among other possible SPI
devices that may eventually be attached to the system.

   Routines to remove:
      pdotq
      everything that doesn't have vectored emit or, eventually, key...
   kjc 20170930
*/

