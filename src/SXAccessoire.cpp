/*
 * SXAccessoire.cpp
 *
 *  Version:    3.3
 *  Copyright:  Gerard van der Sel
 *
 *  Changed on: 25.10.2023
 *  Version: 	3.3
 *  Changes: 	Made ISR all assembly to obtain highest speed (uses less clcokcycles)
 *
 *  Changed on: 16.06.2022
 *  Version: 	3.2
 *  Changes: 	Pin configurable from outside the class
 *              Made check if programming is activated on the SXbus
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Added 3 and 4 pin interface.
 *
 *  Changed on: 19-12.2015
 *  Version: 	3.0
 *  Changes: 	Added some comment. Given its version number.
 *
 *  Changed on: 30.11.2015
 *  Version: 	0.5
 *  Changes: 	Reading and writing to multiple addresses in one cycle, resolved timing issues.
 *
 *  Changed on: 14.11.2015
 *  Version: 	0.4
 *  Changes: 	Reading and writing to multiple addresses in one cycle.
 *
 *  Changed on: 27.10.2015
 *  Version: 	0.3
 *  Changes: 	onWait() added to synchronise with the SXbus.
 *
 *  Changed on: 27.09.2015
 *  Version: 	0.2
 *  Changes: 	Minor changes
 *
 *  Changed on: 10.07.2015
 *  Version: 	0.1
 *  Changes: 	Initial version
 *
 *  interface hardware needed !

 Interface SX-bus
 - SX T0 (Clock) must be connected to Pin 2 or 3 (INT0, INT1)
 - SX T1 must be connected to any IO pin
 - SX D must be connected to any IO pin

 SX-bus interface (NEM 682)

The interface is optimised in time. It is written as fast as posible
  to let the Arduino perform other tasks in the forground.
  Watch out when changing the code and check for performance in time.
  For the AVR family there is an assembly version which runs on average twice as fast

De clock lijn (T0) is verbonden met een interruptingang, zodat op
 de flanken van dit signaal een interrupt gegenereerd kan worden.
 Hierna kan data gelezen worden van T1 of data geschreven worden naar D.

 Klok:
  --    ----------------    ----------------    ----------------    ------
	|  |                |  |                |  |                |  |
	 --                  --                  --                  --

 Data:
  -- ------------------- ------------------- ------------------- ---------
	X                   X                   X                   X
  -- ------------------- ------------------- ------------------- ---------

	   ^                   ^                   ^                   ^
	   P                   P                   P                   P

Opbouw telegram (96 bits):
  0  0 0  1  S 1 A3 A2 1 A1 A0 1     synchronisatie 'byte'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         7 data 'bytes'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1   ieder 'byte' is de inhoud
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         van een adres
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1

 0 = Logische 0
 1 = Logische 1
 S = Spanning rails (0 = uit, 1= aan)
 Ax = Gezamelijk het nummer van het telegram
 Dx = D0 t/m D7 vormen de data op een Selectrix adres.

 Verdeling adressen over de verschillende telegrammen:
 telegram  '0' : 111,  95,  79,  63,  47,  31,  15
 telegram  '1' : 110,  94,  78,  62,  46,  30,  14
 telegram  '2' : 109,  93,  77,  61,  45,  29,  13
 telegram  '3' : 108,  92,  76,  60,  44,  28,  12
 telegram  '4' : 107,  91,  75,  59,  43,  27,  11
 telegram  '5' : 106,  90,  74,  58,  42,  26,  10
 telegram  '6' : 105,  89,  73,  57,  41,  25,   9
 telegram  '7' : 104,  88,  72,  56,  40,  24,   8
 telegram  '8' : 103,  87,  71,  55,  39,  23,   7
 telegram  '9' : 102,  86,  70,  54,  38,  22,   6
 telegram '10' : 101,  85,  69,  53,  37,  21,   5
 telegram '11' : 100,  84,  68,  52,  36,  20,   4
 telegram '12' :  99,  83,  67,  51,  35,  19,   3
 telegram '13' :  98,  82,  66,  50,  34,  18,   2
 telegram '14' :  97,  81,  65,  49,  33,  17,   1
 telegram '15' :  96,  80,  64,  48,  32,  16,   0

This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#include <Arduino.h> 

#include "SXAccessoire.h"

SXAccessoire::SXAccessoire(uint8_t SX_T0_PIN, uint8_t SX_T1_PIN, uint8_t SX_D_PIN) {
	bitClear(SXflags, SXPINS);
	if ((SX_T0_PIN == 2) || (SX_T0_PIN == 3)) {
		// For the 3 line interface
		bitClear(SXflags, SX4LINE);
		pinMode(SX_T0_PIN, INPUT);                // SX-T0 is an input, no pull up
		pinMode(SX_T1_PIN, INPUT);                // SX-T1 is also an input, no pull up
		pinMode(SX_D_PIN, INPUT);                 // SX-D is also an input when not writing to allow other devices to write

		// initialize pin variables (Memorymapped IO)
		// Clock (T0)
		// SX_T0_MASK = digitalPinToBitMask(SX_T0_PIN);
		// SX_T0_OUT = portOutputRegister(digitalPinToPort(SX_T0_PIN));
		// SX_T0_IN = portInputRegister(digitalPinToPort(SX_T0_PIN));
		// SX_T0_DIR = portModeRegister(digitalPinToPort(SX_T0_PIN));
		// Data in (T1)
		SX_T1_MASK = digitalPinToBitMask(SX_T1_PIN);
		// SX_T1_OUT = portOutputRegister(digitalPinToPort(SX_T1_PIN));
		SX_T1_IN = portInputRegister(digitalPinToPort(SX_T1_PIN));
		// SX_T1_DIR = portModeRegister(digitalPinToPort(SX_T1_PIN));
		// Data out (D)
		SX_D_MASK = digitalPinToBitMask(SX_D_PIN);
		SX_D_OUT = portOutputRegister(digitalPinToPort(SX_D_PIN));
		// SX_D_IN = portInputRegister(digitalPinToPort(SX_D_PIN));
		SX_D_DIR = portModeRegister(digitalPinToPort(SX_D_PIN));
		// Check for valid pins
		bitWrite(SXflags, SXPINS, ((digitalPinToPort(SX_T1_PIN) != NOT_A_PIN) &&
			                         (digitalPinToPort(SX_D_PIN) != NOT_A_PIN)));
	}
}

SXAccessoire::SXAccessoire(uint8_t SX_T0_PIN, uint8_t SX_T1_PIN, uint8_t SX_D_LOW_PIN, uint8_t SX_D_HIGH_PIN) {
	bitClear(SXflags, SXPINS);
	if ((SX_T0_PIN == 2) || (SX_T0_PIN == 3)) {
		// For the 4 line interface
		bitSet(SXflags, SX4LINE);
		pinMode(SX_T0_PIN, INPUT_PULLUP);         // SX-T0 is an input,pull  up
		pinMode(SX_T1_PIN, INPUT_PULLUP);         // SX-T1 is also an input, pull up
		digitalWrite(SX_D_LOW_PIN, HIGH);
		pinMode(SX_D_LOW_PIN, OUTPUT);            // SX_D_LOW is output but set high to stop wrting low
		digitalWrite(SX_D_HIGH_PIN, HIGH);
		pinMode(SX_D_HIGH_PIN, OUTPUT);           // SX_D_HIGH is output but set high to stop wrting high

		// initialize pin variables (Memorymapped IO)
		// Clock (T0)
		// SX_T0_MASK = digitalPinToBitMask(SX_T0_PIN);
		// SX_T0_OUT = portOutputRegister(digitalPinToPort(SX_T0_PIN));
		// SX_T0_IN = portInputRegister(digitalPinToPort(SX_T0_PIN));
		// SX_T0_DIR = portModeRegister(digitalPinToPort(SX_T0_PIN));
		// Data in (T1)
		SX_T1_MASK = digitalPinToBitMask(SX_T1_PIN);
		// SX_T1_OUT = portOutputRegister(digitalPinToPort(SX_T1_PIN));
		SX_T1_IN = portInputRegister(digitalPinToPort(SX_T1_PIN));
		// SX_T1_DIR = portModeRegister(digitalPinToPort(SX_T1_PIN));
		// Data out (D)
		SX_D_LOW_MASK = digitalPinToBitMask(SX_D_LOW_PIN);
		SX_D_LOW_OUT = portOutputRegister(digitalPinToPort(SX_D_LOW_PIN));
		// SX_D_LOW_IN = portInputRegister(digitalPinToPort(SX_D_LOW_PIN));
		// SX_D_LOW_DIR = portModeRegister(digitalPinToPort(SX_D_LOW_PIN));
		SX_D_HIGH_MASK = digitalPinToBitMask(SX_D_HIGH_PIN);
		SX_D_HIGH_OUT = portOutputRegister(digitalPinToPort(SX_D_HIGH_PIN));
		// SX_D_HIGH_IN = portInputRegister(digitalPinToPort(SX_D_HIGH_PIN));
		// SX_D_HIGH_DIR = portModeRegister(digitalPinToPort(SX_D_HIGH_PIN));
		// Check for valid pins
		bitWrite(SXflags, SXPINS, ((digitalPinToPort(SX_T1_PIN) != NOT_A_PIN) &&
			                         (digitalPinToPort(SX_D_LOW_PIN) != NOT_A_PIN) &&
			                         (digitalPinToPort(SX_D_HIGH_PIN) != NOT_A_PIN)));
	}
}

// initialize function
bool SXAccessoire::init() {
	// initialize data array
	for (int i = 0; i < SX_ADDRESS_NUMBER; i++) {
		SXarray[i] = 0;                             // set sx data to 0
	}
	// initialize variables
	initVars();                                 // Start looking for SYNC
	return bitRead(SXflags, SXPINS);;
}

void SXAccessoire::initVars() {
	// start always with search for header
	SXstate = SYNC;                             // First look for SYNC pattern
	SXsepCount = SX_SEPLENGTH;                  // Distanse between two separators
	SXbyteCount = SX_STOP;                      // Check for SX_STOP bits of "0"
	// no writing to the SX bus
	SXflags &= 0xF1;                            // Reset flags
	// Powerbit send and receive
	SXnewPWR = PWR_SEND;                        // Don't write power bit
}

#ifdef __AVR__
// Interrupt service routine (AVR INT0/INT1)
// Driven by RISING EDGES of the SX clock signal T0 (SX pin 1)
// This code is execute every 50 usec (keep it as fast as possible)
/*
 * Saved registers: psw, r0, r1, r18, r19, r20, r21, r22, r23, r24, r25, r26, r27, r30 en r31 
 * Used registers:  psw,                   r20, r21, r22, r23, r24, r25,           r30 en r31
 * r30 en r31 = pointer to mmio or sxbus data
 * r25 = work 2
 * r24 = work 1
 * r23 = SXflags
 * r22 = SXbyteCount
 * r21 = SXsepCount
 * r20 = SXshift
 */

void SXAccessoire::isr(void) {
ISRInit:
  asm goto (
  "   lds   r23, %[SXflgs]   ; Initialise common registers                  \n\t"
  "   lds   r22, %[SXbtcnt]                                                 \n\t"
  "   lds   r21, %[SXspcnt]                                                 \n\t"
  "   lds   r20, %[SXshft]                                                  \n\t"
  "   sbrs  r23, %[SXWRTNG]  ; Check if writing                             \n\t"
  "   rcall %l[ReadT1]       ; Read data from SXbus                         \n\t"
  "   lds   r24, %[SXstat]   ; Determine step to perform                    \n\t"
  "   cpi   r24, %[SNC]                                                     \n\t"
  "   brcs  %l[ISRData]      ; Read/write data from/to SXbus                \n\t"
  "   breq  %l[ISRSync]      ; Read sync from SX-bus                        \n\t"
  "   cpi   r24, %[ADR]                                                     \n\t"
  "   brcs  ISRJmpPwr        ; Read/write power from/to SX-bus              \n\t"
  "   breq  ISRJmpAdr        ; Read address from SX-bus                     \n\t"
  "ISRError:                                                                \n\t"
  "   ldi   r24, %[TRISTATE] ; Set outputs to 3 state (no write)            \n\t"
  "   rcall %l[WriteD]                                                      \n\t"
  "   andi  r23, 0x0F1       ; Status error, restart                        \n\t"
  "   sts   %[SXflgs], r23                                                  \n\t"
  "   sts   %[SXshft], r20                                                  \n\t"
  "   ldi   r21, %[SXSTP]                                                   \n\t"
  "   sts   %[SXspcnt], r21                                                 \n\t"
  "   ldi   r24, %[SNC]                                                     \n\t"
  "   sts   %[SXstat], r24                                                  \n\t"
  "   ldi   r24, %[PWRSEND]                                                 \n\t"
  "   sts   %[SXnwpwr], r24                                                 \n\t"
  "   ldi   r24, %[SXSTP]                                                   \n\t"
  "   sts   %[SXbtcnt], r24                                                 \n\t"
  "   ret                                                                   \n\t"
  "ISRJmpPwr:                                                               \n\t"
  "   rjmp  %l[ISRPwr]       ; Jump to power step                           \n\t"
  "ISRJmpAdr:                                                               \n\t"
  "   rjmp  %l[ISRAddr]      ; Jump to address step                         \n\t"
   :
   : [SNC] "M" (SYNC),
     [ADR] "M" (ADDR),
     [SXSTP] "M" (SX_STOP),
     [SXWRTNG] "M" (SXWRITING),
     [PWRSEND] "M" (PWR_SEND),
     [TRISTATE] "M" (TRI_STATE),
     [SXflgs] "i" (&SXflags),
     [SXstat] "i" (&SXstate),
     [SXnwpwr] "i" (&SXnewPWR),
     [SXshft] "i" (&SXshift),
     [SXbtcnt] "i" (&SXbyteCount), 
     [SXspcnt] "i" (&SXsepCount)
   :
   : WriteD, ReadT1, ISRSync, ISRPwr, ISRAddr, ISRData
  );
ISRSync:
  asm volatile (
  "   sbrc  r23, %[SXBT]     ; Check bit from SXbus                         \n\t"
  "   rjmp  ISRSync10                                                       \n\t"
  "   or    r22, r22         ; Bit low, 3 counted for                       \n\t"
  "   breq  ISRSync30        ; Yes, wait for a high bit                     \n\t"
  "   dec   r22              ; No, lower count                              \n\t"
  "   rjmp  ISRSync30                                                       \n\t"
  "ISRSync10:                                                               \n\t"
  "   or    r22, r22         ; Bit high, 3 bits low before it               \n\t"
  "   breq  ISRSync20        ; Yes, to next step                            \n\t"
  "   andi  r23, 0x0F1       ; No, reset flags                              \n\t"
  "   ldi   r21, %[SXSTP]    ; start over looking for 3 times low           \n\t"
  "   rjmp  ISRSync30                                                       \n\t"
  "ISRSync20:                                                               \n\t"
  "   ldi   r24, %[PW]       ; Set next step (power)                        \n\t"
  "   sts   %[SXstat], r24                                                  \n\t"
  "   ldi   r21, %[SXSEPLEN] - 1 ; Only 2 bits                              \n\t"
  "ISRSync30:                                                               \n\t"
  "   sts   %[SXshft], r20   ; Save common registers                        \n\t"
  "   sts   %[SXspcnt], r21                                                 \n\t"
  "   sts   %[SXbtcnt], r22                                                 \n\t"
  "   sts   %[SXflgs], r23                                                  \n\t"
  "   ret                                                                   \n\t"
   :
   : [PW] "M" (PWR),
     [SXSEPLEN] "M" (SX_SEPLENGTH),
     [SXBT] "M" (SXBIT),
     [SXSTP] "M" (SX_STOP),
     [SXstat] "i" (&SXstate),
     [SXflgs] "i" (&SXflags),
     [SXshft] "i" (&SXshift),
     [SXbtcnt] "i" (&SXbyteCount),
     [SXspcnt] "i" (&SXsepCount)
   :
  );
ISRData:
  asm goto (
  "   dec   r21              ; Reached seperator?                           \n\t"
  "   brne  ISRdata10                                                       \n\t"
  "   ldi   r24, %[TRISTATE] ; Set outputs to 3 state (no write)            \n\t"
  "   rcall %l[WriteD]                                                      \n\t"
  "   ldi   r21, %[SXSEPLEN] ; Reset separator counter                      \n\t"
  "   rjmp  ISRdata30                                                       \n\t"
  "ISRdata10:                                                               \n\t"
  "   ror   20               ; Bit to send in cy, bit 7 free                \n\t"
  "   sbrs  r23, %[SXWRTNG]  ; Are we writing?                              \n\t"
  "   rjmp  ISRdata20                                                       \n\t"
  "   sbc   r24, r24         ; Subtract CY to get -1 or 0                   \n\t"
  "   neg   r24              ; Negate to get 1 or 0                         \n\t"
  "   rcall %l[WriteD]       ; Write it                                     \n\t"
  "   rjmp  ISRdata30                                                       \n\t"
  "ISRdata20:                                                               \n\t"
  "   bst   r23, %[SXBT]     ; Get saved bit and                            \n\t"
  "   bld   r20, 7           ; place it in bit 7                            \n\t"
  "ISRdata30:                                                               \n\t"
  "   dec   r22              ; All bits done?                               \n\t"
  "   brne  ISRdata70        ; No, leave                                    \n\t"
  "   ldi   r22, %[SXBTLEN]                                                 \n\t"
  "   lds   ZL, %[SXidx]     ; Load array address data SXbus                \n\t"
  "   lds   ZH, %[SXidx] + 1                                                \n\t"
  "   ldd   r24, Z + 1       ; Check if current data must be                \n\t"
  "   andi  r24, 0x01        ; written                                      \n\t"
  "   brne  ISRdata40        ; Yes, don't save read data                    \n\t"
  "   bst   r23, %[SXWRTNG]  ; Was last byte written                        \n\t"
  "   brts  ISRdata40        ; Yes, don't save read data                    \n\t"
  "   st    Z, r20           ; Save read data                               \n\t"
  "ISRdata40:                                                               \n\t"
  "   mov   r20, __zero_reg__                                               \n\t"
  "   adiw  ZL, 2                                                           \n\t"
  "   sts   %[SXidx], ZL                                                    \n\t"
  "   sts   %[SXidx] + 1, ZH                                                \n\t"
  "   lds   r25, %[SXfrmcnt]                                                \n\t"
  "   clt                                                                   \n\t"
  "   dec   r25                                                             \n\t"
  "   brne  ISRdata50                                                       \n\t"
  "   ldi   r22, %[SXSTP]                                                   \n\t"
  "   ldi   r24, %[SNC]                                                     \n\t"
  "   sts   %[SXstat], r24                                                  \n\t"
  "   rjmp  ISRdata60                                                       \n\t"
  "ISRdata50:                                                               \n\t"
  "   ldd   r24, Z + 1                                                      \n\t"
  "   andi  r24, 0x01                                                       \n\t"
  "   breq  ISRdata60                                                       \n\t"
  "   ld    r20, Z                                                          \n\t"
  "   std   Z + 1, __zero_reg__                                             \n\t"
  "   set                                                                   \n\t"
  "ISRdata60:                                                               \n\t"
  "   bld   r23, %[SXWRTNG]                                                 \n\t"
  "   sts   %[SXfrmcnt], r25                                                \n\t"
  "ISRdata70:                                                               \n\t"
  "   sts   %[SXshft], r20   ; Save common registers                        \n\t"
  "   sts   %[SXspcnt], r21                                                 \n\t"
  "   sts   %[SXbtcnt], r22                                                 \n\t"
  "   sts   %[SXflgs], r23                                                  \n\t"
  "   ret                                                                   \n\t"
   :
   : [TRISTATE] "M" (TRI_STATE),
     [SNC] "M" (SYNC),
     [SXSTP] "M" (SX_STOP),
     [SXWRTNG] "M" (SXWRITING),
     [SXBT] "M" (SXBIT),
     [SXSEPLEN] "M" (SX_SEPLENGTH),
     [SXBTLEN] "M" (SX_BYTELENGTH),
     [SXFRMLEN] "M" (SX_FRAMELENGTH),
     [SXstat] "i" (&SXstate),
     [SXflgs] "i" (&SXflags),
     [SXshft] "i" (&SXshift),
     [SXspcnt] "i" (&SXsepCount),
     [SXbtcnt] "i" (&SXbyteCount),
     [SXfrmcnt] "i" (&SXframeCount),
     [SXidx] "i" (&SXindex)
   :
   : WriteD
  );
ISRPwr:
  asm goto (
  "   dec   r21                                                             \n\t"
  "   brne  ISRpwr10                                                        \n\t"
  "   ldi   r24, %[TRISTATE]                                                \n\t"
  "   rcall %l[WriteD]                                                      \n\t"
  "   ldi   r24, %[ADR]                                                     \n\t"
  "   sts   %[SXstat], r24                                                  \n\t"
  "   ldi   r22, %[SXBTLEN] / 2                                             \n\t"
  "   sts   %[SXbtcnt], r22                                                 \n\t"
  "   ldi   r21, %[SXSEPLEN]                                                \n\t"
  "   mov   r20, __zero_reg__                                               \n\t"
  "   rjmp  ISRpwr30                                                        \n\t"
  "ISRpwr10:                                                                \n\t"
  "   lds   r24, %[SXnwpwr]                                                 \n\t"
  "   cpi   r24, %[PWRSEND]                                                 \n\t"
  "   brcc  ISRpwr20                                                        \n\t"
  "   rcall %l[WriteD]                                                      \n\t"
  "   ldi   r24, %[PWRSEND]                                                 \n\t"
  "   sts   %[SXnwpwr], r24                                                 \n\t"
  "ISRpwr20:                                                                \n\t"
  "   bst   r23, %[SXBT]                                                    \n\t"
  "   bld   r23, %[SXPW]                                                    \n\t"
  "   sts   %[SXflgs], r23                                                  \n\t"
  "ISRpwr30:                                                                \n\t"
  "   sts   %[SXspcnt], r21                                                 \n\t"
  "   ret                                                                   \n\t"
   :
   : [TRISTATE] "M" (TRI_STATE),
     [PWRSEND] "M" (PWR_SEND),
     [ADR] "M" (ADDR),
     [SXBT] "M" (SXBIT),
     [SXPW] "M" (SXPWR),
     [SXSEPLEN] "M" (SX_SEPLENGTH),
     [SXBTLEN] "M" (SX_BYTELENGTH),
     [SXstat] "i" (&SXstate),
     [SXflgs] "i" (&SXflags),
     [SXbtcnt] "i" (&SXbyteCount),
     [SXspcnt] "i" (&SXsepCount),
     [SXnwpwr] "i" (&SXnewPWR)
   :
   : WriteD
  );
ISRAddr:
  asm volatile (
  "   dec   r21                                                             \n\t"
  "   brne  ISRaddr10        ; Skip separator                               \n\t"
  "   ldi   r21, %[SXSEPLEN]                                                \n\t"
  "   rjmp  ISRaddr20                                                       \n\t"
  "ISRaddr10:                                                               \n\t"
  "   add   r20, r20                                                        \n\t"
  "   bst   r23, %[SXBT]     ; Read bit in framenumber                      \n\t"
  "   bld   r20, 0                                                          \n\t"
  "ISRaddr20:                                                               \n\t"
  "   dec   r22              ; All bits done                                \n\t"
  "   brne  ISRaddr60                                                       \n\t"
  "   andi  r20, 0x0F                                                       \n\t"
  "   sts   %[SXfrmnum], r20                                                \n\t"
  "   brne  ISRaddr30        ; Frame '0'?                                   \n\t"
  "   set                                                                   \n\t"
  "   bld   r23, %[SXSNC]    ; Signal frame '0'                             \n\t"
  "ISRaddr30:                                                               \n\t"
  "   mov   r24, r20         ; (7 x 15 = 105 => no carry)                   \n\t"
  "   add   r24, r24         ; x2                                           \n\t"
  "   add   r24, r20         ; x3                                           \n\t"
  "   add   r24, r24         ; x6                                           \n\t"
  "   add   r24, r20         ; x7: bytes in frame                           \n\t"
  "   add   r24, r24         ; x2: convert from byte to int                 \n\t"
  "   ldi   ZL, lo8(%[SXarr]) ; First entry array                           \n\t"
  "   ldi   ZH, hi8(%[SXarr])                                               \n\t"
  "   add   ZL, r24          ; Add in offset                                \n\t"
  "   adc   ZH, __zero_reg__                                                \n\t"
  "   sts   %[SXidx], ZL                                                    \n\t"
  "   sts   %[SXidx] + 1, ZH ; Set as new address                           \n\t"
  "   ldd   r24, Z + 1       ; Check if we want to write                    \n\t"
  "   andi  r24, 0x01                                                       \n\t"
  "   brne  ISRaddr40                                                       \n\t"
  "   mov   r20, __zero_reg__                                               \n\t"
  "   clt                    ; Read                                         \n\t"
  "   rjmp  ISRaddr50                                                       \n\t"
  "ISRaddr40:                                                               \n\t"
  "   ld    r20, Z                                                          \n\t"
  "   std   Z + 1, __zero_reg__ ; Clear flag                                \n\t"
  "   set                    ; Write                                        \n\t"
  "ISRaddr50:                                                               \n\t"
  "   bld   r23, %[SXWRTNG]                                                 \n\t"
  "   ldi   r24, %[SXFRMLEN]                                                \n\t"
  "   sts   %[SXfrmcnt], r24                                                \n\t"
  "   ldi   r24, %[DTA]                                                     \n\t"
  "   sts   %[SXstat], r24                                                  \n\t"
  "   ldi   r22, %[SXBTLEN]                                                 \n\t"
  "ISRaddr60:                                                               \n\t"
  "   sts   %[SXshft], r20   ; Save common registers                        \n\t"
  "   sts   %[SXspcnt], r21                                                 \n\t"
  "   sts   %[SXbtcnt], r22                                                 \n\t"
  "   sts   %[SXflgs], r23                                                  \n\t"
  "   ret                                                                   \n\t"
   :
   : [DTA] "M" (DATA),
     [SXSNC] "M" (SXSYNC),
     [SXBT] "M" (SXBIT),
     [SXWRTNG] "M" (SXWRITING),
     [SXSEPLEN] "M" (SX_SEPLENGTH),
     [SXBTLEN] "M" (SX_BYTELENGTH),
     [SXFRMLEN] "M" (SX_FRAMELENGTH),
     [SXstat] "i" (&SXstate),
     [SXflgs] "i" (&SXflags),
     [SXfrmnum] "i" (&SXframenum),
     [SXshft] "i" (&SXshift),
     [SXidx] "i" (&SXindex),
     [SXarr] "i" (&SXarray[0]),
     [SXbtcnt] "i" (&SXbyteCount),
     [SXspcnt] "i" (&SXsepCount),
     [SXfrmcnt] "i" (&SXframeCount)
   :
  );
ReadT1:
  asm volatile (
  "   lds   r25, %[SXT1MASK] ; Read T1                                      \n\t"
  "   lds   ZL, %[SXT1IN]                                                   \n\t"
  "   lds   ZH, %[SXT1IN] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   clt                                                                   \n\t"
  "   and   r24, r25                                                        \n\t"
  "   breq  ReadT11                                                         \n\t"
  "   set                                                                   \n\t"
  "ReadT11:                                                                 \n\t"
  "   bld   r23, %[SXBT]     ; and save in flags                            \n\t"
  "   ret                                                                   \n\t"
   :
   : [SXBT] "M" (SXBIT),
     [SXT1IN] "i" (&SX_T1_IN),
     [SXT1MASK] "i" (&SX_T1_MASK)
   :
  );
WriteD:
  asm volatile( 
  "   sbrs  r23, %[SX4LN]                                                   \n\t"
  "   rjmp  D3Line                                                          \n\t"
  "D4Line:                                                                  \n\t"
  "   cpi   r24, 1           ; 0=low, 1=high else tristate                  \n\t"
  "   brcs  Low4Line                                                        \n\t"
  "   breq  High4Line                                                       \n\t"
  "TriState4Line:                                                           \n\t"
  "   lds   r25, %[SXDHIGHMASK]                                             \n\t"
  "   lds   ZL, %[SXDHIGHOUT]                                               \n\t"
  "   lds   ZH, %[SXDHIGHOUT] + 1                                           \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   r25, %[SXDLOWMASK]                                              \n\t"
  "   lds   ZL, %[SXDLOWOUT]                                                \n\t"
  "   lds   ZH, %[SXDLOWOUT] + 1                                            \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
  "High4Line:                                                               \n\t"
  "   lds   r25, %[SXDLOWMASK]                                              \n\t"
  "   lds   ZL, %[SXDLOWOUT]                                                \n\t"
  "   lds   ZH, %[SXDLOWOUT] + 1                                            \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   r25, %[SXDHIGHMASK]                                             \n\t"
  "   lds   ZL, %[SXDHIGHOUT]                                               \n\t"
  "   lds   ZH, %[SXDHIGHOUT] + 1                                           \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   com   r25                                                             \n\t"
  "   and   r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
  "Low4Line:                                                                \n\t"
  "   lds   r25, %[SXDHIGHMASK]                                             \n\t"
  "   lds   ZL, %[SXDHIGHOUT]                                               \n\t"
  "   lds   ZH, %[SXDHIGHOUT] + 1                                           \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   r25, %[SXDLOWMASK]                                              \n\t"
  "   lds   ZL, %[SXDLOWOUT]                                                \n\t"
  "   lds   ZH, %[SXDLOWOUT] + 1                                            \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   com   r25                                                             \n\t"
  "   and   r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
  "D3Line:                                                                  \n\t"
  "   cpi   r24, 1           ; 0=low, 1=high else tristate                  \n\t"
  "   brcs  Low3Line                                                        \n\t"
  "   breq  High3Line                                                       \n\t"
  "TriState3Line:                                                           \n\t"
  "   lds   r25, %[SXDMASK]                                                 \n\t"
  "   lds   ZL, %[SXDDIR]                                                   \n\t"
  "   lds   ZH, %[SXDDIR] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   com   r25                                                             \n\t"
  "   and   r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   ZL, %[SXDOUT]                                                   \n\t"
  "   lds   ZH, %[SXDOUT] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   and   r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
  "High3Line:                                                               \n\t"
  "   lds   r25, %[SXDMASK]                                                 \n\t"
  "   lds   ZL, %[SXDDIR]                                                   \n\t"
  "   lds   ZH, %[SXDDIR] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   ZL, %[SXDOUT]                                                   \n\t"
  "   lds   ZH, %[SXDOUT] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
  "Low3Line:                                                                \n\t"
  "   lds   r25, %[SXDMASK]                                                 \n\t"
  "   lds   ZL, %[SXDDIR]                                                   \n\t"
  "   lds   ZH, %[SXDDIR] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   or    r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   lds   ZL, %[SXDOUT]                                                   \n\t"
  "   lds   ZH, %[SXDOUT] + 1                                               \n\t"
  "   ld    r24, Z                                                          \n\t"
  "   com   r25                                                             \n\t"
  "   and   r24, r25                                                        \n\t"
  "   st    Z, r24                                                          \n\t"
  "   ret                                                                   \n\t"
   :
   : [SX4LN] "M" (SX4LINE),
     [SXDLOWOUT] "i" (&SX_D_LOW_OUT),
     [SXDLOWMASK] "i" (&SX_D_LOW_MASK),
     [SXDHIGHOUT] "i" (&SX_D_HIGH_OUT),
     [SXDHIGHMASK] "i" (&SX_D_HIGH_MASK),
     [SXDOUT] "i" (&SX_D_OUT),
     [SXDDIR] "i" (&SX_D_DIR),
     [SXDMASK] "i" (&SX_D_MASK)
   :
  );
}
#else
// Interrupt service routine
// Driven by RISING EDGES of the SX clock signal T0 (SX pin 1)
// This code is execute every 50 usec (keep it as fast as possible)
void SXAccessoire::isr() {
	// Process the T1 signal (read)
  if (!bitRead(SXflags, SXWRITING)) {
    bitWrite(SXflags, SXBIT, readT1());       // read pin
  }
	switch (SXstate) {
	case SYNC:
		// Find sync pattern "0001" to start
		if (bitRead(SXflags, SXBIT) == LOW) {     // Sync bits "0"
			if (SXbyteCount > 0) {                  // If more then 3
				SXbyteCount--;
			}
		}
		else {
			if (SXbyteCount == 0) {                 // High, read 3 bits low?
				SXstate = PWR;                        // Setup for POWER bit
				SXsepCount = SX_SEPLENGTH - 1;        // Set _sx_sepCount and continue
				break;
			}
			SXbyteCount = SX_STOP;                  // Error, setup for restart
			SXflags &= 0x0F0;
		}
		break;
	case PWR:
		// Read (and write) the power bit.
		SXsepCount--;
		if (SXsepCount == 0) {                    // Skip the separator
			writeD(TRI_STATE);		                  // Switch pin to input (let sender set the level)
			SXstate = ADDR;                         // Setup for next state ADDR
			SXbyteCount = SX_BYTELENGTH / 2;
			SXsepCount = SX_SEPLENGTH;
			SXshift = 0;                         // Prepare to read new framenumber
		}
		else {
			if (SXnewPWR < PWR_SEND) {              // Set power from me
				writeD(SXnewPWR);                     // write newPWR
				SXnewPWR = PWR_SEND;                  // Power set
			}
			bitWrite(SXflags, SXPWR, bitRead(SXflags, SXBIT)); //Set PWR Bit in Flag bit     
		}
		break;
	case ADDR:
		// Read the address bits.
		SXsepCount--;
		if (SXsepCount == 0) {             	      // Skip the separator
			SXsepCount = SX_SEPLENGTH;
		}
		else {
			SXshift = SXshift * 2;
			bitWrite(SXshift, 0, bitRead(SXflags, SXBIT)); // Read bit into framenumber
		}
		SXbyteCount--;
		if (SXbyteCount == 0) {                   // Addres part is processed
			if (SXshift == 0) {
				bitSet(SXflags, SXSYNC);              // Signal frame 0 for sync purposes
			}
			// Advance to the next state
			SXstate = DATA;                         // Setup for DATA read
			SXbyteCount = SX_BYTELENGTH;
      SXframeCount = SX_FRAMELENGTH;
			SXindex = SXshift * 7;                  // Calculate index
			// Check if we want to write and prepare it
			if (SXarray[SXindex] < WRITE) {
				bitClear(SXflags, SXWRITING);         // No write
			}
			else {
        SXshift = SXarray[SXindex];           // Get data to write
				SXarray[SXindex] &= 0xFF;             // Reset write flag
				bitSet(SXflags, SXWRITING);           // Write
			}
		}
		break;
	case DATA:
		// Read (and write) the data bits
		SXsepCount--;
		if (SXsepCount == 0) {                    // Skip the separator
			writeD(TRI_STATE);                      // Switch pin to input (let sender set the level)
			SXsepCount = SX_SEPLENGTH;
		}
		else {
			if (bitRead(SXflags, SXWRITING)) {      // Check if we want to write
				writeD(bitRead(SXshift, 0));          // Write bit to bus
				SXshift = SXshift / 2;                // Prepare for next write
			}
			else {
				SXshift = SXshift / 2;                // Prepare for reading data
				bitWrite(SXshift, 7, bitRead(SXflags, SXBIT)); // Insert the bit
			}
		}
		// Check if all bits done
		SXbyteCount--;
		if (SXbyteCount == 0) {                   // All bits done
			if ((!bitRead(SXflags, SXWRITING)) && (SXarray[SXindex] < WRITE)) {
				SXarray[SXindex] = SXshift;           // Save read data in array
			}
			// Setup for next read
			SXbyteCount = SX_BYTELENGTH;
			SXindex++;
			// Decrement dataFrameCount
			// check, if we already reached the last DATA block - in this
			// case move on to the next SX-Datenpaket, i.e. look for SYNC
			SXframeCount--;
			if (SXframeCount == 0) {
				// Move on to find SYNC pattern
				SXstate = SYNC;
				SXbyteCount = SX_STOP;
				bitClear(SXflags, SXWRITING);         // Read
			}
			else {
				// Check if we want to write
				if (SXarray[SXindex] < WRITE) {
					bitClear(SXflags, SXWRITING);       // Read
				}
				else {
					SXarray[SXindex] &= 0xFF;           // Reset write flag
					SXshift = SXarray[SXindex];         // Get data to write
					bitSet(SXflags, SXWRITING);         // Write
				}
			}
		}
		break;
	default:
		writeD(TRI_STATE);                  		  // Switch pin to input
		initVars();                               // Start looking for SYNC
		break;
	}  //end switch/case _sx_state
}

// IO functions (Memorymapped IO)
uint8_t SXAccessoire::readT1() {
  return ((*SX_T1_IN & SX_T1_MASK) > 0);      // Read the data on T1 of the SX-bus
}

void SXAccessoire::writeD(uint8_t val) {
  if (bitRead(SXflags, SX4LINE)) {
    // For the 4 line interface
    switch (val) {
	  case 0:
	    *SX_D_HIGH_OUT |= SX_D_HIGH_MASK;       // Switch to low on pin D of SX-bus
	    *SX_D_LOW_OUT &= ~SX_D_LOW_MASK;
	    break;
	  case 1:
		*SX_D_LOW_OUT |= SX_D_LOW_MASK;         // Switch to high on pin D of SX-bus
		*SX_D_HIGH_OUT &= ~SX_D_HIGH_MASK;
		break;
	  default:
		*SX_D_LOW_OUT |= SX_D_LOW_MASK;         // Switch to inactive on pin D of SX-bus
		*SX_D_HIGH_OUT |= SX_D_HIGH_MASK;
		break;
    }
  }
  else {
    // For the 3 line interface
    switch (val) {
      case 0:
        *SX_D_DIR |= SX_D_MASK;
        *SX_D_OUT &= ~SX_D_MASK;                // Switch to low on pin D of SX-bus
        break;
      case 1:
        *SX_D_DIR |= SX_D_MASK;
        *SX_D_OUT |= SX_D_MASK;                 // Switch to high on pin D of SX-bus
        break;
      default:
        *SX_D_DIR &= ~SX_D_MASK;                // Switch to input (TRI_STATE)
        *SX_D_OUT &= ~SX_D_MASK;                // (internal pull-up off)
        break;
    }
  }
}
#endif

// Convert from SX-bus addresses to index in array.
// Magic arithmic only understand by Selectrix users
/*
 * The SX-bus adresses are linearised to allow the ISR to use a fast, simple incremental index.
 * This index is synchronised with each reception of a new frame
 * Offical formula to calculatea Selectrix address from framenumber and offset
 *   SXaddress = (15 - framenumber) + (6 - offset) * 16
 * (All 112 addresses, 0 - 111, are availeble if centrale permits.)
 */
uint8_t SXAccessoire::calcIndex(uint8_t SXadr) {
	uint8_t frame = 15 - (SXadr & 15);          // Get the frame number
	uint8_t offset = 6 - (SXadr >> 4);          // Get the offset in the frame
	return frame * 7 + offset;                  // Calculate the index in the array
}

// Public functions 'accessing' the SX-bus

// Read data from the array, filled by the isr.
int SXAccessoire::read(uint8_t adr) {
	// returns the value of a SX address
	if (adr < SX_ADDRESS_NUMBER) {
		return SXarray[calcIndex(adr)] & 0xFF;      // Return data from the SX-bus
	}
	return -1;                                  // Return error
}

// Write data to the array, writing to the SX-bus is done by the isr.
uint8_t SXAccessoire::write(uint8_t adr, uint8_t dt) {
	// Check if invalid address.
	if (adr < SX_ADDRESS_NUMBER) {
		SXarray[calcIndex(adr)] = dt | WRITE;
		return 0;                                 // Success
	}
	return 1;                                   // Address out of range
}

// Read POWER status from the SX-bus
uint8_t SXAccessoire::readPWR() {
	return bitRead(SXflags, SXPWR) > 0;
}

// Write POWER status to the SX-bus and control a connected central.
void SXAccessoire::writePWR(uint8_t val) {
	if (val == 0 || val == 1) {
		SXnewPWR = val;
	}
}

// Every time frame 0 is passed sync bit is set by isr.
uint8_t SXAccessoire::inSync() {
	if (bitRead(SXflags, SXSYNC) > 0) {
		// reset sync bit to check for next pass
		bitClear(SXflags, SXSYNC);
		return 1;                                 // Report frame 0 found
	}
	return 0;
}

/*
 * Programming an accessoire on the SXbus is done under control
 * of bit 5 in address 106. This is one of the two free bits in a CC2000
 * https://www.digit-electronic.de (Info => SX-wissen)
 */

 // Check programming status
#define SXcheckprogadr   106     // Address to check if other accessoire is already programming
#define SXcheckprogbit     4     // Bit for programming control (0 based)

// True if programming allowed
bool SXAccessoire::checkProg() {
	return ((read(SXcheckprogadr) & _BV(SXcheckprogbit)) == 0);
}

// Set programming state
void SXAccessoire::setProg(bool state) {
	byte wait = read(SXcheckprogadr);           // Read status byte from SX-bus
	write(SXcheckprogadr, (state ? wait | _BV(SXcheckprogbit) : wait & ~_BV(SXcheckprogbit))); // Selectrix bit 5 <= state  
}

// Check if programming is posible (bit 5 address 106 is "0")
// If possible claim programming by making bit 5 "1" in address 106
// false: no programming, true: programming
// Leave with bit 5 address 106 set if programming is possible (claim).
bool SXAccessoire::claimProg() {
	if (checkProg())
	{
		setProg(true);
		return true;
	}
	return false;
}
