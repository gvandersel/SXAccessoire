/*
 * SXAduino.cpp
 *
 *  Version:    3.2
 *  Copyright:  Gerard van der Sel
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
 - SX T0 (Clock) must be connected to Pin 3 (IOL, INT1);
 - SX T1 must be connected to Pin 5 (IOL, T1);
 - SX D must be connected to Pin 6 (IOL, AIN0).

 SX-bus interface (NEM 682)

The interface is optimised in time. It is written as fast as posible
  to let the Arduino perform other tasks in the forground.
  Watch out when changing the code and check for performance in time.

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

#include "SXArduino.h"

SXArduino::SXArduino(uint8_t SX_T0_PIN, uint8_t SX_T1_PIN, uint8_t SX_D_PIN) {
	bitClear(_sx_busFlag, SXPINS);
	if ((SX_T0_PIN == 2) || (SX_T0_PIN == 3)) {
		// For the 3 line interface
		bitClear(_sx_busFlag, SX4LINE);
		pinMode(SX_T0_PIN, INPUT);                  // SX-T0 is an input, no pull up
		pinMode(SX_T1_PIN, INPUT);                  // SX-T1 is also an input, no pull up
		pinMode(SX_D_PIN, INPUT);                   // SX-D is also an input when not writing to allow other devices to write

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
		bitWrite(_sx_busFlag, SXPINS, ((digitalPinToPort(SX_T1_PIN) != NOT_A_PIN) &&
			                           (digitalPinToPort(SX_D_PIN) != NOT_A_PIN)));
	}
}

SXArduino::SXArduino(uint8_t SX_T0_PIN, uint8_t SX_T1_PIN, uint8_t SX_D_LOW_PIN, uint8_t SX_D_HIGH_PIN) {
	bitClear(_sx_busFlag, SXPINS);
	if ((SX_T0_PIN == 2) || (SX_T0_PIN == 3)) {
		// For the 4 line interface
		bitSet(_sx_busFlag, SX4LINE);
		pinMode(SX_T0_PIN, INPUT_PULLUP);           // SX-T0 is an input,pull  up
		pinMode(SX_T1_PIN, INPUT_PULLUP);           // SX-T1 is also an input, pull up
		digitalWrite(SX_D_LOW_PIN, HIGH);
		pinMode(SX_D_LOW_PIN, OUTPUT);              // SX_D_LOW is output but set high to stop wrting low
		digitalWrite(SX_D_HIGH_PIN, HIGH);
		pinMode(SX_D_HIGH_PIN, OUTPUT);             // SX_D_HIGH is output but set high to stop wrting high

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
		bitWrite(_sx_busFlag, SXPINS, ((digitalPinToPort(SX_T1_PIN) != NOT_A_PIN) &&
			                           (digitalPinToPort(SX_D_LOW_PIN) != NOT_A_PIN) &&
			                           (digitalPinToPort(SX_D_HIGH_PIN) != NOT_A_PIN)));
	}
}

// initialize function
bool SXArduino::init() {
	// initialize data array
	for (int i = 0; i < SX_ADDRESS_NUMBER; i++) {
		_sxbus[i] = 0;                              // set sx data to 0
	}
	// initialize variables
	initVars();                                     // Start looking for SYNC
	return bitRead(_sx_busFlag, SXPINS);;
}

void SXArduino::initVars() {
	// start always with search for header
	_sx_state = SYNC;                               // First look for SYNC pattern
	_sx_sepCount = SX_SEPLEN;                       // Distanse between two separators
	_sx_byteCount = SX_STOP;                        // Check for SX_STOP bits of "0"
	_sx_dataFrameCount = SX_DATACOUNT;              // Read all dataframes

	// no writing to the SX bus
	_sx_busFlag &= 0xF0;                            // Reset flags

	// Powerbit send and receive
	_sx_newPWR = PWR_SEND;                          // Don't write power bit
}

// IO functions (Memorymapped IO)
uint8_t SXArduino::readT1() {
	return ((*SX_T1_IN & SX_T1_MASK) > 0);          // Read the data on T1 of the SX-bus
}

void SXArduino::writeD(uint8_t val) {
	if (bitRead(_sx_busFlag, SX4LINE)) {
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
			*SX_D_DIR |= SX_D_MASK;                	// Switch to low on pin D of SX-bus
			*SX_D_OUT &= ~SX_D_MASK;
			break;
		case 1:
			*SX_D_DIR |= SX_D_MASK;	                // Switch to high on pin D of SX-bus
			*SX_D_OUT |= SX_D_MASK;
			break;
		default:
			*SX_D_DIR &= ~SX_D_MASK;			    // Switch to input (TRI_STATE)
			*SX_D_OUT &= ~SX_D_MASK;			    // (internal pull-up off)
			break;
		}
	}
}

// interrupt service routine (AVR INT0/INT1)
// driven by RISING EDGES of the SX clock signal T0 (SX pin 1)
// This code is execute every 50 usec (keep it fast as possible)
void SXArduino::isr() {
	// Process the T1 signal (read)
	bitWrite(_sx_busFlag, SXBIT, readT1());         // read pin
	switch (_sx_state) {
	case SYNC:
		// Find sync pattern "0001" to start
		if (bitRead(_sx_busFlag, SXBIT) == LOW) {   // Sync bits "0"
			if (_sx_byteCount > 0) {                // If more then 3
				_sx_byteCount--;
			}
		}
		else {
			if (_sx_byteCount == 0) {               // High, read 3 bits low?
				_sx_state = PWR;                    // Setup for POWER bit
				_sx_sepCount = SX_SEPLEN - 1;       // Set _sx_sepCount and continue
				break;
			}
			_sx_byteCount = SX_STOP;                // Error, setup for restart
			_sx_busFlag &= 0x0F0;
		}
		break;
	case PWR:
		// Read (and write) the power bit.
		_sx_sepCount--;
		if (_sx_sepCount == 0) {                    // Skip the separator
			writeD(TRI_STATE);		                // Switch pin to input (let sender set the level)
			_sx_state = ADDR;                       // Setup for next state ADDR
			_sx_byteCount = SX_BYTELEN / 2;
			_sx_sepCount = SX_SEPLEN;
			_sx_numFrame = 0;                       // Prepare to read new framenumber
		}
		else {
			if (_sx_newPWR < PWR_SEND) {            // Set power from me
				writeD(_sx_newPWR);                 // write newPWR
				_sx_newPWR = PWR_SEND;              // Power set
			}
			bitWrite(_sx_busFlag, SXPWR, bitRead(_sx_busFlag, SXBIT)); //Set PWR Bit in Flag bit     
		}
		break;
	case ADDR:
		// Read the address bits.
		_sx_sepCount--;
		if (_sx_sepCount == 0) {             	    // Skip the separator
			_sx_sepCount = SX_SEPLEN;
		}
		else {
			_sx_numFrame = _sx_numFrame * 2;
			bitWrite(_sx_numFrame, 0, bitRead(_sx_busFlag, SXBIT)); // Read bit into framenumber
		}
		_sx_byteCount--;
		if (_sx_byteCount == 0) {                   // Addres part is processed
			if (_sx_numFrame == 0) {
				bitSet(_sx_busFlag, SXSYNC);        // Signal frame 0 for sync purposes
			}
			// Advance to the next state
			_sx_state = DATA;                       // Setup for DATA read
			_sx_byteCount = SX_BYTELEN;
			_sx_index = _sx_numFrame * 7;           // Calculate index
			// Check if we want to write and prepare it
			if (_sxbus[_sx_index] < WRITE) {
				bitClear(_sx_busFlag, SXWRITING);   // No write
			}
			else {
				_sxbus[_sx_index] &= 0xFF;          // Reset write flag
				_sx_write_data = _sxbus[_sx_index]; // Get data to write
				bitSet(_sx_busFlag, SXWRITING);     // Write
			}
		}
		break;
	case DATA:
		// Read (and write) the data bits
		_sx_sepCount--;
		if (_sx_sepCount == 0) {                    // Skip the separator
			writeD(TRI_STATE);                      // Switch pin to input (let sender set the level)
			_sx_sepCount = SX_SEPLEN;
		}
		else {
			if (bitRead(_sx_busFlag, SXWRITING)) {  // Check if we want to write
				writeD(bitRead(_sx_write_data, 0)); // Write bit to bus
				_sx_write_data = _sx_write_data / 2; // Prepare for next write
			}
			else {
				_sx_read_data = (_sx_read_data / 2); // Prepare for reading data
				bitWrite(_sx_read_data, 7, bitRead(_sx_busFlag, SXBIT)); // Insert the bit
			}
		}
		// Check if all bits done
		_sx_byteCount--;
		if (_sx_byteCount == 0) {                   // All bits done
			if ((!bitRead(_sx_busFlag, SXWRITING)) && (_sxbus[_sx_index] < WRITE)) {
				_sxbus[_sx_index] = _sx_read_data;  // Save read data in array
			}
			// Setup for next read
			_sx_byteCount = SX_BYTELEN;
			_sx_index++;
			// Decrement dataFrameCount
			// check, if we already reached the last DATA block - in this
			// case move on to the next SX-Datenpaket, i.e. look for SYNC
			_sx_dataFrameCount--;
			if (_sx_dataFrameCount == 0) {
				// Move on to find SYNC pattern
				_sx_dataFrameCount = SX_DATACOUNT;
				_sx_state = SYNC;
				_sx_byteCount = SX_STOP;
				bitClear(_sx_busFlag, SXWRITING);   // No write
			}
			else {
				// Check if we want to write
				if (_sxbus[_sx_index] < WRITE) {
					bitClear(_sx_busFlag, SXWRITING); // No write
				}
				else {
					_sxbus[_sx_index] &= 0xFF;      // Reset write flag
					_sx_write_data = _sxbus[_sx_index]; // Get data to write
					bitSet(_sx_busFlag, SXWRITING); // Write
				}
			}
		}
		break;
	default:
		writeD(TRI_STATE);                  		// Switch pin to input
		initVars();                                 // Start looking for SYNC
		break;
	}  //end switch/case _sx_state
}

// Convert from SX-bus addresses to index in array.
// Magic arithmic only understand by Selectrix users
/*
 * The SX-bus adresses are linearised to allow the ISR to use a fast, simple incremental index.
 * This index is synchronised with each reception of a new frame
 * Offical formula to calculatea Selectrix address from framenumber and offset
 *   SXaddress = (15 - framenumber) + (6 - offset) * 16
 * (All 112 addresses, 0 - 111, are availeble if centrale permits.)
 */
uint8_t SXArduino::calcIndex(uint8_t SXadr) {
	uint8_t frame = 15 - (SXadr & 15);              // Get the frame number
	uint8_t offset = 6 - (SXadr >> 4);              // Get the offset in the frame
	return frame * 7 + offset;                      // Calculate the index in the array
}

// Public functions 'accessing' the SX-bus

// Read data from the array, filled by the isr.
int SXArduino::read(uint8_t adr) {
	// returns the value of a SX address
	if (adr < SX_ADDRESS_NUMBER) {
		return _sxbus[calcIndex(adr)] & 0xFF;       // Return data from the SX-bus
	}
	return -1;                                      // Return error
}

// Write data to the array, writing to the SX-bus is done by the isr.
uint8_t SXArduino::write(uint8_t adr, uint8_t dt) {
	// Check if invalid address.
	if (adr < SX_ADDRESS_NUMBER) {
		_sxbus[calcIndex(adr)] = dt | WRITE;
		return 0;                                   // Success
	}
	return 1;                                       // Address out of range
}

// Read POWER status from the SX-bus
uint8_t SXArduino::readPWR() {
	return bitRead(_sx_busFlag, SXPWR) > 0;
}

// Write POWER status to the SX-bus and control a connected central.
void SXArduino::writePWR(uint8_t val) {
	if (val == 0 || val == 1) {
		_sx_newPWR = val;
	}
}

// Every time frame 0 is passed sync bit is set by isr.
uint8_t SXArduino::inSync() {
	if (bitRead(_sx_busFlag, SXSYNC) > 0) {
		// reset sync bit to check for next pass
		bitClear(_sx_busFlag, SXSYNC);
		return 1;                                   // Report frame 0 found
	}
	return 0;
}

/*
 * Programming a decoder on the SXbus is done under control
 * of bit 5 in address 106. This is one of the two free bits in a CC2000
 * https://www.digit-electronic.de (Info => SX-wissen)
 */

 // Check programming status
#define SXcheckprogadr   106     // Address to check if other device is programming
#define SXcheckprogbit     4     // Bit for programming control (0 based)

// True if programming allowed
bool SXArduino::checkProg() {
	return ((read(SXcheckprogadr) & _BV(SXcheckprogbit)) == 0);
}

// Set programming state
void SXArduino::setProg(bool state) {
	byte wait = read(SXcheckprogadr);               // Read status byte from SX-bus
	write(SXcheckprogadr, (state ? wait | _BV(SXcheckprogbit) : wait & ~_BV(SXcheckprogbit))); // Bit 5 <= state  
}

// Check if programming is posible (bit 5 address 106 is "0")
// If possible claim programming by making bit 5 "1" in address 106
// false: no programming, true: programming
// Leave with bit 5 address 106 set if programming is possible (claim).
bool SXArduino::claimProg() {
	if (checkProg())
	{
		setProg(true);
		return true;
	}
	return false;
}
