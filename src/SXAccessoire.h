/*
 * SXAccessoire.h
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
 *  Changes:    Pin configurable from outside the class
 *              Made check if programming is activated on the SXbus
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Added 3 and 4 pin interface.
 *
 *  Changed on: 19.12.2015
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
 *  interface hardware needed ! see

 Read SX Signal - SX Clock must be connected to Pin3 = INT1 and
 SX Data must be connected to Pin 5. Both are connected through a resistor off 22 kilo ohm.
 Pin 6 can be connected via a 100 ohm resistor to the write line
 of the SX bus

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

#ifndef SXAccessoire_H_
#define SXAccessoire_H_
// define arduino pins, ports and bits
// depends on the hardware used.
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define TRI_STATE           3                   // Don't set data on SX-bus
#define PWR_SEND            2                   // Don't set power on SX-bus

// defines for Selectrix constants
#define SX_STOP             3                   // 3 "0" bits achter elkaar
#define SX_SEPLENGTH        3                   // 3 bit in a separated part
#define SX_BYTELENGTH      12                   // 12 bits for one byte
#define SX_FRAMELENGTH      7                   // 7 dataframes in 1 frame

#define SX_ADDRESS_NUMBER 112                   // Max number SX channels

#define WRITE           0x100                   // Flag to signal data to write

class SXAccessoire {
public:
	SXAccessoire(uint8_t, uint8_t, uint8_t, uint8_t);   // 4 wire interface
	SXAccessoire(uint8_t, uint8_t, uint8_t);	          // 3 wire interface

	void isr(void);                               // Interrupt service routine, runs in the background
	bool init(void);                              // Init all variables for the SX-bus, return false if somthing fails
	int read(uint8_t);                            // Read given address from the SXbus
	uint8_t write(uint8_t, uint8_t);              // Write data to given address of the SXbus
	uint8_t readPWR(void);                        // Signals if power is on the track
	void writePWR(uint8_t);                       // Sets power on or off
	uint8_t inSync(void);                         // Signals the completion of all addresses
	bool checkProg(void);                         // Checks if posible to program an accessoire
	void setProg(bool);                           // Sets or resets programming state for an accessoire
	bool claimProg(void);                         // Checks and sets programmingstate for an accessoire
private:
	void initVars();                              // Sets internal variable for the isr 
	uint8_t calcIndex(uint8_t adr);               // Addresses are not lineair in the array, so calculate index
	uint8_t readT1();                             // Read SXbus pin T1 (Serial data from centrale to accessoire)
  void writeD(uint8_t val);                     // Write SXbus pin D (Serial data form accessoire to centrale)


  volatile uint8_t SXflags;                     // Various flags
#define SXBIT               0                   //     Value bit read (T1)
#define SXWRITING           1                   //     Flag signalling writing is in progress
#define SXPWR               2                   //     Value powerbit (0 = no power, 1 = power)
#define SXSYNC              3                   //     Flag signaling all frames are processed
#define SX4LINE             4                   //     Flag signaling to use 4 line interface
#define SXPINS              5                   //     Flag signaling pins are initialised
  volatile uint8_t SXstate;
#define DATA                0                   // For performance DATA first
#define SYNC                1                   // (Gives fastest code)
#define PWR                 2
#define ADDR                3

  volatile uint8_t SXnewPWR;                    // Set POWER on track
	//     0 = remove power from track
	//     1 = set power on track
	//    >2 = don't change power
 
  volatile uint8_t SXframeCount;                // Byte counting for frame
  volatile uint8_t SXbyteCount;                 // Bit counting for byte
  volatile uint8_t SXsepCount;                  // Bit counting for separator
  
  volatile uint8_t SXframenum;                  // Number off current frame
  volatile uint8_t SXshift;                     // Data receive/transmit (internal)
  volatile uint16_t SXindex;                    // Current index in the array

	// Array for storing SX_bus data
  volatile uint16_t SXarray[SX_ADDRESS_NUMBER]; // SX data array

	// Addresses and masks for Memorymapped IO 
	// Clock (T0)
	//  volatile uint8_t SX_T0_MASK;
	//  volatile uint8_t * SX_T0_OUT;
	//  volatile uint8_t * SX_T0_IN;
	//  volatile uint8_t * SX_T0_DIR;
	// Data in (T1)
  volatile uint8_t SX_T1_MASK;
	//  volatile uint8_t * SX_T1_OUT;
	volatile uint8_t * SX_T1_IN;
	//  volatile uint8_t * SX_T1_DIR;
	// Data out (D)
	// For the 3 line interface
  volatile uint8_t SX_D_MASK;
  volatile uint8_t * SX_D_OUT;
	//  volatile uint8_t * SX_D_IN;
  volatile uint8_t * SX_D_DIR;
	// For the 4 line interface
  volatile uint8_t SX_D_LOW_MASK;
  volatile uint8_t * SX_D_LOW_OUT;
	//  volatile uint8_t * SX_D_LOW_IN;
	//  volatile uint8_t * SX_D_LOW_DIR;
  volatile uint8_t SX_D_HIGH_MASK;
  volatile uint8_t * SX_D_HIGH_OUT;
	//  volatile uint8_t * SX_D_HIGH_IN;
	//  volatile uint8_t * SX_D_HIGH_DIR;

/* SX Timing
	1   Bit             50 us
	1   Kanal           600 us (= 12 Bit)
	1   Grundrahmen     ca. 4,8 ms
	1   Gesamtrahmen    ca.  80 ms (= 16 Grundrahmen)
	0  0  0  1  S   1  A3  A2  1  A1  A0  1 == sync frame of 12 bits
	*/
};

#endif /* SXAccessoire_H_ */
