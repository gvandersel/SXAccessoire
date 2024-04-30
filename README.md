# SXAccessoire
Selectrix 1 interface for Arduino.

This software gives an Arduino an interface to the SX-bus. 
The SX-bus is the bus to communicate with all accessoires to control the switches and signals. 
Also the feedbacks reports on this bus.

Installing:
Load the SXAccessoire library with the librarymanager. 
It is installed at the right place and ready for use.

Use:
First we have to include the library in our project (sketch). This is done with the following code:
''''
#include <SXAccessoire.h>
''''
After including the library it is ready for use.

Configuration:
SXaccessoire has two constructors which configure the Arduino to follow the information on the SX-bus. 
Which to use depends on the electrical interface (hardware) used to connect to the SX-bus. 
Examples of this hardware are found in the section "hardware".
The three wire interface is configured with:
''''
#define SX_T0        2            // SXbus clock, must be pin 2 or pin 3
#define SX_T1        8            // SXbus data from the centrale 
#define SX_D         4            // SXbus data to the centrale

SXAccessoire SXbus(SX_T0, SX_T1, SX_D);
''''

With this configuration example the pins 2, 8 and 4 are used to control the interface to the SX-bus.

The four wire interface is configured with:
''''
#define SX_T0        2            // SXbus clock, must be pin 2 or pin 3
#define SX_T1        8            // SXbus data from the centrale
#define SX_D_HIGH    4            // SXbus data to the centrale (sets high)
#define SX_D_LOW     5            // SXbus data to the centrale (sets low)

SXAccessoire SXbus(SX_T0, SX_T1, SX_D_LOW, SX_D_HIGH);
''''

With this configuration example the pins 2, 8, 4 and 5 are used to control the interface to the SX-bus.

After initialising the SX-bus can be used through the variable SXbus.

Implementing the ISR:

For communicating with the SX-bus an ISR is defined in the library. This ISR must be called every clock-cycle. 
Depending on the state of the ISR an action on the SX-bus is performed. To call the ISR in the library there is a function nessesary. 
The only action requered in the sketsh is calling the isr in the library. Code for this is:
''''
// Interrupt service routine. (Int0, Rising edge) 
void sxisr(void) {
    SXbus.isr();
} 
''''
In setup() the code is activated on a rising edge of the clocksignal (T0 of the SX-bus). 
Put the following lines in setup() (to initialize SX-bus):
''''
void setup() {
    // put your setup code here, to run once:
    ....
  
    // initialize SX-bus
    if (SXbus.init()) {
        // Rising edges on INT0 triggers the interrupt routine sxisr (see above)
        attachInterrupt(0, sxisr, RISING); 
    }
	
    ....
}
''''
After initialising the interrupt pin is attached to the function so every rising of the clock signal leads to the axecution of the ISR.

We can use the defined functions to read and write to the SX-bus, control power on the track and maintain synchronisity with the SX-bus.
Functions to use:
- isr:
   Executes every clockcyle and makes the nessesary changes to the SX-bus.
- init:
   Initialises the internal variables and sets the ISR to start looking for a frame
- read(address):
   Returns the integer value of the address on the SX-bus. (returns -1 if illegal address is asked.)
- write(address, data):
   Writes the byte data to the specified address. (returns 1 if illegal address is used.)   
- readPWR:
   Read if power is applied to the tracks (0 = no power, 1 = power).
- writePWR(0 of 1):
   Set or remove power from the track (0 = no power, 1 = power).
- inSync:
   Set if all data is read/write from/to the SX-bus.
   
Extra functions which can be used to inform accessoires if the SX-bus is used for proramming an accessoire.
Watch out: Address 106 and bit 5 are used to signal this. This bit is normaly in the reserved area (address 104 to 111).
- checkProg:
    Checks the state of bit 5 of address 106.   
- setProg(false or true):
    Sets bit 5 of address 106 accoording the input value.
- claimProg:
    Checks if bit 5 of address 106 is "0". If so bit 5 of address 106 is set and programming is enabled.
	If bit 5 of address 106 is "1" no programming should be performed.


