/*
   SX interface (66824 or 66842)

   Creator: Gerard van der Sel
   Version: 1.1
   Date: 27-12-2015
   
   This software emulates a SX interface on the Arduino.
   Extra hardware needed is de SX-Arduino board which contains the hardware to communicate with the SX-bus.

   Protocol used by the SX-Interface (66824 or 66842)
   2 byte message:
   First byte contains command and address:
      Command is located in bit 7 of first byte:
      0: Read address
      1: Write address
      Address: 0 to 111 (valid adresses on the SX-bus)
               127 controls track power
   Second byte contains data in case of a write command
      in case of a valid address: 0 to 255
      in case of address 127: 0 trackpower off
                              128 trackpower on
      in case of a read: byte is discarded

   Note: For this sketch to run, don't solder switch and LED on de SXArduino board
*/

#include <Arduino.h>
#include <SXAccessoire.h>

SXAccessoire SXbus(3, 5, 6); // Interface to the SX-bus
boolean cmdAdr;           // Received byte is a command
boolean cmdWrite;         // Write command
uint8_t address;          // Address for reading or writing
uint8_t rcvdData;

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    SXbus.isr();
} 


void setup() {
    // put your setup code here, to run once:
    // initialize serial:
    Serial.begin(19200);

    // initialize SX-bus
    if (SXbus.init()) {
      // Changing edges on INT1 triggers the interrupt routine sxisr (see above)
      attachInterrupt(1, sxisr, RISING); 
    }
    // initialize application
    cmdAdr = true;
    cmdWrite = false;
}

void serialEvent() {
    // Read all the data
    while (Serial.available()) {
        rcvdData = (uint8_t)Serial.read();
        // First byte is a command, decode it
        if (cmdAdr) {
            // If byte value > 127 a write command, data in second byte
            if (rcvdData > 127) {
                cmdWrite = true;
                address = rcvdData - 128;  // address is received data - 128
            } else {  // Read command, perform it
                if (rcvdData < 112) {    // Get address data
                    Serial.print((char)SXbus.read(rcvdData));
                } else {   // Illegal address, power?
                    if (rcvdData == 127) {
                        Serial.print((char)(SXbus.readPWR() * 128));
                    }
                }
            }
            cmdAdr = false;
        } else {
            // Second byte data
            if (cmdWrite) {
                if (address < 112) {
                    SXbus.write(address, rcvdData);
                } else {
                    if (address == 127) {
                        if ((rcvdData & 128) == 0) {
                            SXbus.writePWR(0);
                        } else {
                            SXbus.writePWR(1);
                        }
                    }
                }
                cmdWrite = false;
            }
            cmdAdr = true;
        }
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Nothing to do.
}
