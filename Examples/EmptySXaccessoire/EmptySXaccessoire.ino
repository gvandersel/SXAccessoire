// Empty SX accessoire

#include <Arduino.h>
#include <EEPROM.h>
#include <SXArduino.h>

//Port definitions
#define progKey A0                // Switch connected to ground
#define progLED 8                 // LED connected to ground
#define SX_T0              2
#define SX_T1              4
#define SX_D_LOW           5
#define SX_D_HIGH          6
// SX-bus interface definitie
SXArduino SXbus(SX_T0, SX_T1, SX_D_LOW, SX_D_HIGH);

const int debounceVal = 2;
int debounceWait;
bool switchPressed;
bool programming;
bool dataChanged;

// For debug only
boolean cmdAdr;          // Received byte is a command
boolean cmdWrite;        // Write command
uint8_t address;         // Address for reading or writing
uint8_t rcvdData;


void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    SXbus.isr();
}

bool debounceSwitch() {
  if (digitalRead(progKey) == 0) {
    // Switch pressed, check if debounced
    if (debounceWait != 0) {
      // Not already debounced, check if ready waiting
      debounceWait--;
      return (debounceWait == 0);
    }
  } else {
    // Switch not pressed reset waittime
    debounceWait = debounceVal;
  }
  return false;
}

void initAccessoire() {
  // Init accessoire

}

// Every thing the program has to do without actions on the SXbus
void processAccessoire() {
  // Normal actions without SXbus actions
  
}

// Every thing the program has to do on the SX-bus
void powerAccessoire() {
  
}

// Prepare SX-bus for programming
void startprogAccessoire() {
  // Write all variables to the SXbus to be changed by the user

}

// Wait for user to end programming
void progAccessoire() {
  // Read all variables from the SXbus
  
}

// End programming, release SXbus
void endprogAccessoire() {

}

// EErpom actions
void writeEEprom() {
  // Write all variables to EEprom

}

void readEEprom() {
  // Read all variables from EEprom

}

// For debug perposes only
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

void setup() {
  // put your setup code here, to run once:
  pinMode(progLED, OUTPUT);
  pinMode(progKey, INPUT_PULLUP);

  // initialize SX-bus
  if (SXbus.init()) {
	  // Rising edges on INT0 triggers the interrupt routine sxisr (see above)
	  attachInterrupt(0, sxisr, RISING); 
  }

  // initialize serial (for debug only):
  Serial.begin(19200);
  cmdAdr = true;
  cmdWrite = false;

  // Init internal variables
  
  debounceWait = debounceVal;
  switchPressed = false;
  programming = false;
  dataChanged = false;
  readEEprom();
  digitalWrite(progLED, LOW);                 // LED off

  // Init used varables and ports
  initAccessoire();
}

/* Action performed to acces the SXbus:
 * getPWR()  programming  switchpressed  action
 *    1            *            *        powerAccessoire  
 *    0            0            0        -
 *    0            0            1        startprogAccessoire
 *    0            1            0        progAccessoire
 *    0            1            1        endprogAccessoire
 */

void loop() {
  // put your main code here, to run repeatedly:
  if (SXbus.inSync() == 1) {
    // Debounce switch
    if (debounceSwitch()) {
      switchPressed = true;                      // Switch debounced, signal pressed.
    }
    if (SXbus.readPWR() == 1) {
      // Power on the track, perform normal operation
      // Reset all variables to initial state
      switchPressed = false;  
      if (programming) {
        SXbus.setProg(false);
        programming = false;
      }
      digitalWrite(progLED, LOW);                 // LED off
      // Set all variables conform EEprom values
      if (dataChanged) {
        readEEprom();
        dataChanged = false;
      }
      powerAccessoire();
    } else {
      // No power on the track, on switch press start/stop programming
      if (programming) {
        if (!SXbus.checkProg()) {
          if (switchPressed) {
            // In programming mode and key pressed, terminate programming mode
            endprogAccessoire();
            // Write all to EEProm
            writeEEprom();
            dataChanged = false;
            // End programming mode  
            programming = false;
            SXbus.setProg(false);
            switchPressed = false;  
            digitalWrite(progLED, LOW);                 // LED off
          } else {
            // In programming mode, so communicate with the SXbus
            progAccessoire();
            dataChanged = true;
          }
        } else {
          // End programming mode  
          switchPressed = false;
          programming = false;
          digitalWrite(progLED, LOW);                       // LED off
        }
      } else {
        if ((switchPressed) && (SXbus.claimProg())) {
          // No power on the track and key pressed, start programmin mode
          startprogAccessoire();
          // Start programming mode
          switchPressed = false;  
          programming = true;
          digitalWrite(progLED, HIGH);                 // LED on
        }
      }
    }
  }
  // Perform all action to let the accessoire work
  processAccessoire();
}
