// Occupiance detector for SXbus

#define Version   0x30

#include <Arduino.h>
#include <EEPROM.h>
#include <SXArduino.h>
#include "EEOccupianceDetektor\EEOccupianceDetektor.h"

//Pin definitions
#define progKey      0            // Switch connected to ground
#define progLED      1            // LED connected to ground

#define SX_T0        2            // SXbus clock signal
#define SX_T1        3            // SXbus data signal
#define SX_D_HIGH    4            // SXbus report data high
#define SX_D_LOW     5            // SXbus report data low

#define Relais       9

#define Occupiance0 10            // LOW reports a train
#define Occupiance1 11 
#define Occupiance2 12 
#define Occupiance3  6            // Instead of 13 (LED_BUILTIN)
#define Occupiance4 A0
#define Occupiance5 A1
#define Occupiance6 A2
#define Occupiance7 A3

// SXbus adress dfinition
#define SXAddress    0
#define SXInvert     1
#define SXMaskAB     2
#define SXMaskDE     3
#define SXVersion    4

// EEprom offsets in apart file in subdirectory EEOccupianceDetektor

const int debounceVal = 2;

SXArduino SXbus(SX_T0, SX_T1, SX_D_LOW, SX_D_HIGH);
int debounceWait;
bool switchPressed;
bool programming;
bool dataChanged;

byte Occupiance;
byte Invert;
byte Address;
byte MaskAB;
byte MaskDE;
bool ReportDouble;
long Loops;

// Interrupt service routine. (Int0, Rising edge) 
void sxisr(void) {
    SXbus.isr();
} 

void initAccessoire() {
    // Inputs for occupiance detection inputs
    pinMode(Occupiance0, INPUT_PULLUP);
    pinMode(Occupiance1, INPUT_PULLUP);
    pinMode(Occupiance2, INPUT_PULLUP);
    pinMode(Occupiance3, INPUT_PULLUP);
    pinMode(Occupiance4, INPUT_PULLUP);
    pinMode(Occupiance5, INPUT_PULLUP);
    pinMode(Occupiance6, INPUT_PULLUP);
    pinMode(Occupiance7, INPUT_PULLUP);

    pinMode(Relais, OUTPUT);
    digitalWrite(Relais, LOW);               // Relais off
    // Init internal variables
    Occupiance = 0;                          // No trains
    Loops = 0L;
}

// Every thing the program has to do without actions on the SXbus
void processAccessoire() {
    byte temp;

    // Read all occupaincseses
    bitWrite(temp, 0, digitalRead(Occupiance0));  
    bitWrite(temp, 1, digitalRead(Occupiance1));
    bitWrite(temp, 2, digitalRead(Occupiance2));
    bitWrite(temp, 3, digitalRead(Occupiance3));
    bitWrite(temp, 4, digitalRead(Occupiance4));
    bitWrite(temp, 5, digitalRead(Occupiance5));
    bitWrite(temp, 6, digitalRead(Occupiance6));
    bitWrite(temp, 7, digitalRead(Occupiance7));
    // Set value to report
    Occupiance |= ~temp;
    Loops++;
    
    // Control relais
    if ((Occupiance & MaskDE) == 0) {
        if ((Occupiance & MaskAB) > 0) {
            digitalWrite(Relais, HIGH);    
        }
    } else {
        digitalWrite(Relais, LOW); 
    }
}

// Every thing the program has to do on the SX-bus
void powerAccessoire() {
    // Write "occupiance" to SXbus and reset
    byte temp = Occupiance;
    // Check if AB and DE must be reported as one
    if (ReportDouble) {
        // If A or B occipied then report both
        if ((Occupiance & MaskAB) != 0) {
            temp |= MaskAB;
        }
        // If D or E occipied then report both
        if ((Occupiance & MaskDE) != 0) {
            temp |= MaskDE;
        }
    }
    SXbus.write(Address, temp ^ Invert);
    SXbus.write(Loops, 35);
    Loops /= 256;     // Shift a byte to the right
    SXbus.write(Loops, 36);
    Loops /= 256;     // Shift a byte to the right
    SXbus.write(Loops, 37);
    Loops /= 256;     // Shift a byte to the right
    SXbus.write(Loops, 38);
    
    // Reset all detekted occupances
    Occupiance = 0;
    Loops = 0;
}

// Prepare SX-bus for programming
void startprogAccessoire() {
    // Write all variables to the SXbus to be changed by the user
    byte temp = Address;
    if (ReportDouble) {
        temp += 128;          // if ReportDouble true, then set bit 8
    }
    SXbus.write(SXAddress, temp);
    SXbus.write(SXInvert, Invert);
    SXbus.write(SXMaskAB, MaskAB);
    SXbus.write(SXMaskDE, MaskDE);
    SXbus.write(SXVersion, Version);
}

// Wait for user to end programming
void progAccessoire() {
    // Read all variables from the SXbus
    byte temp = SXbus.read(SXAddress);
    ReportDouble = (temp > 128);
    Address = temp & 0x7F;
    Invert = SXbus.read(SXInvert);
    MaskAB = SXbus.read(SXMaskAB);
    MaskDE = SXbus.read(SXMaskDE);
}

// End programming, release SXbus
void endprogAccessoire() {
    // Nothing to do.
}

// EEprom actions
void writeEEprom() {
    // Write all variables to EEprom
    // To save lifetime use Update instead of Write
    byte temp = Address;
    if (ReportDouble) {
        temp += 128;          // if ReportDouble true, then set bit 8
    }
    EEPROM.update(EEAddress, temp); 
    EEPROM.update(EEInvert, Invert); 
    EEPROM.update(EEMaskAB, MaskAB); 
    EEPROM.update(EEMaskDE, MaskDE); 
}

void readEEprom() {
    // Read all variables from EEprom
    byte temp = EEPROM.read(EEAddress);
    ReportDouble = (temp > 128);
    Address = temp & 0x7F;
    Invert = EEPROM.read(EEInvert);
    MaskAB = EEPROM.read(EEMaskAB);
    MaskDE = EEPROM.read(EEMaskDE);
}

void setup() {
    // put your setup code here, to run once:
    // User IO
    pinMode(progLED, OUTPUT);
    pinMode(progKey, INPUT_PULLUP);
  
    // initialize SX-bus
    if (SXbus.init()) {
        // Rising edges on INT0 triggers the interrupt routine sxisr (see above)
        attachInterrupt(0, sxisr, RISING); 
    }
    debounceWait = debounceVal;
    switchPressed = false;
    programming = false;
    dataChanged = false;
    readEEprom();
    digitalWrite(progLED, LOW);                 // LED off
  
    // Init used varables and ports
    initAccessoire();
}

bool debounceSwitch() {
    if (digitalRead(progKey) == 0) {
        // Switch pressed, check if debounced
        if (debounceWait > 0) {
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

/* Action performed to acces the SXbus:
 * getPWR()  programming switchpressed  action
 *    1            *           *        powerAccessoire  
 *    0            0           0        -
 *    0            0           1        startprogAccessoire
 *    0            1           0        progAccessoire
 *    0            1           1        endprogAccessoire
 */

void loop() {
    // put your main code here, to run repeatedly:
    // Read all SXbus addresses
    if (SXbus.inSync() == 1) {
        // Debounce switch
        if (debounceSwitch()) {
            switchPressed = true;                                  // Switch debounced, signal pressed.
        }
        if (SXbus.readPWR() == 1) {
            // Power on the track, perform normal operation
            // Reset all variables to initial state
            switchPressed = false;
            if (programming) {
                SXbus.setProg(false);                               // Report on SX-bus
                programming = false;
            }
            digitalWrite(progLED, LOW);                             // LED off
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
                    // In programming mode and key pressed, terminate programming mode
                    if (switchPressed) {
                        endprogAccessoire();
                        // Write all to EEProm
                        writeEEprom();
                        dataChanged = false;
                        // End programming mode  
                        switchPressed = false;
                        programming = false;
                        SXbus.setProg(false);                       // Report on SX-bus
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
                    SXbus.setProg(false);                           // Report on SX-bus
                    digitalWrite(progLED, LOW);                     // LED off
                }
            } else {
                if (switchPressed) {
                    if (SXbus.claimProg()) {
                        // No power on the track, no programming on SX-bus and key pressed, start programming mode
                        startprogAccessoire();
                        // Start programming mode
                        programming = true;
                        digitalWrite(progLED, HIGH);                // LED on
                    }
                    switchPressed = false;  
                }
            }
        }
    }
    // Perform all action to let the accessoire work
    processAccessoire();
}
