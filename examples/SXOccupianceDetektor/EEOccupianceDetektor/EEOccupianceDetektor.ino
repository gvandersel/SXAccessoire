#include <Arduino.h>
#include <EEPROM.h>
#include "EEOccupianceDetektor.h"

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  
  EEPROM.update(EEAddress, 40); 
  EEPROM.update(EEInvert, 0x00); 
  EEPROM.update(EEMaskAB, 0x03); 
  EEPROM.update(EEMaskDE, 0x18); 
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}
