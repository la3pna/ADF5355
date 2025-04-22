/*
   ADF5355_Example.ino - Basic usage example for the ADF5355 Arduino Library
   Sets the synthesizer to 2.45 GHz on RFoutA.
*/

#include <ADF5355.h>

ADF5355 rfSynth;

const uint8_t CS_PIN = 10; // LE pin

void setup() {
  Serial.begin(115200);
  while(!Serial);

  if(rfSynth.begin(CS_PIN, ADF5355_DEVICE, 100000000ULL, 1000000000ULL) != 0) {
    Serial.println("Failed to init ADF5355!");
    while(1);
  }
  Serial.println("ADF5355 init OK. Setting 2.45 GHz...");

  if(rfSynth.setFrequency(2450000000ULL) == 0)
    Serial.println("Frequency set!");
  else
    Serial.println("Error setting frequency.");
}

void loop() {
  // nothing
}
