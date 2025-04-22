#include <ADF5355.h>

// Instantiate the synthesizer object
ADF5355 rfSynth;

const uint8_t ADF_CS_PIN = 10;        // Chip select pin connected to LE of ADF5355

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for serial port (if using Arduino Zero/Empyrean)

  // Initialize ADF5355: device type = ADF5355, reference = 100 MHz, initial frequency = 1 GHz on channel 0 (RFoutA)
  int32_t status = rfSynth.begin(ADF_CS_PIN, ADF5355_DEVICE, 100000000ULL, 1000000000ULL, 0, 
                                 /* ref divide-by-2 */ false, /* ref doubler */ false,
                                 /* CP current */ 5000, /* negative bleed */ false,
                                 /* gated bleed */ false, /* bleed polarity */ false,
                                 /* MUXOUT = digital lock detect */ ADF5355_MUX_DIGITAL_LD,
                                 /* mute until lock */ false,
                                 /* RFoutA enabled */ true, /* RFoutB enabled */ false,
                                 /* RFoutA power */ 3, /* RFoutB power */ 3);
  if(status != 0) {
    Serial.println("ADF5355 initialization failed!");
    while(1);
  }
  Serial.println("ADF5355 initialized.");

  // Example: change frequency to 2.450 GHz
  uint64_t newFreq = 2450000000ULL;  // 2450 MHz
  if(rfSynth.setFrequency(newFreq) == 0) {
    Serial.print("Frequency set to ");
    Serial.print(newFreq);
    Serial.println(" Hz");
  } else {
    Serial.println("Failed to set frequency (out of range?)");
  }
}

void loop() {
  // The synthesizer is now outputting the set frequency.
  // You could add code here to periodically change frequency or read lock status.
}
