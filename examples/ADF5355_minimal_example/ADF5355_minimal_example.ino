/*
  ADF5355_minimal_example.ino
  --------------------------------------------------------
  Generates 10 242 MHz on RFoutB (channel 1) of the ADF5355.

  Hardware:
    * Arduino Zero / any 3.3 V Arduino (use level shifters if 5 V)
    * ADF5355 module/eval‑board
    * Reference clock: 100 MHz single‑ended to REFIN
    * Connect LE (chip‑select) to CS_PIN below, DATA to MOSI, CLK to SCK,
      CE tied high, GNDs/common 3.3 V supplies.

  Library:  ADF5355 Arduino Library
*/

#include <ADF5355.h>

ADF5355 synth;

const uint8_t CS_PIN      = 10;              // LE pin
const uint64_t REF_FREQ   = 100000000ULL;    // 100 MHz reference
const uint64_t TARGET_FREQ = 10242000000ULL; // 10 242 MHz (RFoutB)

void setup()
{
  Serial.begin(115200);
  while (!Serial) ;  // wait for USB (for boards with native USB)

  // Initialise synth without tuning yet (initialFreq = 0)
  if (synth.begin(CS_PIN,
                  ADF5355_DEVICE,
                  REF_FREQ,
                  0ULL,          // no initial frequency
                  1,             // placeholder channel
                  false, false,  // no ref ÷2, no ref doubler
                  5000,          // 5 mA charge‑pump
                  false, false, false,
                  ADF5355_MUX_DIGITAL_LD,
                  false,         // don’t mute until lock
                  false,         // disable RFoutA
                  true,          // enable RFoutB
                  3, 3) != 0)    // full power
  {
    Serial.println(F("ADF5355 init failed"));
    while (1);
  }
  Serial.println(F("ADF5355 initialised"));

  // Tune RFoutB to 10.242 GHz
  if (synth.setFrequency(TARGET_FREQ, 1) == 0)
    Serial.println(F("RFoutB = 10 242 MHz"));
  else
    Serial.println(F("Frequency out of range or error"));
}

void loop()
{
  // nothing – PLL free‑runs once locked
}
