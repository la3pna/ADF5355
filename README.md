# ADF5355 Arduino Library

This library allows Arduino boards to program the Analog Devices ADF5355 wideband frequency synthesizer.

See the example sketch in `examples/ADF5355_Example/` for basic usage.

This is a Arduino port of https://github.com/analogdevicesinc/no-OS/tree/main/drivers/frequency/adf5355

By LA3PNA - 22/4-2025

# ADF5355 Arduino Library

This library allows Arduino-compatible boards to control the **Analog Devices ADF5355 wideband PLL frequency synthesizer**, as well as related devices (ADF4355, ADF4356, ADF5356). The ADF5355 can generate frequencies from 54 MHz up to 13.6 GHz using an integrated VCO and programmable dividers. The library provides a high-level interface to configure the device and set output frequencies.

## Features

- **Supported Devices:** ADF5355, ADF5356, ADF4355 (and -2, -3 variants), ADF4356. Select the device type in the `begin()` method.
- **Dual Outputs:** Control RFoutA (up to ~6.8 GHz with divisors for lower frequencies) and RFoutB (doubler output for up to 13.6 GHz on ADF5355/5356, or second output on ADF4356). You can enable/disable each output and set its power level.
- **SPI Interface:** Uses Arduino `<SPI.h>` for hardware SPI. Clock up to 10 MHz by default (chip supports up to 50 MHz SPI. Ensure 3.3V logic for the ADF5355 (use a level shifter if on 5V Arduino).
- **Easy Configuration:** Initialize with `begin(csPin, device, refFreq, ...)`. All major parameters (reference doubler/div2, charge pump current, bleed, muxout, etc.) are configurable through `begin` arguments. 
- **Frequency Control:** Set output frequency in hertz via `setFrequency(freq, channel)`. The library handles the integer/fractional N calculation and writes all necessary registers. You can query the current frequency with `getFrequency()`.
- **Lock Detect:** You can route the lock-detect to the MUXOUT pin (default configuration). The library doesn’t directly read lock status (the ADF5355’s LD is an output pin), but you can configure MUXOUT to Digital Lock Detect and read that pin if wired to a microcontroller input.
- **Power Down and Reset:** The `end()` method disables the outputs and powers down the synthesizer (useful to place the device in a low-power state).

## Installation

1. Download or clone this library into your Arduino libraries folder.
2. Include the library in your sketch with `#include <ADF5355.h>`.

## Usage

```cpp
#include <ADF5355.h>
ADF5355 synth;

void setup() {
  synth.begin(10, ADF5355_DEVICE, 100000000ULL, 1000000000ULL); 
  // Parameters: CS pin =10, device=ADF5355, 100 MHz ref, start at 1 GHz (on RFoutA by default).
  
  synth.setFrequency(2450000000ULL); // change frequency to 2450 MHz
}
void loop() { }
