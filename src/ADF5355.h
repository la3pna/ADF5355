#ifndef ADF5355_H
#define ADF5355_H

#include <Arduino.h>
#include <SPI.h>

// Device ID options (supported synthesizer variants)
enum ADF5355_DeviceID {
    ADF5355_DEVICE = 0,   // ADF5355 (54 MHz to 13.6 GHz)
    ADF4355_DEVICE,       // ADF4355 (55 MHz to 6800 MHz)
    ADF4355_2_DEVICE,     // ADF4355-2 (Min VCO 3.4 GHz)
    ADF4355_3_DEVICE,     // ADF4355-3 (Min VCO 3.3 GHz)
    ADF4356_DEVICE,       // ADF4356 (55 MHz to 6800 MHz, improved)
    ADF5356_DEVICE        // ADF5356 (similar to ADF5355, 55 MHz to 13.6 GHz)
};

// MUX output selection (for MUXOUT pin functionality)
enum ADF5355_MuxOut {
    ADF5355_MUX_TRISTATE,
    ADF5355_MUX_DVDD,
    ADF5355_MUX_GND,
    ADF5355_MUX_R_DIV_OUT,
    ADF5355_MUX_N_DIV_OUT,
    ADF5355_MUX_ANALOG_LD,    // Analog lock detect
    ADF5355_MUX_DIGITAL_LD    // Digital lock detect
};

// Error codes (returns from functions)
const int32_t ADF5355_ERROR_OK       = 0;
const int32_t ADF5355_ERROR_INVALID  = -1;
const int32_t ADF5355_ERROR_SPI      = -2;

// ADF5355 Library Class
class ADF5355 {
public:
    // Constructor (optionally specify SPI settings)
    ADF5355();

    // Initialize the ADF5355 device. 
    // Parameters:
    //    csPin        - Chip select pin number for SPI (latch enable for ADF5355).
    //    deviceId     - Device type (use enum ADF5355_DeviceID, default ADF5355_DEVICE).
    //    refFreq      - Reference clock frequency in Hz (e.g. 100000000 for 100 MHz).
    //    initialFreq  - Initial output frequency in Hz (default 0, can be set later).
    //    channel      - Output channel to use for initialFreq (0 = RFoutA, 1 = RFoutB).
    //    clkinDiv2    - Set true to enable divide-by-2 on reference input (default false).
    //    refDoubler   - Set true to enable reference doubler (default false).
    //    cpCurrent_uA - Charge pump current in microamps (default 5000 µA).
    //    cpNegBleed   - Enable negative bleed current (default false).
    //    cpGatedBleed - Enable gated bleed (default false).
    //    cpBleedPol   - Bleed current polarity (true for positive polarity, default false).
    //    muxOutSel    - MuxOut pin selection (default digital lock detect).
    //    muteUntilLock- Mute RF output until lock (default false).
    //    outAEnable   - Enable RFoutA output (default true).
    //    outBEnable   - Enable RFoutB output (default false).
    //    outAPower    - RFoutA power level (0–3, higher = higher power, default 3).
    //    outBPower    - RFoutB power level (0–3, for devices with adjustable RFoutB, default 3).
    // Returns 0 on success, negative on error.
    int32_t begin(uint8_t csPin, ADF5355_DeviceID deviceId = ADF5355_DEVICE, 
                  uint64_t refFreq = 100000000ULL, uint64_t initialFreq = 0ULL, uint8_t channel = 0,
                  bool clkinDiv2 = false, bool refDoubler = false,
                  uint32_t cpCurrent_uA = 5000, bool cpNegBleed = false,
                  bool cpGatedBleed = false, bool cpBleedPol = false,
                  ADF5355_MuxOut muxOutSel = ADF5355_MUX_DIGITAL_LD,
                  bool muteUntilLock = false,
                  bool outAEnable = true, bool outBEnable = false,
                  uint8_t outAPower = 3, uint8_t outBPower = 3);

    // Set the output frequency (in Hz) on the specified channel (0 = A, 1 = B).
    // Returns 0 on success, negative error code if frequency is out of range.
    int32_t setFrequency(uint64_t frequency, uint8_t channel = 0);

    // Get the current output frequency (in Hz) of the specified channel.
    // (This value is tracked internally after calls to setFrequency)
    uint64_t getFrequency(uint8_t channel = 0) const;

    // Calculate the closest achievable frequency to the requested value.
    // `desiredFreq` in, returns `roundedFreq` as the nearest supported frequency.
    // Always returns 0 (since rounding always succeeds).
    int32_t roundFrequency(uint64_t desiredFreq, uint64_t &roundedFreq) const;

    // Shut down the device and release SPI (optional to call). Returns 0 on success.
    int32_t end();

private:
    // Internal SPI helper to write a 32-bit value to a register
    void writeRegister(uint8_t regAddr, uint32_t regValue);

    // Internal PLL parameter computation for fractional-N (populates integer, fract1, fract2, mod2)
    void computePLL(uint64_t vcoFreq, uint32_t pfdFreq, uint32_t maxMod2,
                    uint32_t &intVal, uint32_t &frac1, uint32_t &frac2, uint32_t &mod2);

    // Internal register update routine (writes registers to device).
    // If fullSync is true or first run, writes all registers; otherwise writes only relevant ones.
    int32_t updateRegisters(bool fullSync);

    // Device configuration state (mirrors registers and settings)
    ADF5355_DeviceID _deviceId;
    uint8_t   _csPin;
    bool      _initialized;
    // Reference configuration
    uint32_t  _refFreq;       // reference input frequency (Hz)
    bool      _refDiv2En;     // reference divide-by-2
    bool      _refDoublerEn;  // reference frequency doubler
    // Device capabilities
    uint64_t  _minVcoFreq;
    uint64_t  _maxVcoFreq;
    uint64_t  _minOutFreq;
    uint64_t  _maxOutFreq;
    // Current state
    uint32_t  _fpfd;          // PFD frequency (phase detector freq)
    uint16_t  _rCounter;      // Reference division factor
    bool      _allRegsSynced; // whether registers have been initially synced
    // Desired output state
    uint64_t  _freqReq[2];    // Requested frequency for each channel (A=0, B=1)
    // Output settings
    bool      _outEnable[2];  // Output enables for A and B
    uint8_t   _outPower[2];   // Output power setting (0-3)
    bool      _outBselFund;   // (for ADF4356) output B fundamental mode
    // PLL config
    uint32_t  _cpCurrent_uA;  // Charge pump current (µA)
    bool      _cpNegBleedEn;
    bool      _cpGatedBleedEn;
    bool      _cpBleedPolEn;
    bool      _muteTillLockEn;
    uint8_t   _phaseDetPol;   // 0=positive, 1=negative (phase detector polarity)
    bool      _refDiff;       // differential reference input enabled
    ADF5355_MuxOut _muxOutSel;
    // Internal registers mirror (14 registers, 0–13)
    static const uint8_t _REG_COUNT = 14;
    uint32_t _regs[_REG_COUNT];
};

#endif
